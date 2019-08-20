"""
  Communication module for vESC - open source BLDC motor driver
  https://www.vandaelectronics.com/collections/electronic-speed-controls/products/vesc
  https://github.com/vedderb/bldc

  There are several ways how to communicate with the device and here
  we work with CAN bus. The motors uses extended addresing mode (24bits)
  and combines command with device identification into the address.

  At the moment only one type of message is decoded and that is "read buffer".
  The device responds with provided "last_message_id" encoded in the address.
  As we need to communicate with four BLDC motors we abuse this "last_message_id"
  for motor identification, so to query motor #1 we set "last_message_id" to 1 etc.
  The reponse is then collection of several CAN messages with address 0x5nn, where nn
  is the motor number, i.e. 0x501 for motor #1.

  Each 0x5nn message contains offset byte in final buffer and seven data bytes (CAN messages
  are limited to 8 bytes only). 

  The transmission is terminated with 0x7nn message, which carries control chechsum and
  total length of already transmitted data.

  Note that the status structure varies among versions and now we are experimenting with
  version 3.40 only.

  For more info see:
     https://robotika.cz/articles/reviews/vesc
"""

import struct
import math
from datetime import timedelta

from osgar.node import Node
from osgar.bus import BusShutdownException


def draw_diff(arr):
    import matplotlib.pyplot as plt
    motors = sort_enc(arr)
    
#    t = [a[0] for a in arr]
#    values = [a[1:] for a in arr]   
    labels = ["m1", "m2", "m3", "m4"] 
    for ii, motor in enumerate(motors):
        plt.plot(motor[:, 0], motor[:, 1], '-o', label = labels[ii])
    plt.legend()
    plt.xlabel('time (s)')
    plt.show()


def sort_enc(arr):
    import numpy as np
    motors = []
    start_enc = [None, None, None, None]
    for ii in range(1,5):
        motor = []
        for a in arr:
            if a[ii] is None:
                continue
            if start_enc[ii-1] is None:
                start_enc[ii-1] = a[ii]
            motor.append([a[0], ( a[ii] - start_enc[ii-1] ) *0.845/100])
        motor = np.array(motor)
        #print(motor.shape)
        motors.append(motor)
    return motors


def vesc_odometry(arr, artf_time = None):
    import numpy as np
    step = 0.5
    start_time = 2.0
    motors = sort_enc(arr)
    motors2 = []
    for motor in motors:
        actual_time = start_time
        actual_dist = 0
        motor2 = [ [ actual_time, actual_dist ] ]
        for motor_time, motor_dist in motor:
            if motor_time > actual_time:
                actual_time += step
                actual_dist = motor_dist
                motor2.append( [actual_time, actual_dist] )
                if artf_time and artf_time < actual_time:
                    break
        motors2.append( np.array(motor2) )
    """
    import matplotlib.pyplot as plt
    for data in motors2:
        plt.plot(data[:, 0], data[:, 1], "+")
    plt.show()
    """
    
    m1, m2, m3, m4 = motors2
    mL = m4[:, 1]
    mR = m3[:, 1]
    N = min( len(mL), len(mR) )
    pose2d = []
    x = 0
    y = 0
    heading = 0
    pose2d.append( [x, y, heading] )
    for ii in range(1, N):
        mL_diff = mL[ii] - mL[ii-1]
        mR_diff = mR[ii] - mR[ii-1]
        
        dist = (mL_diff + mR_diff)/2.0
#        print(dist)
        angle = (mR_diff - mL_diff)/0.496

        # advance robot by given distance and angle
        if abs(angle) < 0.0000001:  # EPS
            # Straight movement - a special case
            x += dist * math.cos(heading)
            y += dist * math.sin(heading)
            #Not needed: heading += angle
        else:
            # Arc
            r = dist / angle
            x += -r * math.sin(heading) + r * math.sin(heading + angle)
            y += +r * math.cos(heading) - r * math.cos(heading + angle)
            heading += angle # not normalized
        pose2d.append( [x, y, heading] )
        #print(x, y, heading)
    print(pose2d[-1])
    pose2d = np.array(pose2d)
    
    f = open("vesc.txt", "w")
    t = start_time
    for ii in range(N):
        f.write( str( [t, pose2d[ii, 0], pose2d[ii, 1], pose2d[ii, 2] ] ) +"\r\n" )
        t += step
    f.close()

    import matplotlib.pyplot as plt
    pose2d = np.array(pose2d)
    plt.plot(pose2d[:,0], pose2d[:, 1], "+")
    plt.axis('equal')
    plt.show()


class MotorDriverVESC(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.debug_arr = []
        self.prev = {}
        self.verbose = False  # TODO move into Node
        self.packet = [[], [], [], []]
        self.tachometer = [None, None, None, None]  # for motors #1, #2, #3 and #4

    def update(self):
        channel = super().update()
        assert channel == 'can', channel
        msg_id, data, flags = self.can  # via update()
        if (msg_id & 0xF00) == 0x500:
            motor_index = (msg_id & 0xFF) - 1  # reindexed motors from #1 to array index 0
            assert data[0] % 7 == 0, data[0]
            self.packet[motor_index].extend(data[1:])
            # the structure for version 3.40 has 59 bytes and looks like:
            # https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L166-L185
            """
     case COMM_GET_VALUES:
     ind = 0;
 0   send_buffer[ind++] = COMM_GET_VALUES;
 1   buffer_append_float16(send_buffer, mc_interface_temp_fet_filtered(), 1e1, &ind);
 3   buffer_append_float16(send_buffer, mc_interface_temp_motor_filtered(), 1e1, &ind);
 5   buffer_append_float32(send_buffer, mc_interface_read_reset_avg_motor_current(), 1e2, &ind);
 9   buffer_append_float32(send_buffer, mc_interface_read_reset_avg_input_current(), 1e2, &ind);
13   buffer_append_float32(send_buffer, mc_interface_read_reset_avg_id(), 1e2, &ind);
17   buffer_append_float32(send_buffer, mc_interface_read_reset_avg_iq(), 1e2, &ind);
21   buffer_append_float16(send_buffer, mc_interface_get_duty_cycle_now(), 1e3, &ind);
23   buffer_append_float32(send_buffer, mc_interface_get_rpm(), 1e0, &ind);
27   buffer_append_float16(send_buffer, GET_INPUT_VOLTAGE(), 1e1, &ind);
29   buffer_append_float32(send_buffer, mc_interface_get_amp_hours(false), 1e4, &ind);
33   buffer_append_float32(send_buffer, mc_interface_get_amp_hours_charged(false), 1e4, &ind);
37   buffer_append_float32(send_buffer, mc_interface_get_watt_hours(false), 1e4, &ind);
41   buffer_append_float32(send_buffer, mc_interface_get_watt_hours_charged(false), 1e4, &ind);
45   buffer_append_int32(send_buffer, mc_interface_get_tachometer_value(false), &ind);
49   buffer_append_int32(send_buffer, mc_interface_get_tachometer_abs_value(false), &ind);
53   send_buffer[ind++] = mc_interface_get_fault();
54   buffer_append_float32(send_buffer, mc_interface_get_pid_pos_now(), 1e6, &ind);
58   send_buffer[ind++] = app_get_configuration()->controller_id;
59
commands_send_packet(send_buffer, ind);
"""
            # the "tachometer" data are stored as 32bit signed integer starting
            # from 45th byte. Because the data blocks contain 7 bytes we are
            # interested in block with "address pointer" set to 42.
            # Then we do not have to re-contruct all four independent bufferes
            # but only "pick" the interesting sub-packet. Yes, there is a danger
            # that checksum of the whole buffer was not valid!
            if data[0] == 42:  # optimization - pick only this packet part
                b = bytes(data[1:])
                prev = self.tachometer[motor_index]
                self.tachometer[motor_index] = struct.unpack_from('>i', b, 3)[0]
                if prev != self.tachometer[motor_index] and self.verbose:
                    print(self.time, hex(msg_id), self.tachometer[motor_index])

                tmp = [None] * 4
                tmp[motor_index] = self.tachometer[motor_index]
                if self.verbose:
                    self.debug_arr.append([self.time.total_seconds()] + tmp)
        if (msg_id & 0xF00) == 0x700:
            motor_index = (msg_id & 0xFF) - 1  # reindexed motors from #1 to array index 0
            if len(self.packet[motor_index]) == 19:  # version
                print(self.time, self.packet[motor_index])
            self.packet[motor_index] = []
        if self.verbose > 1:
            print(self.time, hex(msg_id), list(data), flags)
        return channel

    def run(self):
        try:
            self.update()  # initialize self.time
            while True:
                # example to query all variables
                # COMM_GET_VALUES=4
                for rx_buffer_last_id in [1, 2, 3, 4]:
                    self.publish('can', [0x800 + rx_buffer_last_id, bytes([rx_buffer_last_id, 0, 4]), 1])

                start_time = self.time
                count = 0
                while self.time - start_time < timedelta(seconds=0.1):
                    self.update()
                    msg_id = self.can[0]
                    if (msg_id & 0xF00) == 0x700:
                        count += 1
                        if count >= 4:
                            break
                if count != 4 and self.verbose:
                    print(self.time, "vESC: incomplete loop")

        except BusShutdownException:
            pass


    def draw(self):
        """
        Debug Draw
        """
        vesc_odometry(self.debug_arr)
#        draw_diff(self.debug_arr)


# vim: expandtab sw=4 ts=4
