#!/usr/bin/python
"""
  Remote control of John Deere from ROS
  usage:
       ./ros_proxy.py <notes> | [<metalog> [<F>]]
"""

# Echo server program
import socket
import struct

#HOST = ''                 # Symbolic name meaning all available interfaces
#PORT = 50007              # Arbitrary non-privileged port
HOST = '147.32.83.170'
PORT = 50004

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
conn, addr = s.accept()
print('Connected by', addr)
dist = 0
while 1:
    data = conn.recv(1024)
    print(len(data), data)    
    if not data: break
    if len(data) == 10:
        frame = struct.unpack('<BHBHBHB', data)
        print(frame)
        status = 0
        left = 100 + dist
        right = -1000 + dist
        dist += 1
        data = struct.pack('>BBBBiiiiiii', 0xF0, 0xAF, 0xFA, 7,
                status, left, right, 0, 0, 0, 100)

        conn.sendall(data)
    else:
        print('skipping', len(data))
conn.close()


# vim: expandtab sw=4 ts=4 

