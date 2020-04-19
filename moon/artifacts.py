"""
  Detect Cube Sat and Processing Plant artifacts
"""
from datetime import timedelta
import os
from io import StringIO

import cv2
import numpy as np

from osgar.node import Node
from osgar.bus import BusShutdownException


PROCESSING_PLANT = 'ProcessingPlant'
CUBE_SAT = 'CubeSat'


RED_THRESHOLD = 100
YELLOW_THRESHOLD = 100


g_mask = None


def count_mask(mask):
    """Count statistics and bounding box for given image mask"""
    count = int(mask.sum())
    if count == 0:
        return count, None, None, None, None

    # argmax for mask finds the first True value
    x_min = (mask.argmax(axis=0) != 0).argmax()
    x_max = mask.shape[1] - np.flip((mask.argmax(axis=0) != 0), axis=0).argmax() - 1
    w = (mask.shape[1] - np.flip((mask.argmax(axis=0) != 0), axis=0).argmax()
            - (mask.argmax(axis=0) != 0).argmax())
    h = (mask.shape[0] - np.flip((mask.argmax(axis=1) != 0), axis=0).argmax()
            - (mask.argmax(axis=1) != 0).argmax())
    return count, w, h, x_min, x_max


def count_red(img, filtered=False, stdout=None):
    # well rather "orange" of the "Processing Plant"
    # 192 61 7
    # 185 59 8
    # 193 63 14
    # TODO dark side of the Moon
    b = img[:,:,0]
    g = img[:,:,1]
    r = img[:,:,2]
    mask = np.logical_and(r > 150, np.logical_and(r/3 > g, r/10 > b))
    not_mask = np.logical_not(mask)
    img2 = img.copy()
    img2[mask] = (255, 255, 255)
    img2[not_mask] = (0, 0, 0)

    global g_mask
    g_mask = mask.copy()
    return count_mask(mask)


def count_yellow(img):
    # 146 113 34
    # 170 137 60
    b = img[:,:,0]
    g = img[:,:,1]
    r = img[:,:,2]
    mask = np.logical_and(np.logical_and(r >= 100, g > 0.7 * r), np.logical_and(r*0.5 > b, g*0.5 > b))
    global g_mask
    g_mask = mask.copy()
    # debug
#    img2 = img.copy()
#    img2[mask] = (0, 255, 0)
#    cv2.imwrite('artf.jpg', img2)
    return count_mask(mask)


class ArtifactDetector(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("artf", "dropped", "debug_artf")
        self.best = None
        self.best_count = 0
        self.best_img = None
        self.best_info = None
        self.best_scan = None
        self.best_depth = None
        self.verbose = False
        self.dump_dir = None  # optional debug ouput into directory
        self.scan = None  # should laster initialize super()
        self.depth = None  # more precise definiton of depth image
        self.width = None  # detect from incoming images

    def stdout(self, *args, **kwargs):
        # maybe refactor to Node?
        output = StringIO()
        print(*args, file=output, **kwargs)
        contents = output.getvalue().strip()
        output.close()
#        self.publish('stdout', contents)
        print(contents)

    def waitForImage(self):
        channel = ""
        while channel != "left_image":  # TODO handle right image
            self.time, channel, data = self.listen()
            setattr(self, channel, data)
        self.image = self.left_image
        return self.time

    def run(self):
        try:
            dropped = 0
            while True:
                now = self.publish("dropped", dropped)
                dropped = -1
                timestamp = now
                while timestamp <= now:
                    timestamp = self.waitForImage()
                    dropped += 1
                self.detect(self.image)
        except BusShutdownException:
            pass

    def detect(self, image):
        img = cv2.imdecode(np.fromstring(image, dtype=np.uint8), 1)
        if self.width is None:
            self.stdout('Image resolution', img.shape)
            self.width = img.shape[1]
        assert self.width == img.shape[1], (self.width, img.shape[1])
        rcount, w, h, x_min, x_max = count_red(img)
        yellow_used = False
        if rcount < 20:
            ycount, w, h, x_min, x_max = count_yellow(img)
            if ycount > YELLOW_THRESHOLD:
                yellow_used = True
                count = ycount
            else:
                count = rcount
        else:
            count = rcount

        if self.verbose and count >= 20:
            print(self.time, img.shape, count, w, h, x_min, x_max, w/h, count/(w*h))
        if self.best_count > 0:
            self.best_count -= 1
        if count > RED_THRESHOLD or (yellow_used and count > YELLOW_THRESHOLD):
            if self.best is None or count > self.best:
                self.best = count
                self.best_count = 10
                self.best_img = self.image
                self.best_info = w, h, x_min, x_max, (count == rcount), yellow_used  # RED used
                self.best_scan = self.scan
                self.best_depth = self.depth

        if self.best is not None and self.best_count == 0:
            w, h, x_min, x_max, red_used, yellow_used = self.best_info

            deg_100th, dist_mm = 0, 500  # TODO left & right

            if red_used:
                artf = PROCESSING_PLANT  # BACKPACK
            elif yellow_used:
                artf = CUBE_SAT  # RESCUE_RANDY  # used to be RADIO
            self.stdout(self.time, 'Relative position:', self.best, deg_100th, dist_mm, artf)

            dx_mm, dy_mm = 0, 0  # relative offset to current robot position
            # TODO if VALVE -> find it in scan
            self.publish('artf', [artf, deg_100th, dist_mm])
            self.publish('debug_artf', self.best_img)
            if self.dump_dir is not None:
                filename = 'artf_%s_%d.jpg' % (artf, self.time.total_seconds())
                with open(os.path.join(self.dump_dir, filename), 'wb') as f:
                    f.write(self.best_img)
                if self.best_depth is not None:
                    filename = 'artf_%s_%d.npz' % (artf, self.time.total_seconds())
                    np.savez_compressed(os.path.join(self.dump_dir, filename), depth=self.best_depth)

            # reset detector
            self.best = None
            self.best_count = 0
            self.best_img = None
            self.best_info = None
            self.best_scan = None
            self.best_depth = None


def debug2dir(filename, out_dir):
    from osgar.logger import LogReader, lookup_stream_names
    from osgar.lib.serialize import deserialize

    names = lookup_stream_names(filename)
    assert 'detector.debug_artf' in names, names
    assert 'detector.artf' in names, names
    assert 'rosmsg.sim_time_sec' in names, names
    image_id = names.index('detector.debug_artf') + 1
    artf_id = names.index('detector.artf') + 1
    sim_sec_id = names.index('rosmsg.sim_time_sec') + 1
    sim_time_sec = None
    image = None
    artf = None
    for dt, channel, data in LogReader(filename, only_stream_id=[image_id, artf_id, sim_sec_id]):
        data = deserialize(data)
        if channel == sim_sec_id:
            sim_time_sec = data
        elif channel == image_id:
            image = data
            assert artf is not None
            time_sec = sim_time_sec if sim_time_sec is not None else int(dt.total_seconds())
            name = os.path.basename(filename)[:-4] + '-' + artf[0] + '-' + str(time_sec) + '.jpg'
            print(name)
            with open(os.path.join(out_dir, name), 'wb') as f:
                f.write(image)
        elif channel == artf_id:
            artf = data


if __name__ == '__main__':
    from unittest.mock import MagicMock
    from queue import Queue
    import argparse
    import datetime
    import sys
    from osgar.bus import Bus

    parser = argparse.ArgumentParser(description='Run artifact detection and classification for given JPEG image')
    parser.add_argument('filename', help='JPEG filename')
    parser.add_argument('--debug2dir', help='dump clasified debug images into directory')
    parser.add_argument('-v', '--verbose', help='verbose mode', action='store_true')
    args = parser.parse_args()

    if args.debug2dir is not None:
        debug2dir(args.filename, args.debug2dir)
        sys.exit()

    with open(args.filename.replace('.npz', '.jpg'), 'rb') as f:
        jpeg_data = f.read()

    config = {'virtual_world': True}  # for now
    logger = MagicMock()
    logger.register = MagicMock(return_value=1)
    def counter():
        start = datetime.datetime.utcnow()
        while True:
            dt = datetime.datetime.utcnow() - start
            yield dt
    logger.write = MagicMock(side_effect=counter())
    bus = Bus(logger)
    detector = ArtifactDetector(config, bus.handle('detector'))
    detector.verbose = args.verbose
    tester = bus.handle('tester')
    tester.register('scan', 'left_image', 'right_image', 'tick')
    bus.connect('tester.scan', 'detector.scan')
    bus.connect('tester.left_image', 'detector.left_image')
    bus.connect('tester.right_image', 'detector.right_image')
    bus.connect('detector.artf', 'tester.artf')
    bus.connect('tester.tick', 'tester.tick')
    bus.connect('detector.dropped', 'tester.dropped')
    tester.publish('scan', [2000]*270)  # pretend that everything is at 2 meters
    detector.start()
    for i in range(10 + 1):  # workaround for local minima
        a = tester.listen()
#        print(i, a)
        tester.sleep(0.01)
        tester.publish('left_image', jpeg_data)  # TODO right image
    detector.request_stop()
    detector.join()
    tester.publish('tick', None)
    a = tester.listen()
    print(a)

# vim: expandtab sw=4 ts=4
