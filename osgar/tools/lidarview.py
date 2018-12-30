"""
  Draw laser scans
"""
import math
from datetime import timedelta

import pygame
from pygame.locals import *

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


WINDOW_SIZE = 1200, 660


def scans_gen(logfile, stream_id):
    only_stream = lookup_stream_id(logfile, stream_id)
    with LogReader(logfile) as log:
        for timestamp, stream_id, data in log.read_gen(only_stream):
            scan = deserialize(data)
            yield timestamp, scan


def scr(x, y):
    scale = 30
    return WINDOW_SIZE[0]//2 + round(scale*x), WINDOW_SIZE[1]//2 - round(scale*y)


def draw(foreground, scan):
    color = (0, 255, 0)
    for i, i_dist in enumerate(scan):
        if i_dist == 0 or i_dist >= 10000:
            continue
        angle = math.radians(i * 270 / len(scan))
        dist = i_dist/1000.0
        x, y = dist * math.cos(angle), dist * math.sin(angle)
        pygame.draw.circle(foreground, color, scr(x, y), 3)


def lidarview(gen):
    pygame.init()    
    screen = pygame.display.set_mode(WINDOW_SIZE)

    # create backgroud
    background = pygame.Surface(screen.get_size())
#    background.set_colorkey((0,0,0))

    # create foreground
    foreground = pygame.Surface(screen.get_size())
#    foreground.set_colorkey((0,0,0))

    # display everything
    screen.blit(background, (0, 0))
    screen.blit(foreground, (0, 0))
    pygame.display.flip()

    pygame.key.set_repeat(200, 20)  

    for timestamp, scan in gen:
        if max(scan) == 0:
            continue
        pygame.display.set_caption("Time %s" % timestamp)
        foreground.fill((0, 0, 0))
        draw(foreground, scan)
        event = pygame.event.wait()
        if event.type == QUIT:
            break
        if event.type == KEYDOWN:
            if event.key in [K_ESCAPE, K_q]:
                break

        screen.blit(background, (0, 0))
        screen.blit(foreground, (0, 0))
        pygame.display.flip() 


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='View lidar scans')
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--stream', help='stream ID', default='lidar.scan')
    args = parser.parse_args()

    lidarview(scans_gen(args.logfile, args.stream))

# vim: expandtab sw=4 ts=4 

