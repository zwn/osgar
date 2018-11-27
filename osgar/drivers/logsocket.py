"""
  Wrapper & timestamper of input/out over socket
"""

import socket
import urllib.request
from threading import Thread

from osgar.logger import LogWriter
from osgar.bus import BusShutdownException


class LogSocket:
    def __init__(self, socket, config, bus):
        self.socket = socket

        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.output_thread = Thread(target=self.run_output, daemon=True)

        host = config['host']
        port = config['port']
        self.pair = (host, port)
        if 'timeout' in config:
            self.socket.settimeout(config['timeout'])
        self.bufsize = config.get('bufsize', 1024)

        self.bus = bus
        self.server = config.get('server', False)

    def _send(self, data):
        raise NotImplementedError()

    def start(self):
        self.input_thread.start()
        self.output_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)
        self.output_thread.join(timeout=timeout)

    def run_input(self):
        if self.server:
            print("Waiting ...")
            self.socket.listen(1)
            print("end of listen")
            self.socket, addr = self.socket.accept()
            print('Connected by', addr)
        while self.bus.is_alive():
            if self.pair[1] == 0:
                self.bus.sleep(0.1)
                continue
            try:
                data = self.socket.recv(self.bufsize)
                if len(data) > 0:
                    self.bus.publish('raw', data)
            except socket.timeout:
                pass

    def run_output(self):
        try:
            while True:
                __, __, data = self.bus.listen()
                self._send(data)
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()


class LogTCP(LogSocket):
    def __init__(self, config, bus):
        soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        LogSocket.__init__(self, soc, config, bus)

        if config.get('server', False):
            soc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            soc.bind(self.pair)
        elif self.pair[1] != 0:  # 0 for dynamic assignment
            self.socket.connect(self.pair)

    def _send(self, data):
        if self.pair[1] == 0:
            self.pair = tuple(data)
            self.socket.connect(self.pair)
        else:
            self.socket.send(data)


class LogUDP(LogSocket):
    def __init__(self, config, bus):
        soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        LogSocket.__init__(self, soc, config, bus)
        self.socket.bind(('', self.pair[1]))

    def _send(self, data):
        self.socket.sendto(data, self.pair)


class LogHTTP:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)

        self.url = config['url']
        self.bus = bus

    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        while self.bus.is_alive():
            try:
                with urllib.request.urlopen(self.url) as f:
                    data = f.read()
                if len(data) > 0:
                    self.bus.publish('raw', data)
            except socket.timeout:
                pass

    def request_stop(self):
        self.bus.shutdown()


if __name__ == "__main__":
    import time
    from osgar.bus import BusHandler

    config = {'host':'localhost', 'port': 8001, 'timeout': 1.0}
    log = LogWriter(prefix='test-tcp-')
    device = LogTCP(config, bus=BusHandler(log, out={'raw':[]}))
    device.start()
    time.sleep(2)
    device.request_stop()
    device.join()

# vim: expandtab sw=4 ts=4
