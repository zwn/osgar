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
while 1:
    data = conn.recv(1024)
    print(len(data), data)    
    if not data: break
    assert len(data) == 10, len(data)
    frame = struct.unpack('>BHBHBHB', data)
    print(frame)
    conn.sendall(data)
conn.close()


# vim: expandtab sw=4 ts=4 

