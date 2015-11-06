import socket
import struct
from time import time

def cur_milliseconds():
    return time() * 1000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('192.168.1.4', 1611))

i = 0
last_report = cur_milliseconds()
while True:
	a = sock.recv(60000)
	#print a
	i += 1
	now = cur_milliseconds()
	if now - last_report >=1000:
		print i
		i = 0
		last_report = now


"""
while True:
	#print "%d %02f %02f %02f %02f %02f %02f %02f" % struct.unpack("ifffffff", sock.recv(32))
	print sock.recv(1024)

"""