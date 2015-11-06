import socket
import logging

""" SOCKET CONFIGURATION """
# (Port 0 means to select an arbitrary unused port)
# SENDING_IP should be the IP used by this server to send packets
SENDING_IP = "127.0.0.1"
SENDING_PORT = 0
# RECV_HOST should be the IP used by this server to receive packets
RECV_IP = "127.0.0.1"
RECV_PORT = 1609

""" DESTINATION CONFIGURATION """
DEST_IP = "127.0.0.1"
DEST_PORT = 1610

send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
send.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
send.bind((SENDING_IP, SENDING_PORT))

recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
recv.bind((RECV_IP, RECV_PORT))

while True:
	data = recv.recv(65507)
	send.sendto(data, (DEST_IP, DEST_PORT))