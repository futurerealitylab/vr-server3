import socket
import threading
import SocketServer
from time import time
from lxml import etree
import logging
from io import BytesIO

# (Port 0 means to select an arbitrary unused port)
# SENDING_IP should be the IP used by this server to send packets
SENDING_IP = "127.0.0.1"
SENDING_PORT = 0
# RECV_HOST should be the IP used by this server to receive packets
RECV_IP = "127.0.0.1"
RECV_PORT = 1610

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((SENDING_IP, SENDING_PORT))
updates = {}
modification_version = 0
rendered_xml = ""
rendered_version = 0
sendTimer = None
clients = [("127.0.0.1", 1611)]

def send_xml():
	global sendTimer, sock, updates, modification_version, rendered_xml, rendered_version, clients
	if sendTimer:
		sendTimer.cancel()

	if modification_version != rendered_version:
		updates_xml = [u[1][1] for u in updates.iteritems()]
		rendered_xml = '<Updates modification_version="%i">%s</Updates>' % (modification_version, ''.join(updates_xml))
		rendered_version = modification_version

	for host, port in clients:
		sock.sendto(rendered_xml, (host, port))
	sendTimer = threading.Timer(0.01, send_xml)
	sendTimer.start()

class ThreadedUDPRequestHandler(SocketServer.BaseRequestHandler):
	def handle(self):
		global modification_version, updates
		data = self.request[0]
		print data
		event = None
		root = None
		try:
			context = etree.iterparse(BytesIO(data), events=("start",))
			context = iter(context)
			event, root = context.next()
		except etree.XMLSyntaxError:
			logging.error(data)
			return
		if root.tag != "Update":
			logging.error("%s:%s bad XML in packet" % (in_host, in_port))
			return
		update_id = root.get('id')
		update_version = long(root.get('modification_version', 0))
		if not (update_version > updates.get(update_id, (0L, ''))[0]):
			return
		updates[update_id] = (update_version, data)
		modification_version += 1
		send_xml()

class ThreadedUDPServer(SocketServer.ThreadingMixIn, SocketServer.UDPServer):
	# TODO allow_reuse_address
	pass

if __name__ == "__main__":
	server = ThreadedUDPServer((RECV_HOST, RECV_PORT), ThreadedUDPRequestHandler)
	ip, port = server.server_address

	# Start a thread with the server -- that thread will then start one
	# more thread for each request
	server_thread = threading.Thread(target=server.serve_forever)

	# Exit the server thread when the main thread terminates
	server_thread.daemon = True
	server_thread.start()
	print "Server loop running in thread:", server_thread.name

	sendTimer = threading.Timer(0.01, send_xml)
	sendTimer.start()

	raw_input("(press enter to quit) ")
	sendTimer.cancel()
	server.shutdown()