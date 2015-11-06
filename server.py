"""
Black Box XML Relay Server
- Accepts packets containing a <Update id="a" modification_version="b"></Update>.
- Bundles them into <Updates modification_version="c" ><Update id="a" modification_version="b"></Update> ... <Update id="y" modification_version="z"></Update></Updates>
- Pushes these bundles to a set of clients.
"""

from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from twisted.internet import task
from lxml import etree
import logging
import socket
from io import BytesIO
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

class XmlRelay(DatagramProtocol):
        def __init__(self):
                self.updates = {}
                self.modication_version = 0
                self.rendered_xml = ""
                self.rendered_version = 0
                self.sendXmlTimer = None
                self.clients = [("192.168.1.50", 1611), ("192.168.1.3", 1611), ("192.168.1.51", 1611)]

        def datagramReceived(self, data, (in_host, in_port)):
                event = None
                root = None
                #print data
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
                if not (update_version > self.updates.get(update_id, (0L, ''))[0]):
                        return
                self.updates[update_id] = (update_version, data)
                self.modication_version += 1
                self.sendXml()

        def sendXml(self):
                # Every 10 milliseconds
                if self.sendXmlTimer and self.sendXmlTimer.active():
                        self.sendXmlTimer.reset(.01)
                else:
                        self.sendXmlTimer = reactor.callLater(.01, self.sendXml)
                if self.modication_version != self.rendered_version:
                        updates_xml = [u[1][1] for u in self.updates.iteritems()]
                        self.rendered_xml = '<Updates modication_version="%i">%s</Updates>' % (self.modication_version, ''.join(updates_xml))
                        self.rendered_version = self.modication_version
                for host, port in self.clients:
                        sock.sendto(self.rendered_xml, (host, port))
                        """
                        try:
                                self.transport.write(self.rendered_xml, (host, port))
                        except socket.error as e:
                                # The host does not need to exist
                                if not (e.errno in []):
                                        raise e
                        """

relay = XmlRelay()
reactor.listenUDP(1610, relay, interface='192.168.1.3', maxPacketSize=65507)
reactor.run()
