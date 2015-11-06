from twisted.internet.protocol import DatagramProtocol, Protocol
from twisted.internet import reactor
from twisted.internet import stdio
from twisted.internet import task
from lxml import etree
import logging
from io import BytesIO
import time


# Receive XML from Chrome
class ChromeMessage(Protocol):
  def __init__(self):
    self.sendXmlTimer = None
    self.clients = [("192.168.1.2", 1610)]
    self.frame_text = None
    self.text_data = ""
    self.text_length = 0

  def dataReceived(self, data):
    print "RECEIVED"
    """
    # Read the header
    if not self.text_length:
      text_length_bytes = data[:4]
      self.text_length = struct.unpack('i', text_length_bytes)[0]
      data = data[4:]
    
    # Read the text (JSON object) of the message.
    self.frame_text += data[:self.text_length].decode('utf-8')
    self.text_length -= len(data[:self.text_length])

    # Was an entire message received?
    if self.text_length == 0:
      print self.frame_text
      send_message('{"echo": %s}' % self.frame_text)
      self.frame_text = ""

    # Read the rest
    data = data[self.text_length:]
    if data:
      self.dataReceived(data)
    """

stdio.StandardIO(ChromeMessage())
reactor.run(installSignalHandlers=0)