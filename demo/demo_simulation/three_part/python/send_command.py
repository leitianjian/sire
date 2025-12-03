from struct import *
import websocket
import time

class Msg:
    def __init__(self, buffer):
        self.buffer = buffer
        temp = bytearray(self.buffer)
        self.size, self.id, self.type, self.reserved1, self.reserved2, self.reserved3 = unpack("!IIdddd", temp[:40])
        self.data = temp[40:]
        # self.jsData = json.loads(self.data)
    
    def make(str, header=None):
        if (header is None):
            header = bytearray(40)
        header[0] = len(str)
        data = bytearray(40 + len(str))
        data[0:40] = header
        str_array = bytearray()
        str_array.extend(map(ord, str))
        data[40:] = str_array
        return Msg(data)

class Command:
   def __init__(self, msg, callback):
      self.msg = msg
      self.callback = callback


class RobotSocket:
    def __init__(self, ws):
        self.websocket = ws
        self.cmd_map = dict()
        self.cmd_reserved1 = 1000
    
    def connect(self):
        self.cmd_map = dict();
        self.cmd_reserved1 = time.time();

    def sendCmd(self, cmdStr, successCallback=None):
        print("send msg", cmdStr)
        msg = Msg.make(cmdStr)
        msg.reserved1 = time.time()
        if (msg.reserved1 <= self.cmd_reserved1):
          msg.reserved1 = self.cmd_reserved1 + 1
        self.cmd_reserved1 = msg.reserved1
        try: 
            self.websocket.send(msg.buffer)
            successCallback(Msg(self.websocket.recv()))
        except:
           print("send failed")

