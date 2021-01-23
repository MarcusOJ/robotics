import websocket
from threading import Thread
import json

class Client:

    def __init__(self, ws):
        self.go = None
        self.stopped = False
        self.robot = "tulihand"
        self.ws = ws
        self.blue = True


    def start(self):
        Thread(target=self.listen, args=()).start()
        return self

    def listen(self):

        while not self.stopped:
            message = self.ws.recv()
            command = json.loads(message)

            if command["signal"] == "stop" and self.robot in command["targets"]:
                self.go = False
            elif command["signal"] == "start" and self.robot in command["targets"]:
                index = command["targets"].index(self.robot)
                color = command["baskets"][index]
                if color == "blue":
                	self.blue = True
                elif color == "magenta":
                	self.blue = False
                self.go = True
            else:
                pass

    def getter(self):
    	return (self.go, self.blue)

    def stop(self):
        self.stopped = True

