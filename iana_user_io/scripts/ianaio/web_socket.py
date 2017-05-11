#TODO This code is a mess! I (Beni) am so sorry and will clean it up later

from autobahn.twisted.resource import WebSocketResource
from autobahn.twisted.websocket import WebSocketServerProtocol, WebSocketServerFactory
from threading import Thread
from twisted.internet import reactor
from twisted.web.server import Site
from twisted.web.static import File

import settings
from ianaio.iana_io import IanaIO

commands = dict()
protocol = None
submitted_name = None

class WebSocketIO(IanaIO):

    class Protocol(WebSocketServerProtocol):

        def __init__(self):
            global protocol
            protocol = self

        def onConnect(self, request):
            print("Client connecting: {0}".format(request.peer))

        def onOpen(self):
            print("WebSocket connection open.")

        def onMessage(self, payload, isBinary):
            if isBinary:
                print("Binary message received: {0} bytes".format(len(payload)))
            else:
                temp = payload.decode('utf8').split()
                command_name = temp[0]
                params = temp[1:]
                global commands
                command = commands.get(command_name)
                if command is not None:
                    command(*params)
                else:
                    if command_name == "name":
                        self.set_name(*params)
                    else:
                        print("Command \"{0}\" not found".format(command_name))

            self.sendMessage(payload, isBinary)

        def onClose(self, wasClean, code, reason):
            print("WebSocket connection closed: {0}".format(reason))

        def set_name(self, name):
            global submitted_name
            print "Set name to ", name
            submitted_name = name

        def request_name(self):
            global submitted_name
            self.sendMessage("request_name")
            while submitted_name is None:
                pass
            result = submitted_name
            submitted_name = None
            return result

    def __init__(self, publisher):
        super(WebSocketIO, self).__init__(publisher)
        global commands
        commands = dict(
            explore=publisher.explore,
            goto=publisher.goto,
        )

    def broadcast(self, msg):
        for c in self.Protocol.clients:
            c.sendMessage(msg.encode('utf8'))

    def start(self):

        def start_up():
            root = File(".")

            factory = WebSocketServerFactory(u"ws://{0}:{1}".format(settings.INTERFACE, settings.PORT))
            factory.protocol = WebSocketIO.Protocol

            resource = WebSocketResource(factory)

            # websockets resource on "/ws" path
            root.putChild(u"ws", resource)

            site = Site(root)

            reactor.listenTCP(settings.PORT, site)
            reactor.run(installSignalHandlers=False)

        thread = Thread(target=start_up)
        thread.start()

    def request_name(self):
        global protocol
        return protocol.request_name()