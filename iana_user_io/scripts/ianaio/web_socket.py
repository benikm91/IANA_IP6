from autobahn.twisted.resource import WebSocketResource
from autobahn.twisted.websocket import WebSocketServerProtocol, WebSocketServerFactory
from threading import Thread
from twisted.internet import reactor
from twisted.web.server import Site
from twisted.web.static import File

import settings
from ianaio.iana_io import IanaIO

commands = dict()


class WebSocketIO(IanaIO):

    class Protocol(WebSocketServerProtocol):

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
                    print("Command \"{0}\" not found".format(command_name))

            # echo back message verbatim
            self.sendMessage(payload, isBinary)

        def onClose(self, wasClean, code, reason):
            print("WebSocket connection closed: {0}".format(reason))

    def __init__(self, publisher):
        super(WebSocketIO, self).__init__(publisher)
        global commands
        commands = dict(
            explore=publisher.explore,
            goto=publisher.goto
        )

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
        return "MUSTAFA"