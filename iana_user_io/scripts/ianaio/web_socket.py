#TODO This code is a mess! I (Beni) am so sorry and will clean it up later
import os
from autobahn.twisted.resource import WebSocketResource
from autobahn.twisted.websocket import WebSocketServerProtocol, WebSocketServerFactory
from os.path import abspath, dirname
from threading import Thread
from twisted.internet import reactor
from twisted.web.server import Site
from twisted.web.static import File

import cv2
import settings
from ianaio.iana_io import IanaIO

commands = dict()
protocol = None
submitted_name = None


class MapData(object):
    def __init__(self, resolution, origin, width, height, origin_x, origin_y, map):
        self.resolution = resolution
        self.origin = origin
        self.width = width
        self.height = height
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.map = map


class _BroadcastServerProtocol(WebSocketServerProtocol):
    def __init__(self):
        global protocol
        protocol = self

    def onOpen(self):
        self.factory.register(self)
        self.refresh_map(self.factory.current_map)

    def onClose(self, wasClean, code, reason):
        self.factory.unregister(self)

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

    def set_name(self, name):
        global submitted_name
        submitted_name = name

    def request_name(self):
        global submitted_name
        self.sendMessage("request_name")
        while submitted_name is None:
            pass
        result = submitted_name
        submitted_name = None
        return result

    def refresh_map(self, map_data):
        if map_data is None:
            return
        self.sendMessage(
            "refresh_map {0},{1},{2},{3},{4},".format(
                str(map_data.resolution),
                str(map_data.width),
                str(map_data.height),
                str(map_data.origin_x),
                str(map_data.origin_y)
            ) + ','.join((str(0) if i == -1 else str(i)) for i in map_data.map))


class _BroadcastServerFactory(WebSocketServerFactory):
    protocol = _BroadcastServerProtocol

    def __init__(self, url):
        WebSocketServerFactory.__init__(self, url)
        self.clients = []
        self.current_map = None

    def register(self, client):
        if client not in self.clients:
            self.clients.append(client)
            print("registered new client {}".format(client.peer))

    def unregister(self, client):
        if client in self.clients:
            self.clients.remove(client)
            print("unregistered client {}".format(client.peer))

    def request_name(self):
	for c in self.clients:
            c.request_name()

    def refresh_map(self, map_data):
        self.current_map = map_data
        for c in self.clients:
            c.refresh_map(self.current_map)


class WebSocketIO(IanaIO):

    def __init__(self, publisher):
        super(WebSocketIO, self).__init__(publisher)
        global commands
        commands = dict(
            explore=publisher.explore,
            goto=publisher.goto,
        )
        self.factory = None

    def start(self):

        def start_up():
            root = File(dirname(dirname(dirname(abspath(__file__)))))

            self.factory = _BroadcastServerFactory(u"ws://{0}:{1}".format(settings.INTERFACE, settings.PORT))

            resource = WebSocketResource(self.factory)

            # websockets resource on "/ws" path
            root.putChild(u"ws", resource)

            site = Site(root)

            reactor.listenTCP(settings.PORT, site)
            reactor.run(installSignalHandlers=False)

        thread = Thread(target=start_up)
        thread.daemon = True # we don't need the webserver anymore, if the ros node dies
        thread.start()

    def request_name(self, preview_image):
	root = File(dirname(dirname(dirname(abspath(__file__)))))
	cv2.imwrite(root.name+'/preview_image.png', preview_image) 
        return self.factory.request_name()

    def refresh_map(self, resolution, origin, width, height, origin_x, origin_y, map):
        self.factory.refresh_map(MapData(resolution, origin, width, height, origin_x, origin_y, map))
