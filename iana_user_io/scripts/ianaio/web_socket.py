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
import rospy
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
        self.refresh_tasks(self.factory.current_tasks)
        self.refresh_robot_position(self.factory.current_robot_pose)

    def onClose(self, wasClean, code, reason):
        self.factory.unregister(self)

    def onMessage(self, payload, isBinary):
        if isBinary:
            rospy.logdebug("Binary message received: {0} bytes".format(len(payload)))
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
                    rospy.logdebug("Command \"{0}\" not found".format(command_name))

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
            ) + ','.join(map_data.map))

    def refresh_robot_position(self, pose):
        if pose is None:
            return
        self.sendMessage("refresh_robot_position {0},{1},{2}".format(
            str(int(pose.position.x)),
            str(int(pose.position.y)),
            str(int(pose.position.z))
        ))

    def refresh_tasks(self, tasks):
        if tasks is None:
            return
        self.sendMessage("refresh_tasks " + ','.join(tasks))


class _BroadcastServerFactory(WebSocketServerFactory):
    protocol = _BroadcastServerProtocol

    def __init__(self, url):
        WebSocketServerFactory.__init__(self, url)
        self.clients = []
        self.current_map = None
        self.current_robot_pose = None
        self.current_tasks = None

    def register(self, client):
        if client not in self.clients:
            self.clients.append(client)
            rospy.loginfo("registered new client {}".format(client.peer))

    def unregister(self, client):
        if client in self.clients:
            self.clients.remove(client)
            rospy.loginfo("unregistered client {}".format(client.peer))

    def request_name(self):
        pass

    def refresh_map(self, map_data):
        self.current_map = map_data
        for c in self.clients:
            c.refresh_map(self.current_map)

    def refresh_robot_position(self, pose):
        if self.current_robot_pose == pose:
            return
        self.current_robot_pose = pose
        for c in self.clients:
            c.refresh_robot_position(pose)

    def refresh_tasks(self, tasks):
        if self.current_tasks == tasks:
            return
        self.current_tasks = tasks
        for c in self.clients:
            c.refresh_tasks(tasks)


class WebSocketIO(IanaIO):

    def __init__(self, publisher):
        super(WebSocketIO, self).__init__(publisher)
        global commands
        commands = dict(
            explore=publisher.explore,
            explore_random=publisher.explore_random,
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
        global protocol
        if protocol is None:
            return None
        root = File(dirname(dirname(dirname(abspath(__file__)))))
        cv2.imwrite(root.path+'/preview_image.png', preview_image)
        return protocol.request_name()

    def refresh_robot_position(self, pose):
        self.factory.refresh_robot_position(pose)

    def refresh_map(self, resolution, origin, width, height, origin_x, origin_y, map):
        self.factory.refresh_map(MapData(resolution, origin, width, height, origin_x, origin_y, map))

    def refresh_tasks(self, tasks):
        self.factory.refresh_tasks(tasks)
