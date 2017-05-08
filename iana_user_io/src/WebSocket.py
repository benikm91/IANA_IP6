#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

from autobahn.twisted.websocket import WebSocketServerProtocol, WebSocketServerFactory
from threading import Thread


def explore_factory(publisher):
    def explore():
        publisher.publish("EXPLORE!")
    return explore


def goto_factory(publisher):
    def goto(x, y):
        publisher.publish("Goto {0}/{1}".format(x, y))
    return goto

commands = dict()


class IanaUserIOWebSocket(WebSocketServerProtocol):

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


def start_server_factory(explore, goto):
    def start_server():
        global commands
        commands = dict(
            explore=explore,
            goto=goto,
        )
        factory = WebSocketServerFactory(u"ws://127.0.0.1:9000")
        factory.protocol = IanaUserIOWebSocket
        reactor.listenTCP(9000, factory)
        reactor.run(installSignalHandlers=False)
    return start_server


if __name__ == '__main__':

    import sys

    from twisted.python import log
    from twisted.internet import reactor

    log.startLogging(sys.stdout)

    try:
        thread = Thread(target=start_server_factory(
            explore_factory(rospy.Publisher('explorer', String, queue_size=10)),
            goto_factory(rospy.Publisher('goto', String, queue_size=10))
        ))
        thread.start()

        rospy.init_node('web_socket', anonymous=True)
        rate = rospy.Rate(10)  # 10hz
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    thread.join()
