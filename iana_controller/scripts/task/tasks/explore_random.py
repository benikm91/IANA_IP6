import threading
import rospy
import time

from task.task import Task

import std_msgs.msg


class ExploreRandomTask(Task):

    def __init__(self, msg):
        super(ExploreRandomTask, self).__init__()
        self.driver_explore_enable_publisher = rospy.Publisher('/iana/driver_explore/enable', std_msgs.msg.Empty, queue_size=10)
        self.driver_explore_disable_publisher = rospy.Publisher('/iana/driver_explore/disable', std_msgs.msg.Empty, queue_size=10)# explore
        self.running = threading.Event()
        self.until = msg.until

    @property
    def name(self):
        return "Explore Task (Only Random)"

    def update(self, elapsed):
        if time.time() > self.until:
            self.driver_explore_enable_publisher.publish()

    def on_start(self):
        rospy.logerr('start random exploring')
        self.running.set()
        self.driver_explore_enable_publisher.publish()

    def on_resume(self):
        rospy.logerr('resume random exploring')
        self.on_start()

    def on_interrupt(self):
        rospy.logerr('interrupt random exploring')
        self.running.clear()
        self.driver_explore_disable_publisher.publish()

    def on_shutdown(self):
        rospy.logerr('shutdown random exploring')
        self.running.clear()
        self.driver_explore_disable_publisher.publish()
        self.terminated.set()

    def interruptable_by(self, task):
        return True

    def _goal_reached_callback(self, state, result):
        if self.running.is_set():
            rospy.logerr('exploring goal reached: terminated set!')
            self.terminated.set()

