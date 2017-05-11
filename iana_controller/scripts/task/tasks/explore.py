import rospy
import actionlib
import iana_navigation.msg

from task.task import Task


class ExploreTask(Task):

    def __init__(self, until):
        super(ExploreTask, self).__init__()
        self.explore_action = actionlib.SimpleActionClient('/iana/navigation/explore', iana_navigation.msg.ExploreAction)
        self.explore_action.wait_for_server()
        self.goal = iana_navigation.msg.ExploreGoal(until=until)
        rospy.loginfo('New ExploreTask created')

    def start(self):
        rospy.loginfo('goal: {}'.format(self.goal))
        self.explore_action.send_goal(self.goal, self._goal_reached_callback)
        rospy.loginfo('ExploreTask started')

    def resume(self):
        self.start()
        rospy.loginfo('ExploreTask resumed')

    def interrupt(self):
        self.explore_action.cancel_goal()
        rospy.loginfo('ExploreTask interrupted')

    def shutdown(self):
        self.explore_action.cancel_goal()
        rospy.loginfo('ExploreTask shutdown')

    def interruptable_by(self, task):
        return super(ExploreTask, self).interruptable_by(task)

    def _goal_reached_callback(self, state, result):
        self.terminated.set()
        rospy.loginfo('ExploreTask goal reached!')

