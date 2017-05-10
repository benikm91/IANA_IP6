class Command(object):
    def __init__(self, publisher):
        self.publisher = publisher


class ExploreCommand(Command):

    def __init__(self, publisher):
        super(ExploreCommand, self).__init__(publisher)

    def __call__(self):
        self.publisher.publish("EXPLORE!")


class GoToCommand(Command):

    def __init__(self, publisher):
        super(GoToCommand, self).__init__(publisher)

    def __call__(self, x, y):
        self.publisher.publish("Goto {0}/{1}".format(x, y))
