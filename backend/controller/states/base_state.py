import logging


class BaseState:
    def __init__(self):
        self.log = logging.getLogger(__name__)
    name = "base"
    info = None

    def enter(self, controller, info=None):
        self.info = info

    def run(self, controller):
        pass

    def exit(self, controller):
        self.info = None
    