import logging


class StateMachine:
    def __init__(self, controller, initial_state):
        self.log = logging.getLogger(__name__)
        self.controller = controller
        self.state = initial_state
        self.state.enter(controller)
        self.log.debug(f"StateMachine initialized")

    def transition(self, new_state, info=None):
        if self.state.name == "error":
            if new_state.name not in ["boot", "idle", "stopping"]:
                self.log.error(f"Transition to {new_state.name} in Error State not allowed, try Reset or Stop")
                return
        self.log.info(f"exit {self.state.name}")
        self.state.exit(self.controller)
        self.state = new_state
        self.log.info(f"enter {self.state.name}")
        self.state.enter(self.controller, info=info)

    def update(self):
        self.state.run(self.controller)
