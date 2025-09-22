import py_trees


class CheckForNewGoal(py_trees.behaviour.Behaviour):
    def __init__(self, drone_ns: str = "drone_1"):
        super().__init__(name=f"CheckForNewGoal_{drone_ns}")
        self.drone_ns = drone_ns
        self.bb = self.attach_blackboard_client(name=f"Check_{drone_ns}")
        self.key_new_goal = f"{self.drone_ns}/new_goal_received"
        self.key_retry = f"{self.drone_ns}/retry_counter"
        self.bb.register_key(key=self.key_new_goal, access=py_trees.common.Access.READ)
        self.bb.register_key(key=self.key_retry, access=py_trees.common.Access.WRITE)

    def update(self):
        if getattr(self.bb, self.key_new_goal, False):
            # reset retry counter when a new goal arrives
            setattr(self.bb, self.key_retry, 0)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
