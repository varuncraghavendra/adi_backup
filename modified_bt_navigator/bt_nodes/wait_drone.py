import py_trees
import time


class WaitDrone(py_trees.behaviour.Behaviour):
    def __init__(self, node, drone_ns: str = "drone_1", name="Wait", wait_time=5.0):
        super().__init__(f"{name}_{drone_ns}")
        self.node = node
        self.drone_ns = drone_ns
        self.wait_time = wait_time
        self.start_time = None
        self.bb = self.attach_blackboard_client(name=f"Wait_{drone_ns}")
        self.key_retry = f"{self.drone_ns}/retry_counter"
        self.bb.register_key(key=self.key_retry, access=py_trees.common.Access.WRITE)

    def initialise(self):
        self.start_time = time.time()

    def update(self):
        elapsed = time.time() - self.start_time
        if elapsed >= self.wait_time:
            current = getattr(self.bb, self.key_retry, 0)
            setattr(self.bb, self.key_retry, current + 1)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
