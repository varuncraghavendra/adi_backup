import py_trees


class RetryLimitReached(py_trees.behaviour.Behaviour):
    def __init__(self, node, drone_ns: str = "drone_1", retry_limit: int = 5):
        super().__init__(name=f"RetryLimitReached_{drone_ns}")
        self.node = node
        self.drone_ns = drone_ns
        self.retry_limit = retry_limit
        self.bb = self.attach_blackboard_client(name=f"RetryLimit_{drone_ns}")
        self.key_retry = f"{self.drone_ns}/retry_counter"
        self.bb.register_key(key=self.key_retry, access=py_trees.common.Access.READ)

    def update(self):
        count = getattr(self.bb, self.key_retry, 0)
        if count >= self.retry_limit:
            # limit reached → mission fails → higher-level tree can trigger landing
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS
