    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False        
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def timer_callback(self):
        #self.publish_offboard_control_heartbeat_signal()
        #self.get_logger().info('{}'.format(self.vehicle_status.nav_state))
        self.compose_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)     
        #self.get_logger().info('Land command sent')
