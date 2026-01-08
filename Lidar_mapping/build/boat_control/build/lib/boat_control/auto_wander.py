import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class AutoWander(Node):
    def __init__(self):
        super().__init__('auto_wander')
        self.left_pub = self.create_publisher(Float64, '/boat_1/left_thrust_cmd', 10)
        self.right_pub = self.create_publisher(Float64, '/boat_1/right_thrust_cmd', 10)

        # Parameters for thrust and durations
        self.forward_thrust = self.declare_parameter('forward_thrust', 60.0).value
        self.turn_thrust = self.declare_parameter('turn_thrust', 35.0).value
        self.forward_time = self.declare_parameter('forward_time', 3.0).value
        self.turn_time = self.declare_parameter('turn_time', 1.5).value
        self.idle_time = self.declare_parameter('idle_time', 1.0).value
        self.max_delta = self.declare_parameter('max_delta', 30.0).value  # max thrust step per tick

        self.state = 'idle'
        self.state_end = self.get_clock().now()
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self._tick)
        self.last_cmd = (0.0, 0.0)

    def _tick(self):
        now = self.get_clock().now()
        if now >= self.state_end:
            self._choose_next_state(now)
        self._publish_for_state()

    def _choose_next_state(self, now):
        # Bias toward forward motion
        next_states = ['forward', 'forward', 'turn_left', 'turn_right', 'idle']
        self.state = random.choice(next_states)
        if self.state == 'forward':
            duration = self.forward_time
        elif self.state == 'idle':
            duration = self.idle_time
        else:
            duration = self.turn_time
        self.state_end = now + rclpy.duration.Duration(seconds=duration)
        self.get_logger().info(f"State -> {self.state} for {duration:.1f}s")

    def _publish_for_state(self):
        if self.state == 'forward':
            l = self.forward_thrust
            r = self.forward_thrust
        elif self.state == 'turn_left':
            l = -self.turn_thrust
            r = self.turn_thrust
        elif self.state == 'turn_right':
            l = self.turn_thrust
            r = -self.turn_thrust
        else:  # idle
            l = 0.0
            r = 0.0
        self._send(*self._limit_change(l, r))

    def _limit_change(self, l_target, r_target):
        # Clamp thrust changes to avoid abrupt motion
        l_prev, r_prev = self.last_cmd
        l_cmd = self._step_limit(l_prev, l_target)
        r_cmd = self._step_limit(r_prev, r_target)
        self.last_cmd = (l_cmd, r_cmd)
        return l_cmd, r_cmd

    def _step_limit(self, prev, target):
        delta = target - prev
        if delta > self.max_delta:
            delta = self.max_delta
        elif delta < -self.max_delta:
            delta = -self.max_delta
        return prev + delta

    def _send(self, l_val, r_val):
        l_msg = Float64()
        r_msg = Float64()
        l_msg.data = float(l_val)
        r_msg.data = float(r_val)
        self.left_pub.publish(l_msg)
        self.right_pub.publish(r_msg)

    def destroy_node(self):
        # Stop the boat on shutdown
        self._send(0.0, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutoWander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
