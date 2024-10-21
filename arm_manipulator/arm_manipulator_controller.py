import rclpy
from geometry_msgs.msg import Twist
from tutorial_interfaces.msg import ManipulatorSensor

DISTANCE_LATERAL = (0.316)
DISTANCE_LONGITUDINAL = (0.456)
WHEEL_RADIUS = (0.05)

SPEED_INCREMENT = 0.05

class MyRobotDriver:
    def init(self, webots_node, properties):

        self.__robot = webots_node.robot

        self.__motor_speed = [0.0, 0.0, 0.0, 0.0]

        self.__front_right_motor = self.__robot.getDevice('wheel1')
        self.__front_left_motor = self.__robot.getDevice('wheel2')
        self.__back_right_motor = self.__robot.getDevice('wheel3')
        self.__back_left_motor = self.__robot.getDevice('wheel4')

        self.__ARM1 = self.__robot.getDevice('arm1')
        self.__ARM2 = self.__robot.getDevice('arm2')
        self.__ARM3 = self.__robot.getDevice('arm3')
        self.__ARM4 = self.__robot.getDevice('arm4')
        self.__ARM5 = self.__robot.getDevice('arm5')

        self.__finger_right = self.__robot.getDevice('finger::right')
        self.__finger_left = self.__robot.getDevice('finger::left')
        
       
        self.__front_left_motor.setPosition(float('inf'))
        self.__front_left_motor.setVelocity(0.0)

        self.__front_right_motor.setPosition(float('inf'))
        self.__front_right_motor.setVelocity(0.0)

        self.__back_left_motor.setPosition(float('inf'))
        self.__back_left_motor.setVelocity(0.0)

        self.__back_right_motor.setPosition(float('inf'))
        self.__back_right_motor.setVelocity(0.0)

        self.__target_twist = Twist()
        self.__target_manipulator_sensor = ManipulatorSensor()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, '/cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(ManipulatorSensor, '/manipulator_sensor', self.__manipulator_sensor_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def __manipulator_sensor_callback(self, msg):
        self.__target_manipulator_sensor = msg

    def base_move(self, forward_speed, side_speed, angular_speed):
        self.__motor_speed[0] = 1.0 / WHEEL_RADIUS * (forward_speed + side_speed + (DISTANCE_LATERAL + DISTANCE_LONGITUDINAL) * angular_speed)
        self.__motor_speed[1] = 1.0 / WHEEL_RADIUS * (forward_speed - side_speed - (DISTANCE_LATERAL + DISTANCE_LONGITUDINAL) * angular_speed)
        self.__motor_speed[2] = 1.0 / WHEEL_RADIUS * (forward_speed - side_speed + (DISTANCE_LATERAL + DISTANCE_LONGITUDINAL) * angular_speed)
        self.__motor_speed[3] = 1.0 / WHEEL_RADIUS * (forward_speed + side_speed - (DISTANCE_LATERAL + DISTANCE_LONGITUDINAL) * angular_speed)

        self.__front_left_motor.setVelocity(self.__motor_speed[0])
        self.__front_right_motor.setVelocity(self.__motor_speed[1])
        self.__back_left_motor.setVelocity(self.__motor_speed[2])
        self.__back_right_motor.setVelocity(self.__motor_speed[3])

    def arm_move(self):
        self.positions = self.__target_manipulator_sensor.position

        self.__ARM1.setPosition(self.positions[0])
        self.__ARM2.setPosition(self.positions[1])
        self.__ARM3.setPosition(self.positions[2])
        self.__ARM4.setPosition(self.positions[3])
        self.__ARM5.setPosition(self.positions[3])
        self.__finger_right.setPosition(0.01)
        self.__finger_left.setPosition(0.01)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = 0.0
        side_speed = 0.0
        angular_speed = 0.0

        #position_arm = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.__motor_speed = [0.0, 0.0, 0.0, 0.0]

        forward_speed = self.__target_twist.linear.x
        side_speed = self.__target_twist.linear.y
        angular_speed = self.__target_twist.angular.z

        # if len(self.__target_joint_state.position) >= 7:
        #     position_arm = self.__target_joint_state.position[:7]
        # position_arm[0] = self.__target_joint_state.position[self.__target_joint_state.name.index('arm1')]

        # self.__node.get_logger().info('forward_speed: "%s"' % forward_speed)
        # self.__node.get_logger().info('angular_speed: "%s"' % angular_speed)
        # self.__node.get_logger().info('side_speed: "%s"' % side_speed)

        #self.__node.get_logger().info('position_arm: "%s"' % position_arm)

        self.base_move(forward_speed, side_speed, angular_speed)

        self.arm_move()

        # self.__node.get_logger().info('front_left_motor: "%s"' % self.__front_left_motor.getVelocity())
        # self.__node.get_logger().info('front_right_motor: "%s"' % self.__front_right_motor.getVelocity())
        # self.__node.get_logger().info('back_left_motor: "%s"' % self.__back_left_motor.getVelocity())
        # self.__node.get_logger().info('back_right_motor: "%s"' % self.__back_right_motor.getVelocity())

        # self.__node.get_logger().info('finger_right: "%s"' % self.__target_joint_state.position[5])
        # self.__node.get_logger().info('finger_left: "%s"' % self.__target_joint_state.position[6])