import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import math

class MecanumDriveNode(Node):
    def __init__(self):
        super().__init__('mecanum_drive_node')

        # Initialize publishers and subscribers
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # self.clock_subscription = self.create_subscription(
        #     Clock,
        #     '/clock',
        #     self.clock_callback,
        #     10
        # )

        self.joint_states_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.odometry_publisher = self.create_publisher(
            Odometry,
            '/position',
            10
        )

        self.model_states_subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )
        
        #self.timer_pub = self.create_timer(0.10,self.joint_states_timer)

        self.wheel1_vel = 0
        self.wheel2_vel = 0
        self.wheel3_vel = 0
        self.wheel4_vel = 0
        # Initialize joint names and positions
        self.joint_names = ['wheel_FL_joint', 'wheel_BL_joint', 'wheel_BR_joint', 'wheel_FR_joint']
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.count = 0
        self.stop = False

    def cmd_vel_callback(self, msg):
        # Callback function to handle /cmd_vel messages and update joint_states
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # Calculate wheel velocities for mecanum drive
        self.wheel1_vel = 20.0*linear_x + 20.0*linear_y - 0.5*angular_z
        self.wheel2_vel = 20.0*linear_x - 20.0*linear_y - 0.5*angular_z
        self.wheel3_vel = 20.0*linear_x + 20.0*linear_y + 0.5*angular_z
        self.wheel4_vel = 20.0*linear_x - 20.0*linear_y + 0.5*angular_z

        if (linear_x == 0 and linear_y == 0):
            print(self.count)
            self.stop = False
        else:
            self.count = 0
            self.stop = True
        
    
    def joint_states_timer(self):
        # Update joint positions based on wheel velocities (for simplicity, using time increment as 1)
        dt = 0.10
        self.joint_positions[0] += self.wheel1_vel * dt
        self.joint_positions[1] += self.wheel2_vel * dt
        self.joint_positions[2] += self.wheel3_vel * dt
        self.joint_positions[3] += self.wheel4_vel * dt

        # Publish joint states
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.joint_positions
        self.joint_states_publisher.publish(joint_state_msg)
        self.odom_callback(joint_state_msg)

        if self.stop:
            self.count += 1
        
    
    def model_states_callback(self, msg):
        # Callback function to get simulation time
        idx = msg.name.index('my_bot')  # Adjust 'robot' to your robot's name
        self.sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        print(self.sim_time)

    
    def odom_callback(self, msg):
        
        self.x = 0.0125 * (msg.position[0]+msg.position[1]+msg.position[2]+msg.position[3])  
        self.y = 0.0125 * (msg.position[0]-msg.position[1]+msg.position[2]-msg.position[3])  
        self.theta = 0.5 * (-msg.position[0]-msg.position[1]+msg.position[2]+msg.position[3])  
        
        # self.x = math.cos(self.r)*self.u + math.sin(self.r)*self.v
        # self.y = -math.sin(self.r)*self.u + math.cos(self.r)*self.v
        # self.theta = self.r

        # Publish odometry message
        odometry_msg = Odometry()
        odometry_msg.header.stamp = self.get_clock().now().to_msg()
        odometry_msg.header.frame_id = 'odom'
        odometry_msg.child_frame_id = 'base_link'
        odometry_msg.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        odometry_msg.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(self.theta / 2),
            w=math.cos(self.theta / 2)
        )
        

        self.odometry_publisher.publish(odometry_msg)
        


def main():
    rclpy.init()

    mecanum_drive_node = MecanumDriveNode()

    try:
        rclpy.spin(mecanum_drive_node)
    except KeyboardInterrupt:
        pass
    finally:
        mecanum_drive_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
