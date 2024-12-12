import numpy as np
import math
import pandas as pd

# ROS Specific libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock

"""

  This script takes a csv file and replays thruster and fins info into
  gazebo topics. 

"""

MAX_RPM = 2500

# The thruster in dune is controlled with magnitude values of a max RPM
# This function turns into rad per sec which is one of the ways joints are controlled
def max_rpm_to_rads(mag):
  return MAX_RPM*mag * (2*math.pi / 60)

class LogReplayer(Node):

  def __init__(self):
    
    super().__init__('LogReplayer')
    self.sim_last_timestamp = None
    self.sim_current_timestamp = None
    self.in_last_timestamp = None
    self.in_current_timestamp = None

    self.accumulated_sim_time = 0
    self.i_act = 0
    self.df_actuators = pd.read_csv('/home/developer/lrauv_ws/src/G_LAUV/logs/actuactor.csv')
    print(self.df_actuators)

    # Create subscribers and publishers
    sub = self.create_subscription(Clock,
                            '/clock',
                            self.check_sim_time, 
                            100,
                            )
    self.create_publishers()

    
  def check_sim_time(self, msg):
    
    self.sim_last_timestamp = self.sim_current_timestamp
    self.sim_current_timestamp = msg.clock


    # Check if its the first timestamp
    if self.sim_last_timestamp is not None:

      # Calculate time difference
      self.accumulated_sim_time += (self.sim_current_timestamp.sec - self.sim_last_timestamp.sec) + \
                  (self.sim_current_timestamp.nanosec - self.sim_last_timestamp.nanosec) * 1e-9
      
      # Check if it is time to activate a new input 
      in_time_diff = self.in_current_timestamp - self.in_last_timestamp


      if self.accumulated_sim_time >= in_time_diff:
        
        # Reset accumulated time difference
        self.accumulated_sim_time = 0
        self.publish_data()

    # First iteration
    else: 
      
      self.publish_data()
      self.get_logger().info("## Published Initial values")

  
  def create_publishers(self):
    
      self.pub_propel = self.create_publisher(Float64, '/model/tethys/joint/propeller_joint/cmd_vel', 10)
      self.pub_horizontal_fins = self.create_publisher(Float64, '/model/tethys/joint/horizontal_fins_joint/cmd_pos', 10)
      self.pub_vertical_fins = self.create_publisher(Float64, '/model/tethys/joint/vertical_fins_joint/cmd_pos', 10)

      self.get_logger().info("## Created Publishers ##")

  def publish_data(self):
    
    # Propeller input
    propel_msg = Float64()
    propel_msg.data = max_rpm_to_rads(self.df_actuators.loc[self.i_act, 'VEL'])

    # Vertical fins input
    vertical_fins_msg = Float64()
    vertical_fins_msg.data = -self.df_actuators.loc[self.i_act, 'U_SERVO']

    # horizontal fins input
    horizontal_fins_msg = Float64()
    fins_average = (float(self.df_actuators.loc[self.i_act, 'R_SERVO']) + float(self.df_actuators.loc[self.i_act, 'L_SERVO']))/2
    horizontal_fins_msg.data = -fins_average

    self.pub_propel.publish(propel_msg)
    self.pub_vertical_fins.publish(vertical_fins_msg)
    self.pub_horizontal_fins.publish(horizontal_fins_msg)
    
    self.i_act += 1
    self.in_current_timestamp = self.df_actuators.loc[self.i_act, 'TIME']
    self.in_last_timestamp = self.df_actuators.loc[self.i_act - 1, 'TIME']

    self.get_logger().info("Published: \n Thruster: {} Vertical_Fins: {} Horizonal_Fins: {}"
                          .format(propel_msg.data, vertical_fins_msg.data, horizontal_fins_msg.data))

          
def main(args=None):

    rclpy.init(args=args)
    node = LogReplayer()

    try: 
        
      rclpy.spin(node)

    except:
        
      node.get_logger().info('Publisher node shutting down.')

    finally:
        
      node.destroy_node()
      rclpy.shutdown()

if __name__ == '__main__':
   
   main()


    


