import ros2_pyimclsts.pyimc_generated as pg
#import pyimc_generated as pg
import pyimclsts.network as n
import asyncio
import numpy as np
import math
import utm
from scipy.spatial.transform import Rotation as R 
import random

# ROS Specific libraries
import rclpy
from geometry_msgs.msg import Transform, Pose, PointStamped
from sensor_msgs.msg import NavSatFix

"""
    This scripts creates a ROS node and subscribes to a specific topic. 
    It also creates a UDP "connection" between python and DUNE.
"""

def calc_transform_2d(utm1 :Pose , utm2 : Pose):

    translation1 = np.array([utm1.position.x, utm1.position.y, utm1.position.z])
    rot1 = R.from_euler('xyz', [utm1.orientation.x, utm1.orientation.y, utm1.orientation.z]).as_matrix()

    translation2 = np.array([utm2.position.x, utm2.position.y, utm2.position.z])
    rot2 = R.from_euler('xyz', [utm2.orientation.x, utm2.orientation.y, utm2.orientation.z]).as_matrix()

    pose1 = np.eye(4)
    pose1[:3, :3] = rot1
    pose1[:3, 3] = translation1

    pose2 = np.eye(4)
    pose2[:3, :3] = rot2
    pose2[:3, 3] = translation2

    homogeneous_matrix = np.linalg.inv(pose1) @ pose2

    # You can then extract the relevant transform and rotation
    relative_translation = homogeneous_matrix[:3, 3]
    relative_rotation = homogeneous_matrix[:3, :3]

    """
    print("\nRelative Translation:")
    print(relative_translation)

    print("\nRelative Rotation Matrix:")
    print(relative_rotation)
    """
    return homogeneous_matrix

def homoToTf(homoT):

    tf = Transform()
    tf.translation.x = homoT[0,3]
    tf.translation.y = homoT[1,3]
    tf.translation.z = homoT[2,3]

    tf.rotation.x = math.atan2(homoT[2,1], homoT[2,2])
    tf.rotation.y = math.atan2(-homoT[2,0], math.sqrt(homoT[2,1]**2 + homoT[2,2]**2))
    tf.rotation.z = math.atan2(homoT[1,0], homoT[0,0])

    return tf 
    
class FollowRef_Vehicle():
    '''
    Minimal implementation to start a Follow Reference manuever
    '''
        
    def __init__(self, target : str, in_ip : str = "127.0.0.1", in_port : int = 8000):

        '''target is the name of the vehicle as in Announce messages'''
        self.EstimatedState = None
        self.FollowRefState = None
        self.in_ip = in_ip
        self.in_port = in_port
        self.target = target
        self.peers = dict()
        
        # Dune GOTO
        self.goto = pg.messages.Goto()
        self.plan_control = pg.messages.PlanControl()
        self.referenceToFollow = pg.messages.Reference()
        self.new_goal = False

        self.NavSat = NavSatFix()

    def request_followRef(self, send_callback):

        request = pg.messages.PlanControl()

        fr = pg.messages.FollowReference()
        fr.control_src = pg._base._default_src
        fr.control_ent = 0xFF
        fr.timeout = 10
        fr.loiter_radius = 0
        fr.altitude_interval = 0



        print("Requesting Follow Reference Manuever...")

    def send_announce(self, send_callback):
        
        announce = pg.messages.Announce()
        announce.sys_name = 'python-client'
        announce.sys_type = 0
        announce.owner = 65535
        announce.lat = 0.7186986607
        announce.lon = -0.150025012
        announce.height = 0
        announce.services = 'imc+udp://' + self.in_ip + ':' + str(self.in_port)

        send_callback(announce, dst = self.peers.get(self.target, 0xFFFF))

    def update_peers(self, msg : pg.messages.Announce, send_callback):
        self.peers[msg.sys_name] = msg._header.src

    def update_vehicle_state(self, msg : pg.messages.EstimatedState, send_callback):
        
        # Get the estimated state and turn it into a tf tree for publishing
        self.EstimatedState = msg

        # Turn the lat lon values into ECEF Coordinates
        self.currentLoc.fill_it(msg)     
        
        current_loc_utm = utm.from_latlon(math.degrees(self.currentLoc.lat), math.degrees(self.currentLoc.lon))

        # We want to publish the transform matrix between our reference (porto de leixoes) and the actual position
        # We can calculate this.

        self.pose_lauv.position.x = current_loc_utm[0] + msg.y
        self.pose_lauv.position.y = current_loc_utm[1] + msg.x
        self.pose_lauv.position.z = self.currentLoc.depth + msg.z
        self.pose_lauv.orientation.z = -self.currentLoc.yaw + math.pi/2
        h_matrix = calc_transform_2d(pose_leixoes, self.pose_lauv)

        # Fill up the transform my G
        self.tf.transform = homoToTf(h_matrix)

        # Having the transform calculated we now need to publish it to the ROS ecossystem
        if self.tf.header.frame_id != 'map':
            self.tf.header.frame_id = 'map'
            self.tf.child_frame_id = 'base_link'

    def ros_update_GoalPose(self, gps_goal : PointStamped):
        
        # When we actually receive the goal pose, we should turn it into a IMC Goto. 
        print("[ROS Connection] Received a Goal Pose: \n{}".format(gps_goal))


        # This has to be included in a PlanControl
        self.goto.timeout =  1000
        self.goto.lat = 0.718746061281
        self.goto.lon = -0.151947995163
        self.goto.z  = 0
        self.goto.z_units = 1
        self.goto.speed = 1000 
        self.goto.speed_units = 1
        self.goto.roll = 0.0
        self.goto.yaw = 0.0
        self.goto.pitch = 0.0
        self.goto.custom = ""

        self.new_goal = True
    
    def ros_update_NavSat(self, nav_sat : NavSatFix):

        # Get the location from simulator and save it internally     
        self.NavSat = nav_sat

    def send_goal(self, send_callback):

        if self.new_goal == True:


            # To send a goal it should be contained inside a maneuver, which should be contained inside a plan specification 
            # Which should also be inside plan control 

            plan_maneuver = pg.messages.PlanManeuver()
            plan_maneuver.maneuver_id = "1"
            plan_maneuver.data = self.goto
            plan_maneuver.start_actions = []
            plan_maneuver.end_actions = []

            # New Specification
            planSpecification = pg.messages.PlanSpecification()
            planSpecification.plan_id = "python-plan"
            planSpecification.description = ""
            planSpecification.vnamespace = ""
            planSpecification.variables = []
            planSpecification.start_man_id = "1"
            planSpecification.maneuvers = [plan_maneuver]
            planSpecification.transitions = []
            planSpecification.start_actions = []
            planSpecification.end_actions = []

            # New 
            self.plan_control.type = pg.messages.PlanControl.TYPE.REQUEST
            self.plan_control.op = pg.messages.PlanControl.OP.START
            self.plan_control.request_id = random.randint(0, pg._base._default_src)
            self.plan_control.plan_id = "python-plan"
            self.plan_control.flags = pg.messages.PlanControl.FLAGS.IGNORE_ERRORS
            self.plan_control.arg = planSpecification
            self.plan_control.info = "python-plan"

            send_callback(self.plan_control, dst = 30, dst_ent=255)

            print("[UDP Connection] New plan was sent to DUNE ")
            self.new_goal = False

    def send_navsat(self, send_callback):

        if self.NavSat.latitude != 0:

            # Build a Simulated State message to send to dune

            simulatedState = pg.messages.SimulatedState()
            simulatedState.lat = math.radians(self.NavSat.latitude)
            simulatedState.lon = math.radians(self.NavSat.longitude)
            # This is where the depth would be usefull but I fail to find it
            # Actually no, I can get this from the state of the actual vehicle
            simulatedState.height = 0
            simulatedState.x = 0 
            simulatedState.y = 0 
            simulatedState.z = 0 
            # Same thing in the orientation 
            simulatedState.phi = 0 
            simulatedState.theta = 0 
            simulatedState.psi = 0 
            # For the velocity and acceleration we can get it from the IMU 
            simulatedState.u = 0 
            simulatedState.v = 0 
            simulatedState.w = 0 
            # Save the angular velocity which could also come from the IMU 
            simulatedState.p = 0 
            simulatedState.q = 0 
            simulatedState.r = 0 
            simulatedState.svx = 0 
            simulatedState.svy = 0
            simulatedState.svz = 0

            send_callback(simulatedState, dst = 30, dst_ent=255)
            print("[UDP Connection] New Simulated State was sent to DUNE")
            

# THE ACTUAL MAIN TASKS
 
async def ros_sub_GoalPose():

    print("## ROS-Bridge-Dune Node Started ## \n")

    # Subscribe to the /tethys/navsat topic, create a goto and send by UDP to DUNE.
    node.create_subscription(NavSatFix, '/tethys/navsat', vehicle.ros_update_NavSat, 100)
    
    while rclpy.ok():
    
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(0.5)


async def udp_main():

    con = n.udp_interface('localhost', 8000, 'localhost', 6002)
    sub = n.subscriber(con)

    # This handles the communication with DUNE. 
    # Subscriber gets messages, publisher sends messages
    sub.subscribe_async(vehicle.update_peers, pg.messages.Announce)
    #sub.subscribe_async(vehicle.update_vehicle_state, pg.messages.EstimatedState)

    #sub.periodic_async(vehicle.send_goto, 3)
    sub.periodic_async(vehicle.send_announce, 10)
    sub.periodic_async(vehicle.send_navsat, 3)
    
    # Usually we would just call sub.run() Which calls asyncio.run(sub.event_loop)
    # However now we need a another task to deal with ROS environment
    sub._keep_running = True
    await sub._event_loop()


async def main2():
    
    await asyncio.gather(udp_main(), ros_sub_GoalPose())

LocationType = pg._base.locationType

# Save the location in Porto De Leixoes to use as a /map transform
leixoes_loc = [41.18490, -8.70452, 0]
leixoes_loc_utm = utm.from_latlon(leixoes_loc[0], leixoes_loc[1])
print("Leixoes \n LATLON: {} {} \n UTM: {}"
        .format(leixoes_loc[0], leixoes_loc[1], leixoes_loc_utm))

pose_leixoes = Pose()
pose_leixoes.position.x = leixoes_loc_utm[0]
pose_leixoes.position.y = leixoes_loc_utm[1]

# This is just an object to keep track of all the info related with the vehicle. 
vehicle = FollowRef_Vehicle('lauv-xplore-1', 'localhost',8000)

rclpy.init()
node = rclpy.create_node('ros2_dune_bridge')


def main():

    asyncio.run(main2())
    


    


