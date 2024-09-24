import rclpy
from rclpy.node import Node

from fstn_messages_package.msg import Ai2vcu, Vcu2ai
from fstn_messages_package.msg import ConeArrayWithCovariance #, ConeArray

from .components import pathfinding_algorithms as pa
from .components import path_planning_basics as ppb
from .components.event_management import EventManager

AI_STATE = {'NOTHING': 0, 'AS_OFF': 1, 'AS_READY': 2, 'AS_DRIVING': 3, 'EMERGENCY_BRAKE': 4, 'AS_FINISHED': 5}
MISSION = {'NOT_SELECTED': 0, 'ACCELERATION': 1, 'SKIDPAD': 2, 'AUTOCROSS': 3, 'TRACKDRIVE': 4, 'INSPECTION_A': 5, 'INSPECTION_B': 6, 'AUTO_DEMO': 7}
MISSION_STATUS = {'NOT_SELECTED': 0, 'MISSION_SELECTED': 1, 'RUNNING': 2, 'FINISHED': 3}


class PathPlanning(Node):
    # PATH PLANNING variables (feel free to have fun around with these or try define your own) :)
    point = [1,0]                   # A list containing point coordinates [x,y]
    last_point = [1,0]              # A list containing previous point coordinates
    stop = False                    # A flag for stopping criteria
    orange_in_sight = False         # A flag for stating whether the orange is in sight
    max_steering = float(24)        # degrees
    max_speed = 4000                # rpm
    torque = 190.0                  # torque
    steer_tolerance = float(400)    # Limits speed reduction to the given value if reached
    angle = float(0)
    speed = float(0)
    
    def __init__(self):
        super().__init__('path_planning')

        self.i = 0

        self.as_state = AI_STATE['NOTHING']     # Set the AI status to default 'NOTHING'
        self.mission = MISSION['NOT_SELECTED']  # Set the mission status to default 'NOT SELECTED'
        self.wheel_rpm = float(0)               # Set the wheel rotation to default 0 rpm

        self.logger = self.get_logger()         # Define logger (this is used to print out debugging messages to the console)

        self.sub_pub()                          # Initialise publisher and subscriber nodes

    """
    Function to initialise publishers and subscribers
    """
    def sub_pub(self):

        # Vehicle Control Commands Publisher (we publish mission status and vehicle controls to this topic)
        self.publisher_ = self.create_publisher(Ai2vcu, 'vehicle_control_commands', 1)
        # Timer below is in seconds
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Vehicle Control Unit Subscriber (we receive mission status from this topic)
        self.ami_sub = self.create_subscription(Vcu2ai,
                                                '/vcuPublisher',
                                                self.can_callback,
                                                10)
        
        # Cones Subscriber (we receive cone positions from this topic)
        self.subscription = self.create_subscription(ConeArrayWithCovariance,
                                                     'cones_real',
                                                     self.listener_callback,
                                                     10)
    
    """
    Function to send vehicle control data to vehicle control node
    """
    def timer_callback(self):
        msg = Ai2vcu()
        msg.acle_speed_request_rpm = self.speed
        msg.steer_angle_request_deg = -self.angle
        msg.axle_torque_request_nm = self.torque
        msg.brake_press_request_pct = float(0)
        msg.mission_status = self.mission_status

        # Stop the car when the stop flag is true
        if self.stop:
            msg.axle_speed_request_rpm = float(0)
            msg.axle_torque_request_nm = self.torque
            msg.brake_press_request_pct = float(1000)
            msg.estop_request = self.trigger_ebs
            self.mission_status = MISSION_STATUS['FINISHED']
        return
    
    """
    Function to receive data from the vehicle control node
    """
    def can_callback(self, msg):
        # Set AI state
        if self.as_state != msg.as_state:
            self.as_state = msg.as_state
            self.logger.warn(f"AS State set to: {self.as_state}")
            self.mission_setup()

        # Set Mission Status
        if self.mission != msg.ami_state:
            self.mission = msg.ami_state
            self.logger.warn(f"Mission set to: {self.mission}")
            self.mission_setup()

        # Get current wheel RPM from the car
        if self.wheel_rpm != msg.fl_wheel_speed_rpm:
            self.wheel_rpm = msg.fl_wheel_speed_rpm
            self.logger.warn(f"Wheel RPM set to: {self.wheel_rpm}")
        
        return

    """
    Function to listen to cone data from perception node (and do some processing of said data)
    """
    def listener_callback(self, msg):
        if self.as_state == AI_STATE['AS_DRIVING']:
            
            # Store cones as Point objects into the array
            blue_cones = [ppb.Point(p.point.x, p.point.y) for p in msg.blue_cones]
            yellow_cones = [ppb.Point(p.point.x, p.point.y) for p in msg.yellow_cones]
            orange_cones = [ppb.Point(p.point.x, p.point.y) for p in msg.orange_cones]
            big_orange_cones = [ppb.Point(p.point.x, p.point.y) for p in msg.big_orange_cones]
            orange_cones = orange_cones + big_orange_cones

            # Sort cones by distance
            blue_cones = ppb.sort_points(blue_cones)
            yellow_cones = ppb.sort_points(yellow_cones)
            orange_cones = ppb.sort_points(orange_cones)
            
            # Algorithm for AUTOCROSS mission
            if self.mission == MISSION['AUTOCROSS']:
                """
                if there are cones on each side of the track, use midpoints algorithm (self.point = pa.calculatge_midpoint(blue_cone, yellow_cone))
                """
                """
                if there are no cones on the left side of the track, steer left it so we don't drive off the track (i.e. self.point = [1, 1])
                """
                """
                if there are no cones on the right side of the track, steer right it so we don't drive off the track (i.e. self.point = [1, -1])
                """
                pass
        return
            
    
    """
    To get torque (Nm) take acceleration (m/s^2) and times with the weight of the car (kg) and radius of the tyres (m), divided by the belt ratio (3.5:1)
    """
    def get_torque(self, acceleration):
        torque = (abs(acceleration) * 120 * 0.25) / 3.5  
        return torque
    
    """
    Function to check whether there are cones within the radius around the car;
    We use this function to filter out distant cones (since we don't want to calculate the path using the cones that are too far away)
    """
    def cones_nearby(self, cone_array, max_distance):
        for cone in cone_array:
            if ppb.get_distance(0, 0, cone.x, cone.y) > max_distance:
                return False
        return True
    


def main(args=None):
    rclpy.init(args=args)

    path_planning = PathPlanning()

    rclpy.spin(path_planning)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_planning.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
