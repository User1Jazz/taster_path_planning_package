import math

"""

"""
class Point(object):
    x = 0
    y = 0
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def values(self):
        return (self.x, self.y)

"""
Function to calculate acceleration based on the applied steering angle    
"""
def get_acceleration(max_speed, wheels_angle, max_steering, steer_tolerance):
    if max_steering == 0:
        raise ValueError("max_steering must be non-zero.")
    
    speed = max_speed * (1 - abs(wheels_angle / max_steering)) # max speed * (1- (curr_angle / max_angle))
    
    if speed < steer_tolerance:
        speed = steer_tolerance
    elif speed > (max_speed - steer_tolerance):
        speed = max_speed - steer_tolerance
    
    return float(speed)

"""
Function to calculate steering based on the waypoint position
"""
def get_steering(midpoint, max_steering):
    midpoint_x = -midpoint[1]
    midpoint_y = midpoint[0]

    # distance between car and midpoint
    distance = math.sqrt(midpoint_x**2 + midpoint_y**2)
    angle = math.asin(abs(midpoint_x) / distance) #radians

    # negative steering angles turn right, positive turns left
    # if midpoint is on the right (y is negative) make angle negative
    if midpoint_x < 0:
        angle = -angle
        
    # seems to make turns smoother at faster speeds?
    L = 1.5 # Don't ask me what L is I have no idea, it is in Stephen's code as a constant and that's all I know. Ask Stephen for more info and good luck; Sincerely Kristijan Segulja, Chief of Simulations, FSTN
    steering_angle = math.atan((2*L*math.sin(angle)/distance))

    degrees = math.degrees(steering_angle)
    if(degrees > max_steering):
        degrees = max_steering
    elif(degrees < -max_steering):
        degrees = -max_steering

    return float(degrees)

"""
Function to limit steering based on the max_steering parameter
"""
def cap_steering(angle_rad, max_steering):
    degrees = math.degrees(angle_rad)
    if(degrees > max_steering):
        degrees = max_steering
    elif(degrees < -max_steering):
        degrees = -max_steering
    return float(degrees)

"""
Function to sort points, this sorts point based on their distance to the car, nearest first
"""
def sort_points(points_array):
    return sorted(points_array, key=lambda c: math.sqrt(c.x**2 + c.y**2))

"""
Function to calculate vector between two points
"""
def vector_between_points(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    return (dx, dy)

"""
Function to calculate the angle between two vectors
"""
def angle_between_vectors(a, b):
    # Calculate the dot product
    dot_product = a[0] * b[0] + a[1] * b[1]
    
    # Calculate the magnitudes of the vectors
    mag_a = math.sqrt(a[0]**2 + a[1]**2)
    mag_b = math.sqrt(b[0]**2 + b[1]**2)
    
    # Calculate the cosine of the angle
    cos_theta = dot_product / (mag_a * mag_b)
    
    # Calculate the angle in radians
    theta_radians = math.acos(cos_theta)
    
    # Convert the angle to degrees
    theta_degrees = math.degrees(theta_radians)
    
    return theta_radians, theta_degrees

"""
Function to calculate the forward vector, given the angle in radians
"""
def forward_vector_by_angle(angle_radians):
    fx = math.cos(angle_radians)
    fy = math.sin(angle_radians)
    return (fx, fy)

"""
Function to apply forward vector from origin point to aonther point
"""
def apply_forward_vector_to_point_from_origin(origin_angle_radians, point_x, point_y):
    # Calculate the forward vector of the origin
    forward_vector = forward_vector_by_angle(origin_angle_radians)
    
    # Apply the origin's forward vector to the second point
    new_x = point_x + forward_vector[0]
    new_y = point_y + forward_vector[1]
    
    return (new_x, new_y)

def get_distance(p1_x, p1_y, p2_x, p2_y):
    return math.sqrt((p2_x -p1_x)**2 + (p2_y - p1_y)**2)