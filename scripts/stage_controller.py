#! /usr/bin/env python
import rospy 
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import math
from tf.transformations import euler_from_quaternion

laser = LaserScan()
LASER_LEFT = (740, 851)
LASER_FRONT = (500, 581)

x = 0.0
y = 0.0
theta = 0.0

# point_of_encounter_with_wall(x, y, angle_to_target)
point_of_encounter_with_wall = (None, None, None)

# POINTS
# (-1, 6) v
# (-4, 3) v
# (4, -1) v .
# (4.5, 5) v 

target_x = 4.5
target_y = 5
min_distance = 0.1

MAX_DISTANCE_TO_WALL = .5

# DIRECTIONS
STRAIGHT = 0
LEFT = 1
RIGHT = 2

def get_direction_str(move):
  if move == STRAIGHT:
    return "straight"
  elif move == LEFT:
    return "left"
  else:
    return "right"

def odometry_callback(data):
	global x
	global y
	global theta
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y

	rot_q = data.pose.pose.orientation
	(_, _, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	
def laser_callback(data):
	global laser
	laser = data
 
def init_listener():
  rospy.init_node("stage_controller_node", anonymous=False)
  rospy.Subscriber("/base_pose_ground_truth", Odometry, odometry_callback)
  rospy.Subscriber("/base_scan", LaserScan, laser_callback)

def go(speed, direction):
	global pub
 
	if direction == STRAIGHT:
		speed.linear.x = 1.2
		speed.angular.z = 0
	elif direction == LEFT:
		speed.angular.z = 0.25
		speed.linear.x = 0
	elif direction == RIGHT:
		speed.angular.z = -0.25
		speed.linear.x = 0
	pub.publish(speed)
 
def facing_point():
	if None in (x, y, theta):
			return False
	n = get_angle_to_target()
	return n - min_distance <= theta <= n + min_distance

def is_target_angle_closer_through_left():
	if None in (x, y, theta):
			return False
	return theta - get_angle_to_target() < 0

def face_goal(speed):
	while not facing_point():
		rospy.loginfo("Location: X: %s | Y: %s | THETA: %s", x, y, theta)
		go(speed, RIGHT)

def follow_wall(speed):
  global point_of_encounter_with_wall
  global laser
  
  while get_laser_min_distance_within_range(LASER_FRONT) <= MAX_DISTANCE_TO_WALL:
    go(speed, RIGHT)
  
  last_move = None
  
  while not should_leave_wall():  
    rospy.loginfo("Location: X: %s | Y: %s | THETA: %s", x, y, theta)

    left = get_laser_min_distance_within_range(LASER_LEFT)
    front = get_laser_min_distance_within_range(LASER_FRONT)
    move = RIGHT
    
    if front <= MAX_DISTANCE_TO_WALL and last_move != LEFT:
      move = RIGHT
    elif MAX_DISTANCE_TO_WALL - .1 <= left <= MAX_DISTANCE_TO_WALL + .1:
      move = STRAIGHT
    elif left > MAX_DISTANCE_TO_WALL + .1:
      move = LEFT
    elif last_move == LEFT:
      move = STRAIGHT
    else:
      move = RIGHT
    
    rospy.loginfo("Going %s...", get_direction_str(move))
    go(speed, move)
    last_move = move
  
  face_goal(speed)
  point_of_encounter_with_wall = (None, None, None)  
    
def should_leave_wall():
	global point_of_encounter_with_wall
  
	if None in point_of_encounter_with_wall:
		point_of_encounter_with_wall = (x, y, get_angle_to_target())
		return False

	curr_angle_to_target = get_angle_to_target()
 
	# poe = point of encounter
	(poe_x, poe_y, poe_angle_to_target) = point_of_encounter_with_wall
	poe_distance_to_target = math.sqrt( (poe_x-target_x)**2 + (poe_y-target_y)**2 )
	distance_to_target = math.sqrt( (x-target_x)**2 +  (y-target_y)**2 )
	angle_adjustment = 0.01

	are_angles_close = poe_angle_to_target - angle_adjustment <= curr_angle_to_target <= poe_angle_to_target + angle_adjustment
	if are_angles_close and not is_near_to_point_of_encounter(): 
		if distance_to_target < poe_distance_to_target:
			point_of_encounter_with_wall = (None, None, None)
			rospy.loginfo("Finished going around obstacle!")
			return True
	return False

def is_near_to_point_of_encounter():
	global point_of_encounter_with_wall
	(poe_x, poe_y, _) = point_of_encounter_with_wall

	nearx = poe_x - .3 <= x <= poe_x + .3
	neary = poe_y - .3 <= y <= poe_y + .3
	return nearx and neary

def get_angle_to_target():
	delta_x = target_x - x
	delta_y = target_y - y
	return math.atan2(delta_y, delta_x)

def get_laser_min_distance_within_range(range):
  (a, b) = range
  global laser
  
  is_in_range = lambda x: laser.range_min <= x <= laser.range_max
  filtered_values = filter(is_in_range, laser.ranges[a:b])
  if len(filtered_values) > 0:
    return min(filtered_values)
  else:
   return sys.maxint

if __name__ == "__main__": 
	init_listener()

	speed = Twist()
	rospy.Rate(5) # 10hz
 
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)   
 
	while not rospy.is_shutdown():		
   
		distance = math.sqrt((x-target_x)**2 + (y-target_y)**2)
  
		if (distance > min_distance):
			rospy.loginfo("Location: X: %s | Y: %s | THETA: %s", x, y, theta)

			laser_left = get_laser_min_distance_within_range(LASER_LEFT)
			laser_front = get_laser_min_distance_within_range(LASER_FRONT)
   
			if (laser_front <= MAX_DISTANCE_TO_WALL):
				rospy.loginfo("Following wall...")
				follow_wall(speed)
			else:
				if facing_point():
					go(speed, STRAIGHT)
				elif is_target_angle_closer_through_left():
					go(speed, LEFT)
				else:
					go(speed, RIGHT)     		

		else:
			speed.linear.x = 0.0
			speed.angular.z = 0.0
			pub.publish(speed)
			rospy.loginfo("Reached target!")
			rospy.sleep(10)
			break
