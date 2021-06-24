"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

#from controller import LightSensor


# get the time step of the current world.
# timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
def turn_left_on_place(motors, smoothness_factor = 10):
    #right motors
    motors[1].setVelocity(MAX_VELOCITY/smoothness_factor)
    motors[3].setVelocity(MAX_VELOCITY/smoothness_factor)
    # left motors
    motors[0].setVelocity(-MAX_VELOCITY/smoothness_factor)
    motors[2].setVelocity(-MAX_VELOCITY/smoothness_factor)
    
def turn_right_on_place(motors, smoothness_factor = 10):
    #right motors
    motors[1].setVelocity(-MAX_VELOCITY/smoothness_factor)
    motors[3].setVelocity(-MAX_VELOCITY/smoothness_factor)
    # left motors
    motors[0].setVelocity(+MAX_VELOCITY/smoothness_factor)
    motors[2].setVelocity(+MAX_VELOCITY/smoothness_factor)
    


def turn_left(motors, smoothness_factor = 2):
    #right motors
    motors[1].setVelocity(MAX_VELOCITY/smoothness_factor)
    motors[3].setVelocity(MAX_VELOCITY/smoothness_factor)
    # left motors
    motors[0].setVelocity(0.25*MAX_VELOCITY/smoothness_factor)
    motors[2].setVelocity(0.25*MAX_VELOCITY/smoothness_factor)


def turn_right(motors, smoothness_factor = 2):
    # right motors
    motors[1].setVelocity(0.25*MAX_VELOCITY/smoothness_factor)
    motors[3].setVelocity(0.25*MAX_VELOCITY/smoothness_factor)
    
    # left motors
    motors[0].setVelocity(MAX_VELOCITY/smoothness_factor)
    motors[2].setVelocity(MAX_VELOCITY/smoothness_factor)
    

def move_forward(motors, SPEED):
    motors[0].setVelocity(SPEED)
    motors[1].setVelocity(SPEED)
    motors[2].setVelocity(SPEED)
    motors[3].setVelocity(SPEED)
    
def ghost_detected(ls_l, ls_r):

    LS_THRESHOLD = 800
    if ls_l > LS_THRESHOLD or ls_r > LS_THRESHOLD:
        STATE = 'GHOST_DETECTED'
        return True
    else:
        return False    

def avoid_ghost(motors, ls_l, ls_r, smoothness_factor=10):
    
    if ls_l > ls_r:
        turn_left(motors, smoothness_factor)

        #print('avoid_ghost.should_turn_left_1')
        # turn_right is the right call here
        
    elif ls_l <= ls_r:
        turn_left(motors, smoothness_factor)
        #print('avoid_ghost.should_turn_left_2')

def check_model_is_treasure(model):
    if str(model) == 'treasure':
        return True
    else:
        return False

def check_color_is_gold(rbg_list):
    if rbg_list[0]==0.9999 and rbg_list[1]==0.8398 and rbg_list[2]==0:
        return True
    else:
        return False 

# records the LAST goal position as 3D coordinates list. This is done to iteratively improve 
# goal's position while moving towards it. The drawback is that if the robot moves away from it
# goal_position is a bit inaccurate as it contains the last position viewed by the camera. 
def detect_goal(obj_list, GOAL_DETECTED, GOAL_POSITION, robot, gps_values_robot):
    #print(type(obj_list))
    #print(obj_list)
    if len(obj_list)>0:
        print(len(obj_list))
        for obj in obj_list:
            # i find the treasure by checking its color and its model
            # DELETE
            print('detect_goal debug - obg.get_model(): {}'.format(str(obj.get_model())))
            print('detect_goal debug - obg.get_colors(): {}'.format(str(obj.get_colors())))
            # insert ad hoc functions to make the code more readable
            if check_model_is_treasure(obj.get_model()) and check_color_is_gold(obj.get_colors()):    
            #if (str(obj.get_model())=='treasure') and (obj.get_colors()==[0.9999, 0.8398, 0.0]):
            
                print('Treasure Found')
                GOAL_DETECTED = True
                
                GOAL_POSITION = calculate_goal_position(obj.get_position(),gps_values_robot )# current_position + relative position taken by the camera
                return GOAL_DETECTED, GOAL_POSITION
            else:
                return GOAL_DETECTED, GOAL_POSITION
    else:
        return GOAL_DETECTED, GOAL_POSITION 
#assumes that position_goal and position_robot are lists
# in the order [x_coord, y_coord, z_coord, ... other things ... ]
# returns a list like [x_cooord, y_coord, z_coord]
def calculate_goal_position(position_goal_rel, position_robot):
    return [position_goal_rel[0] + position_robot[0], position_goal_rel[1] + position_robot[1], position_goal_rel[2] + position_robot[2]]



#####
##### rifare
def align_with_goal(robot, motors, camera, GOAL_POSITION):
    rel_orientation_goal = camera.get_orientation()
    angle_x = rel_orientation_goal[0]*rel_orientation_goal[3]
    angle_z = rel_orientation_goal[2]*rel_orientation_goal[3]
    if not angle_x <0.02 and angle_z < 0.02:
        turn_left_on_place(motors, smoothness_factor=10)

def align_w_goal(gps_f, gps_b, goal_pos, motors, ALIGNMENT_THRESHOLD=0.001):
    # calculate m and q in the plane XZ
    print('Align w goal called.')
    m = (goal_pos[2] - gps_b[2])/(goal_pos[0] - gps_b[0] + 1E-6)
    q = ( goal_pos[0]*gps_b[2] - goal_pos[2]*gps_b[0])/(goal_pos[0] - gps_b[0] + 1E-6)
    # if gps_f not aligned and not d(goal, gps_f)<d(goal, gps_b) -> turn left on the spot
    # if not (z_gps_f - m*x_gps_f - q < THRESHOLD) and not (d_xz(goal, gps_f)<d_xz(goal, gps_b))
    if not (gps_f[2] - m*gps_f[0] - q < ALIGNMENT_THRESHOLD) and not (d_xz(goal_pos, gps_f)<d_xz(goal_pos, gps_b)):
        turn_left_on_place(motors)
    
    else:
        pass
    
        
# euclidean distance in the XZ plane        
def d_xz(coord1, coord2):
    return ((coord1[0] - coord2[0])**2 + (coord1[2] - coord2[2])**2 )**0.5
#######
######


### CONDITIONS TO DETECT OBJECTS
### Right Side
# checking whether there is an object at mild distance in front or laterally ON THE RIGHT SIDE 
def object_near_front_or_lateral_right(ds_r_val, ds_r_side_f_val):
    if (ds_r_val < 1000 and ds_r_val > 300) or ( ds_r_side_f_val<400 and ds_r_side_f_val>300):
        return True
    else:
        return False
        
# check whether an object is very near frontally on the right side 
def object_very_near_front_right(ds_r_val):
        if ds_r_val < 300:
            return True
        else:
            return False
            
# check whether an object is very near laterally on the right side
def object_very_near_lateral_right(ds_r_side_f_val):
    if ds_r_side_f_val <= 300:
        return True
    else:
        return False

# check whether the object is paraller to the robot in the right side.
# particularly useful for walls
def object_parallel_right_side(ds_r_side_f_val, ds_r_side_b_val):
    if ds_r_side_f_val - ds_r_side_b_val > 0 and ds_r_side_f_val - ds_r_side_b_val < 10:
        return True
    else:
        return False

### Left Side
def object_near_front_or_lateral_left(ds_l_val, ds_l_side_f_val):
    if (ds_l_val < 1000 and ds_l_val > 300) or ( ds_l_side_f_val<400 and ds_l_side_f_val>300):
        return True
    else:
        return False
        
# check whether an object is very near frontally on the right side 
def object_very_near_front_left(ds_l_val):
        if ds_l_val < 300:
            return True
        else:
            return False
            
# check whether an object is very near laterally on the right side
def object_very_near_lateral_left(ds_l_side_f_val):
    if ds_l_side_f_val <= 300:
        return True
    else:
        return False

# check whether the object is paraller to the robot in the right side.
# particularly useful for walls
def object_parallel_left_side(ds_l_side_f_val, ds_l_side_b_val):
    if ds_l_side_f_val - ds_l_side_b_val > 0 and ds_l_side_f_val - ds_l_side_b_val < 10:
        return True
    else:
        return False

       
# check whether an object is detected in any way
def object_detected(ds_r_val, ds_l_val,
                    ds_r_side_f_val, ds_l_side_f_val, 
                    ds_r_side_b_val, ds_l_side_b_val, obj_ls):
    
    if len(obj_ls)>0:
        obj = obj_ls[0]
        if not (str(obj.get_model())=='treasure' and obj.get_colors()==[0.9999, 0.8398, 0.0]):
            # check object on the right side
            if object_near_front_or_lateral_right(ds_r_val, ds_r_side_f_val):
                return True
            elif object_very_near_front_right(ds_r_val):
                return True
            elif object_very_near_lateral_right(ds_r_side_f_val):
                return True
            #elif object_parallel_right_side(ds_r_side_f_val, ds_r_side_b_val):
            #    return True
                
            # check object on the left side
            if object_near_front_or_lateral_left(ds_l_val, ds_l_side_f_val):
                return True
            elif object_very_near_front_left(ds_l_val):
                return True
            elif object_very_near_lateral_left(ds_l_side_f_val):
                return True
            #elif object_parallel_left_side(ds_l_side_f_val, ds_l_side_b_val):
            #    return True
            
            # if no objects are detected on both sides, return False
            else:
                return False
         # if  found the goal, return False, as it is not an object
        else:
            return False
    elif len(obj_ls)==0:
        # check object on the right side
            if object_near_front_or_lateral_right(ds_r_val, ds_r_side_f_val):
                return True
            elif object_very_near_front_right(ds_r_val):
                return True
            elif object_very_near_lateral_right(ds_r_side_f_val):
                return True
            #elif object_parallel_right_side(ds_r_side_f_val, ds_r_side_b_val):
            #    return True
                
            # check object on the left side
            if object_near_front_or_lateral_left(ds_l_val, ds_l_side_f_val):
                return True
            elif object_very_near_front_left(ds_l_val):
                return True
            elif object_very_near_lateral_left(ds_l_side_f_val):
                return True
        

def move_towards_goal(ls_l_val, ls_r_val, ds_r_val, ds_l_val, ds_r_side_f_val, ds_l_side_f_val, ds_r_side_b_val, ds_l_side_b_val, gps_val, gps_b_val, obj_ls, motors, GOAL_POSITION, GHOST_DETECTED, OBSTACLE_DETECTED):
    # if the robots does not detect objstacles or ghosts, it aligns with the goal and moves forwards with a braitenberg love behaviour.
    
    STEERING_OPERATION = 'Nothing'
    STATE_DIAGNOSTICS = 'Nothing'
    
    SMOOTHING_FACTOR = 1
    
    if not ghost_detected(ls_l_val, ls_r_val) and not object_detected(ds_r_val, ds_l_val, ds_r_side_f_val, ds_l_side_f_val, ds_r_side_b_val, ds_l_side_b_val, obj_ls):
        # calculate m and q
        gps_f, gps_b = gps_val, gps_b_val
        goal_pos = GOAL_POSITION
        
        ALIGNMENT_THRESHOLD = 0.001 # standard is 0.001
        #ORIENTATION_THRESHOLD = 0.005 # STANDARD IS ORIENTATION_THRESHOLD = 0.0025
        
        
        GHOST_DETECTED = False
        OBSTACLE_DETECTED = False
        
        m = (goal_pos[2] - gps_b[2])/(goal_pos[0] - gps_b[0] + 0.0001)
        q = (goal_pos[0]*gps_b[2] - goal_pos[2]*gps_b[0])/(goal_pos[0] - gps_b[0] + 0.0001)
        displacement = gps_f[2] - m*gps_f[0] - q
        
        print('m e q: {}, {}'.format(m, q))
        print('Disallineamento: {}'.format(gps_f[2] - m*gps_f[0] - q))
        
        # if gps_f not aligned and not d(goal, gps_f)<d(goal, gps_b) -> turn left on the spot
        # if not (z_gps_f - m*x_gps_f - q < THRESHOLD) and not (d_xz(goal, gps_f)<d_xz(goal, gps_b))
        if len(obj_ls)>0:
            #print('One object detected!')
            obj = obj_ls[0]
            orientation = obj.get_orientation()
            position = obj.get_position()
            #print('gps_f: {}'.format(gps_f))
            #print('GOAL_POSITION: {}'.format(GOAL_POSITION))
            
            distance = d_xz(GOAL_POSITION, gps_f)
            #print('DISTANCE: {}'.format(distance))
            #print('Orientation based on distance: {}'.format(orientation(distance)))
            calculated_orientation = orientation_with_distance(distance)
            
            if str(obj.get_model())=='treasure' and obj.get_colors() == [0.9999, 0.8398, 0.0]:
                print('Object detected in the alignment process')
                #if (obj.get_position()[0]**2 + obj.get_position()[2]**2)**0.5 > 300:   
                # orientation(distance)
                #if orientation[0] > ORIENTATION_THRESHOLD:
                STATE_DIAGNOSTICS = 'Goal detected on Camera.'
                
                
                #if orientation[0]*orientation[3] > orientation[2]*orientation[3] and orientation[0]*orientation[3]<-calculated_orientation:
                if position[0]<0 and position[2]<0:
                    if position[2] <= position[0] and d_xz(gps_val, goal_pos)>0.5:
                                 #orientation(d_xz(goal_pos, gps_f)): #ORIENTATION_THRESHOLD: and orientation[2] > 0.0025:
                                #if displacement >= ALIGNMENT_THRESHOLD and d_xz(goal_pos, gps_f)>d_xz(goal_pos, gps_b): #or displacement < -ALIGNMENT_THRESHOLD : #and gps_f[2] - m*gps_f[0] - q >= 0:
                    
                                #if d_xz(goal_pos, gps_f)>d_xz(goal_pos, gps_b):
                        #print('USING ORIENTATION 1')
                        STATE_DIAGNOSTICS = STATE_DIAGNOSTICS + ' Orientation 1.0 conditions fulfilled.'
                        STEERING_OPERATION = 'Turn Left'
                        #print('Orientation: {}'.format()
                        #turn_left_on_place(motors)
                        turn_left(motors, SMOOTHING_FACTOR)
                        #return GHOST_DETECTED, OBSTACLE_DETECTED

                    else:
                        move_forward_with_love(ds_l_val, ds_r_val, motors, SMOOTHING_FACTOR)
                        STATE_DIAGNOSTICS = STATE_DIAGNOSTICS + ' Orientation 1.1 conditions fulfilled.'
                        STEERING_OPERATION = 'Move forward with love'    
                        #return distance, displacement
                            #elif orientation[0] < -ORIENTATION_THRESHOLD: #-orientation(d_xz(goal_pos, gps_f)):
                #elif orientation[0]*orientation[3] < orientation[2]*orientation[3] and orientation[2]*orientation[3]>calculated_orientation:    
                elif position[2] < 0 and position[0] > 0 and d_xz(gps_val, goal_pos)>0.5:
                    #print('USING ORIENTATION 2')
                    #print('Orientation: {}'.format())
                    #turn_right_on_place(motors)     
                    turn_right(motors, SMOOTHING_FACTOR)
                    #return GHOST_DETECTED, OBSTACLE_DETECTED
                    STATE_DIAGNOSTICS = STATE_DIAGNOSTICS + ' Orientation 2.1 conditions fulfilled.'
                    STEERING_OPERATION = 'Turn right'
                    ####
                
                else:
                    print('Orientation is ok. Lets move forward')
                    print('Orientation on X and Z: {}, {}').format(orientation[0], orientation[2])
                    move_forward_with_love(ds_l_val, ds_r_val, motors, SMOOTHING_FACTOR)
                    STATE_DIAGNOSTICS = STATE_DIAGNOSTICS + ' Orientation 3.1 conditions fulfilled. Orientation is ok.'
                    STEERING_OPERATION = 'Move forward with love'
                    #return GHOST_DETECTED, OBSTACLE_DETECTED
            # if the object detected on camera is not the goal, do nothing 
            else:
                pass
                #explore(ls_l_val, ls_r_val, ds_r_val, ds_l_val, ds_r_side_f_val, ds_r_side_b_val, ds_l_side_f_val, ds_l_side_b_val, motors)
                
                
                
                
                
                    
        elif len(obj_ls)==0:
            STATE_DIAGNOSTICS = 'No object detected on Camera.'
            distance = d_xz(GOAL_POSITION, gps_f)
            print('No object detected!')
            print('Disallineamento: {}'.format(gps_f[2] - m*gps_f[0] - q))
            print(d_xz(goal_pos, gps_f),d_xz(goal_pos, gps_b))
            if displacement >= ALIGNMENT_THRESHOLD: #or displacement < -ALIGNMENT_THRESHOLD : #and gps_f[2] - m*gps_f[0] - q >= 0:
                if not d_xz(goal_pos, gps_f)<d_xz(goal_pos, gps_b):
                    print('USING ORIENTATION')
                    print('Disallineamento: {}'.format(gps_f[2] - m*gps_f[0] - q))
                    turn_left_on_place(motors, SMOOTHING_FACTOR)
                    STATE_DIAGNOSTICS = STATE_DIAGNOSTICS + ' Orientation 4.1 conditions fulfilled.'
                    STEERING_OPERATION = 'Turn left on the spot'
                    #return GHOST_DETECTED, OBSTACLE_DETECTED
                    
                else:
                    #print('Turn Left')
                    turn_left(motors, SMOOTHING_FACTOR)
                    STATE_DIAGNOSTICS = STATE_DIAGNOSTICS + ' Orientation 4.2 conditions fulfilled.'
                    STEERING_OPERATION = 'Turn left'
                    #return GHOST_DETECTED, OBSTACLE_DETECTED
        
        
            elif - displacement > ALIGNMENT_THRESHOLD: #or displacement < -ALIGNMENT_THRESHOLD : #and gps_f[2] - m*gps_f[0] - q >= 0:
                if not d_xz(goal_pos, gps_f)<d_xz(goal_pos, gps_b):
                    print('USING DISPLACEMENT. SHOULD ALIGN. TURN RIGHT')
                    print('Disallineamento: {}'.format(gps_f[2] - m*gps_f[0] - q))
                    turn_right_on_place(motors, SMOOTHING_FACTOR)
                    STATE_DIAGNOSTICS = STATE_DIAGNOSTICS + ' Orientation 5.1 conditions fulfilled.'
                    STEERING_OPERATION = 'Turn right on the spot'
                    #return GHOST_DETECTED, OBSTACLE_DETECTED
                    
                else:
                    print('Turn right')
                    turn_right(motors, SMOOTHING_FACTOR)
                    STATE_DIAGNOSTICS = STATE_DIAGNOSTICS + ' Orientation 5.2 conditions fulfilled.'
                    STEERING_OPERATION = 'Turn right'
                    #return GHOST_DETECTED, OBSTACLE_DETECTED
            else:
                print('Moving Forward with love')

                #align_w_goal(gps_val, gps_b_val, GOAL_POSITION, motors, ALIGNMENT_THRESHOLD=0.0001)
                move_forward_with_love(ds_l_val, ds_r_val, motors, SMOOTHING_FACTOR)
                STATE_DIAGNOSTICS = STATE_DIAGNOSTICS + ' Orientation 6.1 conditions fulfilled. Orientation is ok.'
                STEERING_OPERATION = 'Move forward with love'
                #return GHOST_DETECTED, OBSTACLE_DETECTED
        
    elif object_detected(ds_r_val, ds_l_val, ds_r_side_f_val, ds_l_side_f_val, ds_r_side_b_val, ds_l_side_b_val, obj_ls):
        
        GHOST_DETECTED = False
        OBSTACLE_DETECTED = True
        STATE_DIAGNOSTICS = 'Avoid obstacle.'
        STEERING_OPERATION = None
        SMOOTHNESS_FACTOR = 2
        ## DETECT RIGHT SIDE, TURN LEFT
        # if the robot detects something near right, turns left
        if object_near_front_or_lateral_right(ds_r_val, ds_r_side_f_val):
            turn_left(motors, SMOOTHNESS_FACTOR)
            STATE_DIAGNOSTICS = STATE_DIAGNOSTICS + ' Obstacle near front or lateral right'
            STEERING_OPERATION = 'Turn left'
            #return GHOST_DETECTED, OBSTACLE_DETECTED
            
        # if the robot detects an obstacle very near right, turns left on the spot
        elif object_very_near_front_right(ds_r_val) or object_very_near_lateral_right(ds_r_side_f_val):
            turn_left_on_place(motors, SMOOTHNESS_FACTOR)
            STEERING_OPERATION = 'Turn left on place'
            STATE_DIAGNOSTICS = STATE_DIAGNOSTICS + ' Obstacle very near front right'
            #return GHOST_DETECTED, OBSTACLE_DETECTED
        ## DETECT LEFT SIDE; TURN RIGHT
        elif object_near_front_or_lateral_left(ds_l_val, ds_l_side_f_val):
            print('CP-1')
            turn_right(motors, SMOOTHNESS_FACTOR)
            STEERING_OPERATION = 'Turn right'
            STATE_DIAGNOSTICS = STATE_DIAGNOSTICS + ' Obstacle near front or lateral left'

            #return GHOST_DETECTED, OBSTACLE_DETECTED
            
        elif object_very_near_front_left(ds_l_val) or object_very_near_lateral_left(ds_l_side_f_val):
            print('CP-2')
            turn_right_on_place(motors, SMOOTHNESS_FACTOR)
            #return GHOST_DETECTED, OBSTACLE_DETECTED
            STEERING_OPERATION = 'Turn right'
            STATE_DIAGNOSTICS = STATE_DIAGNOSTICS + ' Obstacle very near front or lateral left'

            
            
    elif ghost_detected(ls_l_val, ls_r_val):
        
        GHOST_DETECTED = True
        OBSTACLE_DETECTED = False
        STEERING_OPERATION = 'Always turn left'
        STATE_DIAGNOSTICS = 'Avoid ghost'

        avoid_ghost(motors, ls_l_val, ls_r_val, smoothness_factor = 4)
        
        #return GHOST_DETECTED, OBSTACLE_DETECTED
        
    else:
        STATE_DIAGNOSTICS = None
        STEERING_OPERATION = 'Move forward with love'
        move_forward_with_love(ds_l_val, ds_r_val, motors)
        
    return GHOST_DETECTED, OBSTACLE_DETECTED, STATE_DIAGNOSTICS, STEERING_OPERATION
        
                    
def move_forward_with_love(ds_l, ds_r, motors, smoothness_factor=1):
    # manage right motors
    if ds_l > 700 and ds_l < 1000:
        motors[1].setVelocity(MAX_VELOCITY/smoothness_factor)
        motors[3].setVelocity(MAX_VELOCITY/smoothness_factor)
    
    elif ds_l ==1000:
        motors[1].setVelocity(MAX_VELOCITY/smoothness_factor)
        motors[3].setVelocity(MAX_VELOCITY/smoothness_factor)
    
    else:
    
        motors[0].setVelocity(0)
        motors[2].setVelocity(0)
        motors[1].setVelocity(0)
        motors[3].setVelocity(0)
        
    # manage left motors
    if ds_r > 700 and ds_r < 1000:
        motors[0].setVelocity(MAX_VELOCITY/smoothness_factor)
        motors[2].setVelocity(MAX_VELOCITY/smoothness_factor)
        # manage right motors
        #motors[1].setVelocity((ds_l/1000)*MAX_VELOCITY/(2*smoothness_factor))
        #motors[3].setVelocity((ds_l/1000)*MAX_VELOCITY/(2*smoothness_factor))
    elif ds_r ==1000:
        motors[0].setVelocity(MAX_VELOCITY/smoothness_factor)
        motors[2].setVelocity(MAX_VELOCITY/smoothness_factor)
    
    else:
        motors[0].setVelocity(0)
        motors[2].setVelocity(0)
        motors[1].setVelocity(0)
        motors[3].setVelocity(0)



def orientation_with_distance(distance):
    if distance >= 3:
        return 1/57.4 #0.002
        
    elif distance < 3 and distance >= 2:
        return 0.1/57.4#0.000275
    
    elif distance < 2 and distance >= 1:
        return 0.075/57.4#0.00025 
       
    else: 
        return 0.05/57.4#0.000155
  
       
       
        
def explore(ls_l_val, ls_r_val, ds_r_val, ds_l_val, ds_r_side_f_val, ds_r_side_b_val, ds_l_side_f_val, ds_l_side_b_val, motors):
# GHOST DETECTION
    if ghost_detected(ls_l_val, ls_r_val):
        avoid_ghost(motors, ls_l_val, ls_r_val, smoothness_factor = 2)
        print('GHOST_DETECTED')
    # EXPLORE navigation   
    else:
    
        # object detected at mild distance in front or laterally on the right side
        if (ds_r_val < 1000 and ds_r_val > 500) or ( ds_r_side_f_val<600 and ds_r_side_f_val>300):
            turn_left(motors, 2)
            operation = 'TURN_LEFT'
            print(operation)
        
        # object detected very near in front on the right side
        elif ds_r_val < 500: # or ds_r_side_f_val < 250:
            #if  ds_r_side_f_val - ds_r_side_b_val > 0 and ds_r_side_f_val - ds_r_side_b_val < 10:
            #    move_forward(motors, MAX_VELOCITY)
            #    state = 'FORWARD'
                
            #else:
            turn_left_on_place(motors)
            operation = 'ROTATE_LEFT_ON_SPOT'
            print(operation)
            
        #  SPERIMENTALE  
        # object detected very near laterally on the right side
        elif ds_r_side_f_val <= 300:
            # if the object is parallel to the robot on the right side
            if ds_r_side_f_val - ds_r_side_b_val > 0 and ds_r_side_f_val - ds_r_side_b_val < 10:
                move_forward(motors, MAX_VELOCITY)
            else:
                turn_left_on_place(motors)
        
        ###############
        
        elif (ds_l_val < 1000 and ds_l_val > 500) or ( ds_l_side_f_val<600 and ds_l_side_f_val>300):
            turn_right(motors, 2)
            operation = 'TURN_LEFT'
            print(operation)
        
        # object detected very near in front on the right side
        elif ds_l_val < 500: # or ds_r_side_f_val < 250:
            #if  ds_r_side_f_val - ds_r_side_b_val > 0 and ds_r_side_f_val - ds_r_side_b_val < 10:
            #    move_forward(motors, MAX_VELOCITY)
            #    state = 'FORWARD'
                
            #else:
            turn_right_on_place(motors)
            operation = 'ROTATE_LEFT_ON_SPOT'
            print(operation)
            
        #  SPERIMENTALE  
        # object detected very near laterally on the right side
        elif ds_l_side_f_val <= 300:
            # if the object is parallel to the robot on the right side
            if ds_l_side_f_val - ds_l_side_b_val > 0 and ds_l_side_f_val - ds_l_side_b_val < 10:
                move_forward(motors, MAX_VELOCITY)
            else:
                turn_right_on_place(motors)
       
       
            
       ################
            
        else:
            move_forward(motors, MAX_VELOCITY)
            operation = 'FORWARD'
            print(operation)

#### Useful constants    
MAX_VELOCITY = 20
GOAL_DETECTED = False
GOAL_POSITION = None
GHOST_DETECTED = False
OBSTACLE_DETECTED = False
STATE_DIAGNOSTICS = None
STEERING_OPERATION = None
#timestep = int(robot.getBasicTimeStep())
TIMESTEP = 64


def robot_controller(robot, GOAL_DETECTED, GOAL_POSITION, GHOST_DETECTED, OBSTACLE_DETECTED):
    
    motors_names = ['rm_lf', 'rm_rf', 'rm_lb', 'rm_rb' ]
    motors = []
    wheels_names = ['wheel_lf', 'wheel_rf', 'wheel_lb', 'wheel_rb']
    wheels = []
    ls_names = ['ls_l', 'ls_r']
    ls = []
    
    ds_names = ['ds_l', 'ds_r', 'ds_r_side_f', 'ds_r_side_b', 'ds_l_side_f', 'ds_l_side_b']
    ds = []
    
    camera = []
    
    # enabling the motors
    for engine_ix in range(len(wheels_names)):
        motors.append(robot.getDevice(wheels_names[engine_ix]))
        #print(motors[engine_ix])
        motors[engine_ix].setPosition(float('inf'))
        motors[engine_ix].setVelocity(0.0)
        
    # enabling the light sensors
    for ix in range(len(ls_names)):
        ls.append(robot.getDevice(ls_names[ix]))
        ls[ix].enable(TIMESTEP/2)

    # enabling the distance sensors
    for ix in range(len(ds_names)):
        ds.append(robot.getDevice(ds_names[ix]))
        ds[ix].enable(TIMESTEP)

    camera.append(robot.getDevice('camera'))
    camera[0].enable(TIMESTEP)
    camera[0].recognitionEnable(TIMESTEP)
    print(camera[0].hasRecognition()) # delete it
    
    gps, gps_b = robot.getDevice('gps'), robot.getDevice('gps_b')
    gps.enable(TIMESTEP)
    gps_b.enable(TIMESTEP)
    #ds_l = robot.getDevice('ds_l')
    #ds_r = robot.getDevice('ds_r')

   
    #for ix in range(len(ls_names)):
    #    ls_measurement.append(robot.getLightSensor(ls_names(ix)))
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(TIMESTEP) != -1:
        
        # Read the sensors:     
        ls_l_val, ls_r_val = ls[0].getValue(), ls[1].getValue()
        ds_l_val, ds_r_val = ds[0].getValue(), ds[1].getValue()
        
        ds_r_side_f_val, ds_r_side_b_val  = ds[2].getValue(), ds[3].getValue()
        ds_l_side_f_val, ds_l_side_b_val  = ds[4].getValue(), ds[5].getValue()
        
        gps_val = gps.getValues()
        gps_b_val = gps_b.getValues()
        
        cam_val = camera[0].getRecognitionObjects()
        
        #######TESTING THE detect_goal function#####
        GOAL_DETECTED, GOAL_POSITION = detect_goal(cam_val, GOAL_DETECTED, GOAL_POSITION, robot, gps_val)
        #######FINE TEST#######
        
        
        # Process sensor data here.
    
        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)
        #if (ds_r_val < 1000 and ds_r_val>750) or ( ds_r_side_f_val<300 and ds_r_side_f_val>250):
        #    turn_left(motors, smoothness_factor=4)
        #    state = 'TURN_LEFT_SPEED_1'
        if GOAL_DETECTED:
            GHOST_DETECTED, OBSTACLE_DETECTED, STATE_DIAGNOSTICS, STEERING_OPERATION = move_towards_goal(ls_l_val, ls_r_val, 
            ds_r_val, ds_l_val, 
            ds_r_side_f_val, ds_l_side_f_val, 
            ds_r_side_b_val, ds_l_side_b_val, gps_val, gps_b_val, cam_val, motors, GOAL_POSITION, GHOST_DETECTED, OBSTACLE_DETECTED)
            #move_forward_with_love(ds_l_val, ds_r_val, motors, smoothness_factor=1)
        
        else:
            # GHOST DETECTION
            if ghost_detected(ls_l_val, ls_r_val):
                avoid_ghost(motors, ls_l_val, ls_r_val, smoothness_factor = 10)
                print('GHOST_DETECTED')
            
            # EXPLORE navigation   
            else:
                explore(ls_l_val, ls_r_val, ds_r_val, ds_l_val, ds_r_side_f_val, ds_r_side_b_val, ds_l_side_f_val, ds_l_side_b_val, motors)
            
        print('------------------------------------------------------')
        print('------------------------------------------------------\n')
        print(' ')
        
        print('SENSORS')
        print(' ')
        print('Light Sensors L/R : {:.3f}, {:.3f}'.format(ls_l_val, ls_r_val))
        print('Distance Sensors Front L/R: {:.3f}, {:.3f}'.format(ds_l_val, ds_r_val))
        print('Distance Sensors R Side B/F: {:.3f}, {:.3f}'.format(ds_r_side_b_val, ds_r_side_f_val))
        print('Distance Sensors L Side B/F: {:.3f}, {:.3f}'.format(ds_l_side_b_val, ds_l_side_f_val))
        
        print(' ')
        print('CAMERA')
        print(' ')
        print('N of objects detected: {}'.format(len(cam_val)))
        if len(cam_val)>0:
            #print('id, name, color: {}, {}, {}'.format(cam_val[0].get_id(), cam_val[0].get_model(), cam_val[0].get_colors()))
            print('Detected object model: {}'.format(cam_val[0].get_model()))
            print('Detected object RGB color: {}'.format(cam_val[0].get_colors()))
        
        print(' ')
        print('WORLD VARIABLES')
        print(' ')
        print('Goal Detected: {}'.format(GOAL_DETECTED))
        print('Goal coordinates: {}'.format(GOAL_POSITION))
        
        print(' ')
        print('ROBOTS VARIABLES')
        print(' ')
        print('Front/Back GPS coordinates: {}, {}'.format(gps_val, gps_b_val))
        print('Front Wheels Speed L/R: {}'.format(00000))
        print('Back Wheels Speed L/R: {}'.format(00000))

        
        #if GOAL_DETECTED:
        #    print('Robot-Goal euclidean distance: {}'.format(GHOST_DETECTED))
        #    sign = lambda i: '+' if i >=0 else '-'

        #    print('Joining-line explicit equation: z = {}x {}'.format(str(sign(2))+str(abs(2)),sign(1) + str(abs(1))))
        #    print('Robot-Goal disalignment: {}'.format(OBSTACLE_DETECTED))
        
        print(' ')
        print('ROBOTS STATE')
        print('Ghost Detected: {}'.format(GHOST_DETECTED))
        print('Obstacle Detected: {}'.format(OBSTACLE_DETECTED))
        print('State diagnostics: {}'.format(STATE_DIAGNOSTICS))
        print('Steering operation: {}'.format(STEERING_OPERATION))
        print(' ')        
            

if __name__ == '__main__':
    robot = Robot()
    robot_controller(robot, GOAL_DETECTED, GOAL_POSITION, GHOST_DETECTED, OBSTACLE_DETECTED)        

