''' movement functions '''
'''
    holonomic drive (move robot in a certain angle for a certain distance, while facing forward): chassis_ctrl.move_with_distance(angle, distance)
    move arm to absolute position: robotic_arm_ctrl.moveto(x_coordinate, y_coordinate)
    move arm to relative position: robotic_arm_ctrl.move(x_coordinate, y_coordinate)
    rotate on the spot: chassis_ctrl.rotate_with_degree(direction, angle)
    control all speeds: chassis_ctrl.move_with_speed(fb_speed, lr_speed, rotational_speed)
'''



picked_up = False
pickup_params = []

# Vision Markers
# CHANGE HERE
luggage1 = 1
luggage2 = 2
left = 3
right = 4
intersection = 5
dropoff = 6


VM_List_picked_up = [luggage1, luggage2, left, right, intersection, dropoff] #the list of valid vision markers
VM_List_no_picked_up = [luggage1, luggage2, left, right, intersection] 

curr_luggage = 0
num_luggage1 = 0
num_luggage2 = 0

distance = 1900 #set to a random large number
def start(): 
    gripper_ctrl.update_power_level(4) # max gripping power
    led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 0, 0, rm_define.effect_always_on) 
    while (not gripper_ctrl.is_open()): 
        gripper_ctrl.open()
    global picked_up 
    global pickup_params 
    global distance 
    see_VM()

def drop(): 
    global picked_up 
    global pickup_params 
    global distance 
    dist = 0.2
    chassis_ctrl.stop() 
    if picked_up == True: 
        # move forward into luggage deposit area
        chassis_ctrl.move_with_distance(0, dist) 
        # lower claw
        robotic_arm_ctrl.moveto(pickup_params[0], pickup_params[1], wait_for_complete=True) 
         # open claw
        while (not gripper_ctrl.is_open()): 
            gripper_ctrl.open() 
        print("Dropped") 
        picked_up = False 
        # reset claw position
        robotic_arm_ctrl.recenter() 
        # move backwards to prevent hitting luggage upon reversing
        chassis_ctrl.move_with_distance(180, dist * 2) 
        # reverse robot
        chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 180) 
        distance = ir_distance_sensor_ctrl.get_distance_info(1) 
    else: 
        print("Nothing to drop") 

def see_and_pickup(vm): 
    global distance 
    global picked_up 
    marker_List = [] 
    # PID object for centering robot to vision marker
    pid_Centralise_Marker = PIDCtrl() 
    # lower claw to default position
    # TO BE CALIBRATED
    if picked_up == False: 
        robotic_arm_ctrl.moveto(180, -70, wait_for_complete=True)
    vision_ctrl.enable_detection(rm_define.vision_detection_marker) 
    vision_ctrl.set_marker_detection_distance(3) 
    ir_distance_sensor_ctrl.enable_measure(1) 
    pid_Centralise_Marker.set_ctrl_params(100, 1, 0) 
    chassis_ctrl.set_trans_speed(1) 
    state = True 
    error = 100 
    distance = ir_distance_sensor_ctrl.get_distance_info(1)
    while state: 
        marker_List = vision_ctrl.get_marker_detection_info() 
        distance = ir_distance_sensor_ctrl.get_distance_info(1)
        print(marker_List, distance, error) 
        if (len(marker_List)>2): 
            
            x = marker_List[marker_List.index(vm) +1] 
            error = x-0.5 
            #value to be calibrated 
            pid_Centralise_Marker.set_error(error) 
            pid_output = pid_Centralise_Marker.get_output() 
            if distance > 11:
                x_speed = 0.1 
            else: 
                x_speed = 0 
            if abs(error)>0.01: 
                chassis_ctrl.move_with_speed(0, 0, pid_output)
            else: 
                chassis_ctrl.move_with_speed(x_speed, 0, 0) 
                print("Moving with speed", x_speed) 
                print("Distance is ", distance)
        else:
            # stop once robot has fully aligned to vision marker and is close enough
            chassis_ctrl.stop()
            if (distance >0 and distance <11 and abs(error)<0.3): 
                print("Moving to pick up, distance is", distance) 
                chassis_ctrl.move_with_distance(0, 0.1) 
                print("Moved forward by 0.10") 
                chassis_ctrl.stop()
                print("Ready to Pick Up")
                state = False
                
    #reset distance 
    distance = 100 
    # fully open claw just in case, not really necessary
    while (not gripper_ctrl.is_open()): 
        gripper_ctrl.open() 
    # raise claw by 6cm to grab top of luggage
    # TO BE CALIBRATED
    robotic_arm_ctrl.move(0, 60, wait_for_complete=True)
    global pickup_params 
    pickup_params = robotic_arm_ctrl.get_position() 
    while (not gripper_ctrl.is_closed()): 
        gripper_ctrl.close() 
    

    # raise claw by another 5cm after closing
    # TO BE CALIBRATED
    robotic_arm_ctrl.move(0, 50, wait_for_complete=True)
    picked_up = True 
    distance = ir_distance_sensor_ctrl.get_distance_info(1)
    # reverse robot
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 180) 


def see_VM():
    global distance 
    global curr_luggage
    marker_List = [] 
    pid_Centralise_Marker = PIDCtrl() 
    robotic_arm_ctrl.recenter() 
    vision_ctrl.enable_detection(rm_define.vision_detection_marker) 
    vision_ctrl.set_marker_detection_distance(3)
    media_ctrl.exposure_value_update(rm_define.exposure_value_large)
    ir_distance_sensor_ctrl.enable_measure(1) 
    pid_Centralise_Marker.set_ctrl_params(100, 1, 0) 
    state = True
    direction = ''
    to_drop = False
    global picked_up_misaligned
    global picked_up 
    while state:
        # turn at closer dist when luggage is picked up since ir sensor is further forward
        # MAY NEED TO BE CALIBRATED
        if picked_up == True:
            min_dist = 13 
            VM_List = VM_List_picked_up 
        else:
            min_dist = 17 
            VM_List = VM_List_no_picked_up
        marker_List = vision_ctrl.get_marker_detection_info() 
        distance = ir_distance_sensor_ctrl.get_distance_info(1)
        print(marker_List, distance)
        lost_vm = True 
        if (len(marker_List)>2 and marker_List[1] in VM_List):
            #first marker in list 
            x = marker_List[2] 
            if ((luggage1 in marker_List or luggage2 in marker_List) and picked_up ==False):
                chassis_ctrl.stop()  
                if luggage1 in marker_List:
                    curr_luggage = 1
                    see_and_pickup(luggage1) 
                else:
                    curr_luggage = 2
                    see_and_pickup(luggage2) 
            elif (right in marker_List): 
                direction = 'right'
            elif (left in marker_List):
                direction = 'left' 
            elif (intersection in marker_List):
                if curr_luggage == 1:
                    direction = 'left'
                elif curr_luggage == 2:
                    direction = 'right'
                else:
                    print("NO LUGGAGE")
                    # go right by default
                    direction = 'right'
            elif (marker_List[1] == dropoff and picked_up == True):  
                to_drop = True 
                print(to_drop) 
                
            error = x-0.5 #value to be calibrated 
            pid_Centralise_Marker.set_error(error) 
            pid_output = pid_Centralise_Marker.get_output()
            distance = ir_distance_sensor_ctrl.get_distance_info(1) 
            print("got VM, big error", distance) 
            if (distance > min_dist and picked_up ==False): 
                x_speed = 0.5 
            elif (distance >min_dist and picked_up ==True): 
                x_speed = 0.3 
            else: 
                x_speed = 0 
            #if correct vision marker seen, move towards the vision marker
            chassis_ctrl.move_with_speed(x_speed, 0, pid_output) 
            print("Correcting direction") 
            lost_vm = False
            
        if (to_drop == True and distance >0 and distance <min_dist): 
            print("Dropping") 
            drop() 
            lost_vm = False 
            to_drop = False
        elif (direction == 'left' and distance >0 and distance <min_dist):
            chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, 90) 
            print("Rotating left") 
            print("Rotation complete") 
            direction = '' 
            lost_vm = False
        elif (direction == 'right' and distance >0 and distance <min_dist): 
            print("Rotating right") 
            chassis_ctrl.rotate_with_degree(rm_define.clockwise, 90) 
            print("Rotation complete") 
            direction = '' 
            lost_vm = False
        if lost_vm: 
            print("LOST VM") 
            if picked_up == False: 
                chassis_ctrl.move_with_speed(0.3, 0, 0) 
                #move slower in case there is a humanoid 
            else: 
                chassis_ctrl.move_with_speed(0.4, 0, 0)
                
start()


'''
IMPORTANT SHIT TO TEST WITH ACTUAL ROBOT

POSITION OF RAISED AND LOWERED CLAW
'''
