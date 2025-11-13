from controller import Robot, Keyboard
from math import cos, sin
from pid_controller import pid_velocity_fixed_height_controller
import os
import cv2
import numpy as np
import random
import shutil
from collections import deque

FLYING_ATTITUDE = 0.5  # target altitude in meters

# ================== IMAGE SAVE CONFIG ==================

OUTPUT_DIR = "/home/intellisense08/Yehan/project_dir/Webots/Root_data/train"
VALIDATION_DIR = os.path.join(OUTPUT_DIR, "/home/intellisense08/Yehan/project_dir/Webots/Root_data/validation/")


LEFT_BEND_DIR = os.path.join(OUTPUT_DIR, "1_0_0")
LEFT_JUNC_DIR = os.path.join(OUTPUT_DIR, "1_0_1")
RIGHT_BEND_DIR = os.path.join(OUTPUT_DIR, "0_1_0")
RIGHT_JUNC_DIR = os.path.join(OUTPUT_DIR, "0_1_1")
T_JUNC_DIR = os.path.join(OUTPUT_DIR, "1_1_0")
FOUR_W_DIR = os.path.join(OUTPUT_DIR, "1_1_1")
STRAIGHT_DIR = os.path.join(OUTPUT_DIR, "0_0_1")
DEAD_END = os.path.join(OUTPUT_DIR, "0_0_0")

os.makedirs(LEFT_BEND_DIR, exist_ok=True)
os.makedirs(LEFT_JUNC_DIR, exist_ok=True)
os.makedirs(RIGHT_BEND_DIR, exist_ok=True)
os.makedirs(RIGHT_JUNC_DIR, exist_ok=True)
os.makedirs(STRAIGHT_DIR, exist_ok=True)
os.makedirs(FOUR_W_DIR, exist_ok=True)
os.makedirs(T_JUNC_DIR, exist_ok=True)
os.makedirs(DEAD_END, exist_ok=True)

CLASS_DIRS = {
    "1_0_0": os.path.join(OUTPUT_DIR, "1_0_0"),
    "1_0_1": os.path.join(OUTPUT_DIR, "1_0_1"),
    "0_1_0": os.path.join(OUTPUT_DIR, "0_1_0"),
    "0_1_1": os.path.join(OUTPUT_DIR, "0_1_1"),
    "1_1_0": os.path.join(OUTPUT_DIR, "1_1_0"),
    "1_1_1": os.path.join(OUTPUT_DIR, "1_1_1"),
    "0_0_1": os.path.join(OUTPUT_DIR, "0_0_1"),
    "0_0_0": os.path.join(OUTPUT_DIR, "0_0_0"),
}

#Make directories
for d in CLASS_DIRS.values():
    os.makedirs(d, exist_ok=True)

frame_counter = 0
save_distance_before_wp = 1.15 # meters before waypoint to start saving images

def save_camera_image(camera, folder, counter):
    image = camera.getImage()
    if image:
        width = camera.getWidth()
        height = camera.getHeight()
        img = np.frombuffer(image, np.uint8).reshape((height, width, 4))
        img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
        filename = os.path.join(folder, f"a100_env1_ms_50_{counter:06d}.jpg")
        cv2.imwrite(filename, img_bgr)

def move_validation_split():
    """After mission is complete, move 20% of images from each class into validation/<class_name>"""
    print("\n=== Splitting dataset: moving 20% into validation folders ===")
    for class_name, folder in CLASS_DIRS.items():
        validation_class_dir = os.path.join(VALIDATION_DIR, class_name)
        os.makedirs(validation_class_dir, exist_ok=True)

        images = [f for f in os.listdir(folder) if f.lower().endswith(('.jpg', '.png'))]
        if not images:
            continue

        sample_size = max(1, int(0.2 * len(images)))
        selected = random.sample(images, sample_size)

        for img in selected:
            src = os.path.join(folder, img)
            dst = os.path.join(validation_class_dir, img)
            shutil.move(src, dst)

        print(f"Moved {len(selected)} images from {class_name} → {validation_class_dir}")
    print("=== Validation split complete ===")

if __name__ == '__main__':

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Initialize motors
    m1_motor = robot.getDevice("m1_motor"); m1_motor.setPosition(float('inf')); m1_motor.setVelocity(-1)
    m2_motor = robot.getDevice("m2_motor"); m2_motor.setPosition(float('inf')); m2_motor.setVelocity(1)
    m3_motor = robot.getDevice("m3_motor"); m3_motor.setPosition(float('inf')); m3_motor.setVelocity(-1)
    m4_motor = robot.getDevice("m4_motor"); m4_motor.setPosition(float('inf')); m4_motor.setVelocity(1)

    # Initialize Sensors
    imu = robot.getDevice("inertial_unit"); imu.enable(timestep)
    gps = robot.getDevice("gps"); gps.enable(timestep)
    gyro = robot.getDevice("gyro"); gyro.enable(timestep)
    camera = robot.getDevice("camera"); camera.enable(timestep)
    range_front = robot.getDevice("range_front"); range_front.enable(timestep)
    range_left = robot.getDevice("range_left"); range_left.enable(timestep)
    range_back = robot.getDevice("range_back"); range_back.enable(timestep)
    range_right = robot.getDevice("range_right"); range_right.enable(timestep)

    # Get keyboard
    keyboard = Keyboard(); keyboard.enable(timestep)

    # Initialize variables
    past_x_global = 0
    past_y_global = 0
    past_time = 0
    first_time = True

    # Crazyflie velocity PID controller
    PID_crazyflie = pid_velocity_fixed_height_controller()
    height_desired = FLYING_ATTITUDE

    autonomous_mode = False

    # ================== WAYPOINT NAVIGATION ==================
    waypoints = [
        (-2.91,-2.47,FLYING_ATTITUDE),#0
        (1.51, -2.48, FLYING_ATTITUDE),
        (6.05, -2.2, FLYING_ATTITUDE),
        (6.03, -0.588, FLYING_ATTITUDE),
        (1.41, -0.58, FLYING_ATTITUDE),
        (-3.08, -0.573, FLYING_ATTITUDE),
        (-3.07, 1.84, FLYING_ATTITUDE),
        (1.51,1.83, FLYING_ATTITUDE),
        (6.09 ,1.83, FLYING_ATTITUDE),
        (6.14 ,3.84, FLYING_ATTITUDE),
        (1.49 ,3.83, FLYING_ATTITUDE),
        (-2.55 ,3.96, FLYING_ATTITUDE), #11
        (1.49 ,3.83, FLYING_ATTITUDE),
        (6.14 ,3.84, FLYING_ATTITUDE),
        (6.09 ,1.83, FLYING_ATTITUDE),
        (1.51,1.83, FLYING_ATTITUDE),
        (-3.07, 1.84, FLYING_ATTITUDE),
        (-3.08, -0.573, FLYING_ATTITUDE),
        (1.41, -0.58, FLYING_ATTITUDE),
        (6.03, -0.588, FLYING_ATTITUDE),
        (6.11, -2.4, FLYING_ATTITUDE),
        (1.51, -2.48, FLYING_ATTITUDE),
    ]
    current_wp_index = 0
    wp_tolerance = 0.3   # meters

    # Rotation config
    yaw_rotation_points = {
        2: {'direction': +1, 'cycles': 80},   
        3: {'direction': +1, 'cycles': 80},   
        5: {'direction': -1, 'cycles': 80},
        6: {'direction': -1, 'cycles': 80},
        8: {'direction': +1, 'cycles': 80},
        9: {'direction': +1, 'cycles': 80},
        11: {'direction': +1, 'cycles': 165},
        13: {'direction': -1, 'cycles': 80},
        14: {'direction': -1, 'cycles': 80},
        16:{'direction': +1, 'cycles': 80},
        17:{'direction': +1, 'cycles': 70},
        19: {'direction': -1, 'cycles': 80},
        20: {'direction': -1, 'cycles': 80},
        
    }

    rotate_counter = 0
    rotate_limit = 0
    rotate_direction = 0
    rotating = False

    # Wiggle state
    wiggling = False
    wiggle_steps = []
    wiggle_counter = 0
    wiggle_delay = 0.1
    last_wiggle_time = 0
    straight_buffer = {}

    print("\n====== Controls =======\n")
    print("- Arrow keys: move drone manually in XY")
    print("- Q/E: rotate around yaw")
    print("- W/S: go up and down")
    print("- Press A: enable autonomous waypoint navigation")
    print("- Press D: disable autonomous mode\n")

    # ================== MAIN LOOP ==================
    while robot.step(timestep) != -1:

        dt = robot.getTime() - past_time
        if first_time:
            past_x_global = gps.getValues()[0]
            past_y_global = gps.getValues()[1]
            past_time = robot.getTime()
            first_time = False

        # Get sensor data
        roll, pitch, yaw = imu.getRollPitchYaw()
        yaw_rate = gyro.getValues()[2]
        x_global = gps.getValues()[0]
        y_global = gps.getValues()[1]
        altitude = gps.getValues()[2]

        v_x_global = (x_global - past_x_global) / dt
        v_y_global = (y_global - past_y_global) / dt

        cos_yaw = cos(yaw)
        sin_yaw = sin(yaw)
        v_x = v_x_global * cos_yaw + v_y_global * sin_yaw
        v_y = -v_x_global * sin_yaw + v_y_global * cos_yaw

        forward_desired = 0
        sideways_desired = 0
        yaw_desired = 0
        height_diff_desired = 0

        # Manual control
        key = keyboard.getKey()
        while key > 0:
            if key == Keyboard.UP:
                forward_desired += 0.5
            elif key == Keyboard.DOWN:
                forward_desired -= 0.5
            elif key == Keyboard.RIGHT:
                sideways_desired -= 0.5
            elif key == Keyboard.LEFT:
                sideways_desired += 0.5
            elif key == ord('Q'):
                yaw_desired = +1
            elif key == ord('E'):
                yaw_desired = -1
            elif key == ord('W'):
                height_diff_desired = 0.1
            elif key == ord('S'):
                height_diff_desired = -0.1
            elif key == ord('A'):
                if not autonomous_mode:
                    autonomous_mode = True
                    current_wp_index = 0
                    print("Autonomous mode: ON")
            elif key == ord('D'):
                if autonomous_mode:
                    autonomous_mode = False
                    print("Autonomous mode: OFF")
            key = keyboard.getKey()

        height_desired += height_diff_desired * dt

        # ================== AUTONOMOUS WAYPOINT NAV ==================
        if autonomous_mode and current_wp_index < len(waypoints):
            target_x, target_y, target_z = waypoints[current_wp_index]

            # errors in world frame
            error_x = target_x - x_global
            error_y = target_y - y_global
            error_z = target_z - altitude

            # transform error to body frame
            error_x_body = error_x * cos_yaw + error_y * sin_yaw
            error_y_body = -error_x * sin_yaw + error_y * cos_yaw

            # proportional velocity control
            forward_desired = 0.5 * error_x_body
            sideways_desired = 0.5 * error_y_body
            height_diff_desired = 0.5 * error_z

            # check if waypoint is reached
            dist_to_wp = (error_x**2 + error_y**2 + error_z**2) ** 0.5
            if dist_to_wp < wp_tolerance and not rotating:
                print(f"Waypoint {current_wp_index} reached!")

                if current_wp_index in yaw_rotation_points:
                    rotate_counter = 0
                    rotate_limit = yaw_rotation_points[current_wp_index]['cycles']
                    rotate_direction = yaw_rotation_points[current_wp_index]['direction']
                    rotating = True
                    print(f"Starting yaw rotation at waypoint {current_wp_index} for {rotate_limit} cycles")
                else:
                    current_wp_index += 1
                    if current_wp_index == len(waypoints):
                        print("All waypoints completed!")
                        autonomous_mode=False
                        move_validation_split()

        # ================== HANDLE ROTATION ==================
        if rotating:
            if rotate_counter < rotate_limit:
                yaw_desired = rotate_direction
                rotate_counter += 1
            else:
                yaw_desired = 0.0
                rotating = False
                current_wp_index += 1
                print("Yaw rotation complete. Moving to next waypoint.")

        # ================== IMAGE SAVE WITH WIGGLE ==================
        if autonomous_mode and current_wp_index < len(waypoints) and not rotating:
            target_x, target_y, target_z = waypoints[current_wp_index]
            dist_to_next_wp = ((target_x - x_global)**2 +
                               (target_y - y_global)**2 +
                               (target_z - altitude)**2) ** 0.5
        
            # Choose special folder
            if current_wp_index in [1, 3, 10,17]:
                special_folder = LEFT_JUNC_DIR
            elif current_wp_index in [2, 9]:
                special_folder = LEFT_BEND_DIR
            elif current_wp_index in [7, 4,15, 18]:
                special_folder = FOUR_W_DIR
            elif current_wp_index in [5,8,16, 19]:
                special_folder = T_JUNC_DIR
            elif current_wp_index in [6,12,14,21]:
                special_folder = RIGHT_JUNC_DIR
            elif current_wp_index in [13,20]:
                special_folder = RIGHT_BEND_DIR
            elif current_wp_index in [11]:
                special_folder = DEAD_END
            else:
                special_folder = STRAIGHT_DIR   # fallback
        
            # Initialize counters if not exists
            if "straight_counters" not in globals():
                straight_counters = {}
            if "special_counters" not in globals():
                special_counters = {}
        
            if current_wp_index not in straight_counters:
                straight_counters[current_wp_index] = 0
            if current_wp_index not in special_counters:
                special_counters[current_wp_index] = 0
        
        
            # Case 1: BEFORE reaching save_distance_before_wp → save 5 images in STRAIGHT_DIR
            if dist_to_next_wp > save_distance_before_wp and straight_counters[current_wp_index] < 40:
                # Initialize buffer for this waypoint (hold up to 15)
                if current_wp_index not in straight_buffer:
                    straight_buffer[current_wp_index] = []
            
                # Store image + counter in buffer (up to 15 total)
                image = camera.getImage()
                if image:
                    width = camera.getWidth()
                    height = camera.getHeight()
                    img = np.frombuffer(image, np.uint8).reshape((height, width, 4))
                    img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
                    straight_buffer[current_wp_index].append((frame_counter, img_bgr))
                    frame_counter += 1
                    straight_counters[current_wp_index] += 1
        
        
            # Case 2: INSIDE save_distance_before_wp → trigger wiggle
            elif dist_to_next_wp <= save_distance_before_wp and not wiggling and special_counters[current_wp_index] < 10:
                print(f"Starting wiggle at waypoint {current_wp_index}")
                wiggling = True
                wiggle_steps = [0.2, -0.2]  # sideways motions
                wiggle_counter = 0   
            
            
            
            elif dist_to_next_wp <= save_distance_before_wp and not wiggling:
                # Save only the FIRST 5 from the buffered images
                if current_wp_index in straight_buffer and len(straight_buffer[current_wp_index]) > 0:
                    last_five = straight_buffer[current_wp_index][-5:]  # take last 5
                    for cnt, img_bgr in last_five:
                        filename = os.path.join(STRAIGHT_DIR, f"a100_env1_ms_50_{cnt:06d}.jpg")
                        cv2.imwrite(filename, img_bgr)
                    print(f"Saved {len(last_five)} (last 5 of {len(straight_buffer[current_wp_index])}) images for waypoint {current_wp_index}")
                    straight_buffer[current_wp_index].clear()
            # If currently wiggling → apply sideways offset and save images
            # ================== HANDLE WIGGLING ==================
            if wiggling:
                current_time = robot.getTime()
                if wiggle_counter < len(wiggle_steps):
                    if current_time - last_wiggle_time >= wiggle_delay:
                        sideways_desired = wiggle_steps[wiggle_counter]
                        # Save image in SPECIAL folder (not straight)
                        if special_counters[current_wp_index] < 10:  # limit to 5 images
                            save_camera_image(camera, special_folder, frame_counter)
                            frame_counter += 1
                            special_counters[current_wp_index] += 1
            
                        wiggle_counter += 1
                        last_wiggle_time = current_time
                else:
                    wiggling = False
                    print("Wiggle complete. Continuing navigation.")
        # ================== PID CONTROLLER ==================
        motor_power = PID_crazyflie.pid(dt,
                                        forward_desired,
                                        sideways_desired,
                                        yaw_desired,
                                        height_desired,
                                        roll, pitch, yaw_rate,
                                        altitude, v_x, v_y)

        m1_motor.setVelocity(-motor_power[0])
        m2_motor.setVelocity(motor_power[1])
        m3_motor.setVelocity(-motor_power[2])
        m4_motor.setVelocity(motor_power[3])

        past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global