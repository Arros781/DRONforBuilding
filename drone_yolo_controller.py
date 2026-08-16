from controller import Robot
import sys
import math
import base64
import requests

try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found.")

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

class AIDrone(Robot):
    K_VERTICAL_THRUST = 68.5
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0
    K_ROLL_P = 50.0
    K_PITCH_P = 30.0
    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = -1
    target_precision = 0.5 

    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)

        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        self.camera_pitch_motor = self.getDevice("camera pitch")
        self.camera_pitch_motor.setPosition(0.7)

        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1.0)

        self.current_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.target_position = [0.0, 0.0, 0.0]
        self.target_index = 0
        self.target_altitude = 0.0
        
        self.ai_target = None
        self.ai_scan_timer = 0
        self.ai_enabled = True 
        self.YOLO_URL = "http://127.0.0.1:5000/detect"
        self.patrol_waypoints = [[0, 3], [3, 3], [3, -3], [-3, -3], [-3, 3], [0, 0]] 

    def set_position(self, pos):
        self.current_pose = pos

    def send_to_yolo(self):
        if not self.ai_enabled: return None
        try:
            img = self.camera.getImage()
            img_base64 = base64.b64encode(img).decode('utf-8')
            payload = {"image": img_base64}
            response = requests.post(self.YOLO_URL, json=payload, timeout=1.0)
            if response.status_code == 200:
                detections = response.json().get("detections", [])
                if detections:
                    det = detections[0]
                    return {"class": det['class']}
        except:
            pass
        return None

    def move_to_target(self, target_x, target_y, verbose=False):
        dist = math.sqrt((target_x - self.current_pose[0])**2 + (target_y - self.current_pose[1])**2)
        if dist < self.target_precision:
            return 0.0, 0.0, True

        angle_to_target = math.atan2(target_y - self.current_pose[1], target_x - self.current_pose[0])
        angle_left = angle_to_target - self.current_pose[5]
        angle_left = (angle_left + math.pi) % (2 * math.pi) - math.pi

        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * math.pi)
        pitch_disturbance = clamp(math.log10(abs(angle_left) + 0.1), self.MAX_PITCH_DISTURBANCE, 0.1)
        return yaw_disturbance, pitch_disturbance, False

    def run(self):
        t1 = self.getTime()
        self.target_altitude = 4.0
        self.target_position = self.patrol_waypoints[0]
        print("🚀 AI Дрон запущен. Взлетаю...")

        while self.step(self.time_step) != -1:
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acc, pitch_acc, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            roll_disturbance = 0.0
            pitch_disturbance = 0.0
            yaw_disturbance = 0.0
            mission_complete = False

            # --- AI СКАНИРОВАНИЕ ---
            detected_object = None
            if self.ai_enabled and altitude > 1.0:
                self.ai_scan_timer += self.time_step
                if self.ai_scan_timer > 2000:
                    self.ai_scan_timer = 0
                    detected_object = self.send_to_yolo()
                    if detected_object:
                        print(f"🔍 YOLO обнаружил: {detected_object['class']}! Лечу к нему!")
                        self.target_position = [x_pos, y_pos + 2.0, 0.0] # Летим вперед
                        t1 = self.getTime() # Сбрасываем таймер навигации

            # --- НАВИГАЦИЯ ---
            if altitude > 1.0:
                if self.getTime() - t1 > 0.1:
                    # Если YOLO что-то нашел, летим к нему. Иначе по патрульному кругу.
                    if detected_object:
                        yaw_d, pitch_d, done = self.move_to_target(self.target_position[0], self.target_position[1])
                    else:
                        # Периодически обновляем следующую точку патруля
                        dist_to_waypoint = math.sqrt((self.patrol_waypoints[self.target_index][0] - x_pos)**2 + 
                                                     (self.patrol_waypoints[self.target_index][1] - y_pos)**2)
                        if dist_to_waypoint < self.target_precision:
                            self.target_index += 1
                            if self.target_index >= len(self.patrol_waypoints):
                                self.target_index = 0
                            self.target_position = self.patrol_waypoints[self.target_index]
                        
                        yaw_d, pitch_d, done = self.move_to_target(self.target_position[0], self.target_position[1])
                    
                    yaw_disturbance, pitch_disturbance = yaw_d, pitch_d
                    t1 = self.getTime()

            # --- ПИД ---
            roll_input = self.K_ROLL_P * clamp(roll, -1.0, 1.0) + roll_acc + roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1.0, 1.0) + pitch_acc + pitch_disturbance
            yaw_input = yaw_disturbance
            clamped_diff_alt = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1.0, 1.0)
            vertical_input = self.K_VERTICAL_P * pow(clamped_diff_alt, 3.0)

            fl = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            fr = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rl = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rr = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.front_left_motor.setVelocity(fl)
            self.front_right_motor.setVelocity(-fr)
            self.rear_left_motor.setVelocity(-rl)
            self.rear_right_motor.setVelocity(rr)

robot = AIDrone()
robot.run()