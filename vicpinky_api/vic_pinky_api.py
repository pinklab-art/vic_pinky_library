import math
import time
import threading
import logging

# Set up default logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger("VicPinkyAPI")

# ZLACDriver is in the same directory now
from .zlac_driver import ZLACDriver

class WheelController:
    """
    A high-level controller for differential-drive robots (Vic Pinky).
    Abstracts ZLAC Modbus driver control, kinematics, and velocity profiling.
    Designed for standalone Python usage without ROS 2.
    """
    def __init__(self, port="/dev/motor", baudrate=115200, modbus_id=0x01,
                 wheel_rad=0.0825, wheel_base=0.441,
                 accel_limit=0.4, decel_limit=0.5,
                 ang_accel_limit=1.0, ang_decel_limit=1.5,
                 max_rpm=28.0):
        
        self.driver = ZLACDriver(port, baudrate, modbus_id)
        
        # Hardware Parameters
        self.WHEEL_RAD = wheel_rad
        self.WHEEL_BASE = wheel_base
        self.PULSE_PER_ROT = 4096
        self.RPM2RAD = 0.104719755
        self.CIRCUMFERENCE = 2 * math.pi * self.WHEEL_RAD
        self.MAX_RPM = max_rpm
        
        # Velocity Profiles
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.ang_accel_limit = ang_accel_limit
        self.ang_decel_limit = ang_decel_limit
        
        # Velocity States
        self.current_cmd_x = 0.0
        self.current_cmd_th = 0.0
        
        self.target_v_x = 0.0
        self.target_v_th = 0.0
        self.speed_scale = 1.0  # Global multiplier for all velocity commands
        
        # Encoder States
        self.last_encoder_l = 0
        self.last_encoder_r = 0
        
        # Global Pose (Meters, Radians)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Threading & Control
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._last_step_time = 0
        
        # Safety
        self.is_connected = False
        self._error_count = 0
        self.MAX_ERRORS = 5

        # Raw feedback state
        self.current_rpm_l = 0.0
        self.current_rpm_r = 0.0
        self.current_encoder_l = 0
        self.current_encoder_r = 0

    def connect(self):
        """Initializes connection to ZLAC driver, enables motors and captures initial encoder state."""
        if not self.driver.begin():
            return False
            
        time.sleep(0.1)
        if not self.driver.set_vel_mode():
            self.driver.terminate()
            return False
            
        time.sleep(0.1)
        if not self.driver.enable():
            self.driver.terminate()
            return False
            
        time.sleep(1.0)
        
        # Verify controller check
        rpm_l, rpm_r = self.driver.get_rpm()
        if rpm_l is None or rpm_r is None:
            self.driver.terminate()
            return False
            
        time.sleep(0.1)
        
        # Clear RPM logic
        success = False
        for _ in range(3):
            if self.driver.set_double_rpm(0, 0):
                success = True
                break
            time.sleep(0.2)
            
        if not success:
            self.driver.terminate()
            return False
            
        # Get baseline positions
        self.last_encoder_l, self.last_encoder_r = self.driver.get_position()
        if self.last_encoder_l is None or self.last_encoder_r is None:
            self.driver.terminate()
            return False
            
        self.is_connected = True
        self._error_count = 0
        logger.info(f"Connected to motor driver on {self.driver.port}")
        
        # Start background loop
        self._running = True
        self._last_step_time = time.time()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        
        return True

    def _run_loop(self):
        """Background thread that calls step() at ~20Hz."""
        while self._running:
            now = time.time()
            dt = now - self._last_step_time
            if dt >= 0.05: # 20Hz
                if not self.step(dt):
                    self._error_count += 1
                    if self._error_count >= self.MAX_ERRORS:
                        logger.critical("Communication lost. Attempting emergency stop.")
                        # Best-effort halt before tearing down the loop so the robot
                        # does not keep coasting on its last commanded RPM.
                        try:
                            if self.driver.ser and self.driver.ser.is_open:
                                self.driver.set_double_rpm(0, 0)
                        except Exception:
                            pass
                        self.current_cmd_x = 0.0
                        self.current_cmd_th = 0.0
                        self.is_connected = False
                        self._running = False
                        break
                else:
                    self._error_count = 0
                self._last_step_time = now
            time.sleep(0.01) # Sleep to prevent high CPU usage

    def get_pose(self):
        """Returns the current estimated pose (x, y, theta)."""
        with self._lock:
            return self.x, self.y, self.theta

    def set_odometry(self, x, y, theta):
        """Sets the robot's current pose (odometry) manually."""
        with self._lock:
            self.x = x
            self.y = y
            self.theta = theta
            # Normalize theta
            self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

    def reset_odometry(self):
        """Resets the robot's pose to (0, 0, 0)."""
        self.set_odometry(0.0, 0.0, 0.0)

    def set_target_velocity(self, linear_x, angular_z):
        """Sets the desired linear and angular velocity in m/s and rad/s.

        Targets are stored in real units; ``speed_scale`` is applied only at the
        final motor-output stage in :meth:`step`, so blocking P-controllers
        (turn_to, move_position, ...) operate on true velocities.
        """
        self.target_v_x = linear_x
        self.target_v_th = angular_z
        
    def stop(self):
        """Halt the wheels immediately and zero out target and command velocities.

        Clearing ``current_cmd_*`` prevents the 20 Hz loop from re-accelerating
        right after the direct stop command. The stop is attempted whenever the
        serial port is open, even if ``is_connected`` has been cleared (e.g. after
        a communication-loss event), so the robot can always be commanded to halt.
        """
        self.target_v_x = 0.0
        self.target_v_th = 0.0
        self.current_cmd_x = 0.0
        self.current_cmd_th = 0.0
        if self.driver.ser and self.driver.ser.is_open:
            self.driver.set_double_rpm(0, 0)
        
    def step(self, dt):
        """
        Applies acceleration limits, updates odometry via kinematic calculations, 
        and sends target RPM commands to the motors.
        Must be called periodically.

        :param dt: Time elapsed (in seconds) since last step call.
        :return: Dict containing delta movement and velocities, or None if error.
        """
        if not self.is_connected or dt <= 0:
            return None
            
        # 1. Odometry Calculation (Read Sensors)
        rpm_l_fb, rpm_r_fb = self.driver.get_rpm()
        encoder_l, encoder_r = self.driver.get_position()

        if rpm_l_fb is None or rpm_r_fb is None or encoder_l is None or encoder_r is None:
            return None

        # Update raw state
        self.current_rpm_l = rpm_l_fb
        self.current_rpm_r = rpm_r_fb
        self.current_encoder_l = encoder_l
        self.current_encoder_r = encoder_r

        # Calculate encoder differentials
        delta_l = encoder_l - self.last_encoder_l
        delta_r = encoder_r - self.last_encoder_r
        
        self.last_encoder_l = encoder_l
        self.last_encoder_r = encoder_r

        # Distance conversions
        dist_l = (delta_l / self.PULSE_PER_ROT) * self.CIRCUMFERENCE
        dist_r = (delta_r / self.PULSE_PER_ROT) * self.CIRCUMFERENCE

        # Forward kinematics for movement deltas
        delta_dist = (dist_r + dist_l) / 2.0
        delta_th = (dist_r - dist_l) / self.WHEEL_BASE
        
        # Update Global Pose
        with self._lock:
            # Simple differential drive odometry
            self.x += delta_dist * math.cos(self.theta + delta_th / 2.0)
            self.y += delta_dist * math.sin(self.theta + delta_th / 2.0)
            self.theta += delta_th
            # Normalize theta to [-pi, pi]
            self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        
        # Real instantaneous velocity estimated by encoders
        v_x_real = delta_dist / dt
        v_th_real = delta_th / dt
        
        # 2. Velocity profile ramp.
        # Ramp from the PREVIOUS COMMAND (not the measured velocity), so encoder
        # noise / momentary stalls don't collapse the command, and the profile
        # accelerates deterministically.
        prev_cmd_x = self.current_cmd_x
        if abs(self.target_v_x) < abs(prev_cmd_x):
            step_lin = self.decel_limit * dt
        elif (self.target_v_x * prev_cmd_x) < 0: # Changing direction
            step_lin = self.decel_limit * dt
        else:
            step_lin = self.accel_limit * dt

        if prev_cmd_x < self.target_v_x:
            self.current_cmd_x = min(self.target_v_x, prev_cmd_x + step_lin)
        elif prev_cmd_x > self.target_v_x:
            self.current_cmd_x = max(self.target_v_x, prev_cmd_x - step_lin)
        else:
            self.current_cmd_x = self.target_v_x

        prev_cmd_th = self.current_cmd_th
        if abs(self.target_v_th) < abs(prev_cmd_th):
            step_ang = self.ang_decel_limit * dt
        elif (self.target_v_th * prev_cmd_th) < 0:
            step_ang = self.ang_decel_limit * dt
        else:
            step_ang = self.ang_accel_limit * dt

        if prev_cmd_th < self.target_v_th:
            self.current_cmd_th = min(self.target_v_th, prev_cmd_th + step_ang)
        elif prev_cmd_th > self.target_v_th:
            self.current_cmd_th = max(self.target_v_th, prev_cmd_th - step_ang)
        else:
            self.current_cmd_th = self.target_v_th

        # 3. Inverse Kinematics (Twist to double RPM).
        # speed_scale is applied here as a global output throttle, leaving the
        # commanded/target velocities in real units for the controllers above.
        cmd_x = self.current_cmd_x * self.speed_scale
        cmd_th = self.current_cmd_th * self.speed_scale

        v_l = cmd_x - (cmd_th * self.WHEEL_BASE / 2.0)
        v_r = cmd_x + (cmd_th * self.WHEEL_BASE / 2.0)

        rpm_l_raw = v_l / (self.WHEEL_RAD * self.RPM2RAD)
        rpm_r_raw = v_r / (self.WHEEL_RAD * self.RPM2RAD)

        # Bound requested RPM, scaling both wheels equally to preserve curvature
        max_req = max(abs(rpm_l_raw), abs(rpm_r_raw))
        if max_req > self.MAX_RPM:
            scale = self.MAX_RPM / max_req
            rpm_l_raw *= scale
            rpm_r_raw *= scale

        # Final safety-net clip, then quantize to integer RPM
        rpm_l = int(max(min(rpm_l_raw, self.MAX_RPM), -self.MAX_RPM))
        rpm_r = int(max(min(rpm_r_raw, self.MAX_RPM), -self.MAX_RPM))

        # A failed write must be reported so the loop's error counter can trip
        # the communication-loss safeguard (do not swallow it).
        if not self.driver.set_double_rpm(rpm_l, rpm_r):
            return None

        return {
            'delta_dist': delta_dist,
            'delta_th': delta_th,
            'vel_x_real': v_x_real,
            'vel_th_real': v_th_real,
            'rpm_l_fb': rpm_l_fb,
            'rpm_r_fb': rpm_r_fb,
            'encoder_l': self.last_encoder_l,
            'encoder_r': self.last_encoder_r
        }

    # --- High Level Intuitive API ---
    
    def move(self, linear, angular=0.0):
        """Simple non-blocking command to set velocities."""
        self.set_target_velocity(linear, angular)

    def turn_to(self, target_theta, speed=0.5, tolerance=0.05):
        """Blocking command to rotate to a specific absolute angle (radians)."""
        logger.info(f"Turning to {target_theta:.2f} rad...")
        while self._running:
            curr_x, curr_y, curr_th = self.get_pose()
            
            # Shortest angular distance
            diff = target_theta - curr_th
            diff = (diff + math.pi) % (2 * math.pi) - math.pi
            
            if abs(diff) < tolerance:
                break
                
            # Simple P-control for angular velocity
            w = 2.0 * diff
            # Clip to speed
            w = max(min(w, speed), -speed)
            
            self.set_target_velocity(0.0, w)
            time.sleep(0.05)
            
        self.stop()
        logger.info("Turn complete.")

    def turn_relative(self, angle, speed=0.5, tolerance=0.05):
        """Blocking command to rotate by a specific relative angle (radians)."""
        curr_x, curr_y, curr_th = self.get_pose()
        target_theta = curr_th + angle
        # Normalize to [-pi, pi]
        target_theta = (target_theta + math.pi) % (2 * math.pi) - math.pi
        self.turn_to(target_theta, speed, tolerance)

    def move_position(self, target_x, target_y, speed=0.2, tolerance=0.05):
        """
        Blocking command to move the robot to a specific absolute (x, y) coordinate.
        First rotates toward the point, then moves forward.
        """
        logger.info(f"Moving to ({target_x:.2f}, {target_y:.2f})...")
        
        # 1. Turn towards the target
        curr_x, curr_y, curr_th = self.get_pose()
        dx = target_x - curr_x
        dy = target_y - curr_y
        target_angle = math.atan2(dy, dx)
        
        self.turn_to(target_angle, speed=0.8, tolerance=0.1) # Faster turn
        
        # 2. Move forward
        while self._running:
            curr_x, curr_y, curr_th = self.get_pose()
            dx = target_x - curr_x
            dy = target_y - curr_y
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist < tolerance:
                break
            
            # Angle correction while moving
            target_angle = math.atan2(dy, dx)
            angle_diff = (target_angle - curr_th + math.pi) % (2 * math.pi) - math.pi
            
            v = speed
            w = 1.0 * angle_diff # Simple orientation correction
            
            self.set_target_velocity(v, w)
            time.sleep(0.05)
            
        self.stop()
        logger.info("Position reached.")

    def smooth_move_to(self, target_x, target_y, speed=0.2, tolerance=0.05):
        """
        [SMOOTHING] Blocking command to move to (x, y) in a smooth arc.
        Does not stop to turn; instead adjusts heading continuously while moving.
        """
        logger.info(f"Smoothly moving to ({target_x:.2f}, {target_y:.2f})...")
        
        while self._running:
            curr_x, curr_y, curr_th = self.get_pose()
            dx = target_x - curr_x
            dy = target_y - curr_y
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist < tolerance:
                break
                
            # Target heading to point
            target_angle = math.atan2(dy, dx)
            angle_diff = (target_angle - curr_th + math.pi) % (2 * math.pi) - math.pi
            
            # Smoothly reduce linear speed if the angle error is high
            v = speed * max(0.0, math.cos(angle_diff))
            # Angular velocity based on heading error
            w = 2.0 * angle_diff
            
            self.set_target_velocity(v, w)
            time.sleep(0.05)
            
        self.stop()
        logger.info("Smooth move reached.")

    def move_distance(self, distance, speed=0.2, tolerance=0.05):
        """Blocking command to move a specific relative distance (meters)."""
        curr_x, curr_y, curr_th = self.get_pose()
        target_x = curr_x + distance * math.cos(curr_th)
        target_y = curr_y + distance * math.sin(curr_th)
        self.move_position(target_x, target_y, speed, tolerance)

    def is_moving(self, threshold=0.01):
        """Returns True if the robot is currently commanded to move or is moving."""
        return abs(self.target_v_x) > threshold or abs(self.target_v_th) > threshold or \
               abs(self.current_cmd_x) > threshold or abs(self.current_cmd_th) > threshold

    def get_encoders(self):
        """Returns the latest read encoder pulse counts (left, right)."""
        return self.current_encoder_l, self.current_encoder_r

    def get_rpms(self):
        """Returns the latest read RPM values (left, right)."""
        return self.current_rpm_l, self.current_rpm_r

    def wait_until_stopped(self, timeout=10.0):
        """Blocking call that waits until is_moving() returns False or timeout reached."""
        start_time = time.time()
        while self.is_moving() and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        return not self.is_moving()

    def get_status(self):
        """Returns a comprehensive status dictionary of the robot."""
        x, y, th = self.get_pose()
        return {
            'is_connected': self.is_connected,
            'is_moving': self.is_moving(),
            'pose': {'x': x, 'y': y, 'theta': th},
            'target_velocity': {'linear': self.target_v_x, 'angular': self.target_v_th},
            'current_command': {'linear': self.current_cmd_x, 'angular': self.current_cmd_th},
            'raw_data': {
                'encoder_l': self.current_encoder_l,
                'encoder_r': self.current_encoder_r,
                'rpm_l': self.current_rpm_l,
                'rpm_r': self.current_rpm_r
            },
            'error_count': self._error_count,
            'speed_scale': self.speed_scale
        }

    def disconnect(self):
        """Terminates and closes motor interface safely."""
        # Zero the targets/commands first so the loop's last action is a halt.
        self.target_v_x = 0.0
        self.target_v_th = 0.0
        self.current_cmd_x = 0.0
        self.current_cmd_th = 0.0

        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

        if self.is_connected:
            self.driver.terminate()
            self.is_connected = False
        logger.info("Disconnected.")
