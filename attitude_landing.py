#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.qos import qos_profile_sensor_data

class FixedWingAttitudeLanding(Node):
    def __init__(self):
        super().__init__('fixed_wing_landing_node')

        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos_profile_sensor_data)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.attitude_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.current_state = State()
        self.current_pose = PoseStamped()
        self.is_takeoff_complete = False
        self.landing_phase_active = False
        self.current_wp_index = 0

        self.alignment_waypoints = [
            {"name": "1. Sola Acilma", "x": -300.0, "y": 0.0, "z": 50.0},
            {"name": "2. Geriye Cekilme", "x": -300.0, "y": -800.0, "z": 50.0},
            {"name": "3. Piste Hizalanma", "x": 0.0, "y": -800.0, "z": 50.0},
            {"name": "4. Son Yaklasma Girisi", "x": 0.0, "y": -600.0, "z": 50.0}
        ]

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("✈️ Hassas Kalibrasyonlu Attitude İniş Başlatıldı...")

    def state_cb(self, msg): self.current_state = msg
    def pose_cb(self, msg): self.current_pose = msg

    def calculate_distance(self, wp):
        dx = self.current_pose.pose.position.x - wp['x']
        dy = self.current_pose.pose.position.y - wp['y']
        dz = self.current_pose.pose.position.z - wp['z']
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.mode_client.call_async(req)

    def set_arm(self, state):
        req = CommandBool.Request()
        req.value = state
        self.arm_client.call_async(req)

    def control_loop(self):
        if not self.current_state.connected: return

        # 1. KALKIŞ
        if not self.is_takeoff_complete:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.alignment_waypoints[0]['x'], self.alignment_waypoints[0]['y'], self.alignment_waypoints[0]['z']
            self.local_pos_pub.publish(pose)

            if self.current_state.mode != "AUTO.TAKEOFF": self.set_mode("AUTO.TAKEOFF")
            elif not self.current_state.armed: self.set_arm(True)
            elif self.current_pose.pose.position.z > 40.0:
                self.get_logger().info("✅ Kalkış tamam.")
                self.is_takeoff_complete = True
            return

        # 2. HİZALANMA
        if self.current_wp_index < len(self.alignment_waypoints):
            if self.current_state.mode != "OFFBOARD": self.set_mode("OFFBOARD")
            target_wp = self.alignment_waypoints[self.current_wp_index]
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = target_wp['x'], target_wp['y'], target_wp['z']
            q = self.euler_to_quaternion(0.0, 0.0, math.pi / 2.0)
            pose.pose.orientation = Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))
            self.local_pos_pub.publish(pose)

            dist = self.calculate_distance(target_wp)
            if dist < 25.0:
                self.get_logger().info(f"🎯 Hedef Gecildi: {target_wp['name']}")
                self.current_wp_index += 1
            return

        # 3. ATTITUDE İLE İNİŞ (MESAFE AYARLI)
        curr_z = self.current_pose.pose.position.z
        att_msg = AttitudeTarget()
        att_msg.header.stamp = self.get_clock().now().to_msg()
        att_msg.type_mask = 7 
        target_yaw, target_roll = math.pi / 2.0, 0.0

        if curr_z > 8.0:
            # AŞAMA 1: Yaklaşma / Süzülme (Glide)
            target_pitch = math.radians(-4.0) 
            target_thrust = 0.12 

        elif 1.2 < curr_z <= 8.0:
            # AŞAMA 2: Alçalma Devamı (Descent) 
            # Flare öncesi stabil tutuş bölgesi
            target_pitch = math.radians(4.0) # Daha sığ bir açıyla yaklaşma
            target_thrust = 0.02


        else:
            # AŞAMA 4: TOUCHDOWN (Teker Koyma)
            target_pitch, target_thrust = 0.0, 0.0
            if self.current_state.armed:
               self.get_logger().info("🛑 PİSTTESİN! Motorlar kapanıyor.")
               self.set_arm(False) 

        q_att = self.euler_to_quaternion(target_roll, target_pitch, target_yaw)
        att_msg.orientation = Quaternion(x=float(q_att[0]), y=float(q_att[1]), z=float(q_att[2]), w=float(q_att[3]))
        att_msg.body_rate = Vector3(x=0.0, y=0.0, z=0.0)
        att_msg.thrust = float(target_thrust)
        self.attitude_pub.publish(att_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FixedWingAttitudeLanding()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()