#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.qos import qos_profile_sensor_data
import math


class FixedWingLandingMission(Node):

    def __init__(self):
        super().__init__('fixed_wing_landing_node')

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_cb,
            qos_profile_sensor_data
        )

        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.current_state = State()
        self.current_pose = PoseStamped()

        self.is_takeoff_complete = False
        self.current_wp_index = 0

        self.waypoints = [
            {"name": "1. Sola Acilma (Crosswind)", "x": -300.0, "y": 0.0, "z": 50.0},
            {"name": "2. Geriye Cekilme (Downwind)", "x": -300.0, "y": -800.0, "z": 50.0},
            {"name": "3. Piste Hizalanma (Base Leg)", "x": 0.0, "y": -800.0, "z": 50.0},
            {"name": "4. Son Yaklasma Girisi", "x": 0.0, "y": -600.0, "z": 50.0},

            {"name": "5. Alcalma Baslangici", "x": 0.0, "y": -400.0, "z": 30.0},
            {"name": "6. Son Yaklasma", "x": 0.0, "y": -200.0, "z": 15.0},
            {"name": "7. Pist Basi (Erken Yaklasma)", "x": 0.0, "y": -100.0, "z": 5.0},
            {"name": "8. Agresif Flare", "x": 0.0, "y": -50.0, "z": 1.0},
            {"name": "9. Teker Koyma ve Roll-out", "x": 0.0, "y": 100.0, "z": -1.0}
        ]

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("✈️ İniş Senaryosu Başlatıldı. Bağlantı bekleniyor...")


    def state_cb(self, msg):
        self.current_state = msg


    def pose_cb(self, msg):
        self.current_pose = msg


    def calculate_distance(self, wp):
        dx = self.current_pose.pose.position.x - wp['x']
        dy = self.current_pose.pose.position.y - wp['y']
        dz = self.current_pose.pose.position.z - wp['z']
        return math.sqrt(dx**2 + dy**2 + dz**2)


    def control_loop(self):

        if not self.current_state.connected:
            return

        # TAKEOFF
        if not self.is_takeoff_complete:

            if self.current_state.mode != "AUTO.TAKEOFF":
                self.set_mode("AUTO.TAKEOFF")

            elif not self.current_state.armed:
                self.set_arm(True)

            else:
                if self.current_pose.pose.position.z > 45.0:
                    self.get_logger().info("✅ Kalkış tamamlandı. İniş başlıyor!")
                    self.is_takeoff_complete = True

            return

        # WAYPOINT CONTROL
        current_target_index = min(self.current_wp_index, len(self.waypoints) - 1)
        target_wp = self.waypoints[current_target_index]

        pose = PoseStamped()
        pose.pose.position.x = target_wp['x']
        pose.pose.position.y = target_wp['y']
        pose.pose.position.z = target_wp['z']

        self.local_pos_pub.publish(pose)

        curr_y = self.current_pose.pose.position.y
        curr_z = self.current_pose.pose.position.z

        # FINAL LANDING LOGIC
        if current_target_index == len(self.waypoints) - 1:

            if curr_y > -15.0 and curr_z < 1.5:
                if self.current_state.armed:
                    self.get_logger().info(
                        f"🛑 FRENLEME AKTİF! Y:{curr_y:.1f} Z:{curr_z:.1f} - MOTORLAR KESİLİYOR!"
                    )

                    if self.current_state.mode != "AUTO.LAND":
                        self.set_mode("AUTO.LAND")

                    self.set_arm(False)

        else:
            if self.current_state.mode != "OFFBOARD":
                self.set_mode("OFFBOARD")

        # DISTANCE CHECK
        dist = self.calculate_distance(target_wp)

        passed_target_y = False

        # Sadece flare öncesi waypoint'te aktif
        if current_target_index == len(self.waypoints) - 2:
            if curr_y > target_wp['y']:
                passed_target_y = True

        # ACCEPTANCE RADIUS
        if target_wp['z'] > 10.0:
            accept_radius = 15.0
        else:
            accept_radius = 25.0

        if dist < accept_radius or passed_target_y:
            self.get_logger().info(f"🎯 Hedef Gecildi/Ulasildi: {target_wp['name']}")
            self.current_wp_index += 1


    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.mode_client.call_async(req)


    def set_arm(self, state):
        req = CommandBool.Request()
        req.value = state
        self.arm_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)

    node = FixedWingLandingMission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("❌ Senaryo durduruldu.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()