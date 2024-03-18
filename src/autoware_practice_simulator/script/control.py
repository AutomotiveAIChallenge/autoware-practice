# Copyright 2024 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import curses
import math
import threading

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import GearCommand
from autoware_auto_vehicle_msgs.msg import GearReport
from autoware_auto_vehicle_msgs.msg import SteeringReport
from autoware_auto_vehicle_msgs.msg import VelocityReport
import rclpy

gear_msgs = {"P": GearCommand.PARK, "N": GearCommand.NEUTRAL, "D": GearCommand.DRIVE, "R": GearCommand.REVERSE}
gear_strs = {GearReport.PARK: "P", GearReport.NEUTRAL: "N", GearReport.DRIVE: "D", GearReport.REVERSE: "R"}


def addstr_and_clrtoeol(stdscr, y, x, text):
    stdscr.addstr(y, x, text)
    stdscr.clrtoeol()


class VehicleControl:
    def __init__(self):
        self.thread = None
        self.command_accel = 0.0
        self.current_accel = 0.0
        self.command_speed = 0.0
        self.current_speed = 0.0
        self.command_steer = 0.0
        self.current_steer = 0.0
        self.command_gear = "D"
        self.current_gear = "D"

    def ros_init(self):
        rclpy.init()
        self.node = rclpy.create_node("test")
        self.pub_command = self.node.create_publisher(AckermannControlCommand, "/simulator/command/control", 1)
        self.pub_gear = self.node.create_publisher(GearCommand, "/simulator/command/gear", 1)
        self.sub_velocity = self.node.create_subscription(VelocityReport, "/simulator/status/velocity", self.on_velocity, 1)
        self.sub_steering = self.node.create_subscription(SteeringReport, "/simulator/status/steering", self.on_steering, 1)
        self.sub_gear = self.node.create_subscription(GearReport, "/simulator/status/gear", self.on_gear, 1)
        self.timer = self.node.create_timer(0.1, self.on_timer)
        self.thread = threading.Thread(target=self.ros_spin, args=[self.node])
        self.thread.start()

    def ros_spin(self, node):
        try:
            rclpy.spin(node)
        except rclpy.executors.ExternalShutdownException:
            pass

    def ros_join(self):
        rclpy.try_shutdown()
        self.thread.join()

    def on_velocity(self, msg):
        self.current_speed = msg.longitudinal_velocity

    def on_steering(self, msg):
        self.current_steer = math.degrees(msg.steering_tire_angle)

    def on_gear(self, msg):
        self.current_gear = gear_strs.get(msg.report)

    def on_timer(self):
        sign = -1.0 if self.command_gear == "R" else +1.0
        self.command_accel = math.copysign(5.0, self.command_speed - self.current_speed) * sign
        command = AckermannControlCommand()
        command.stamp = self.node.get_clock().now().to_msg()
        command.longitudinal.speed = self.command_speed
        command.longitudinal.acceleration = self.command_accel
        command.lateral.steering_tire_angle = math.radians(self.command_steer)
        self.pub_command.publish(command)

        gear = GearCommand()
        gear.stamp = self.node.get_clock().now().to_msg()
        gear.command = gear_msgs.get(self.command_gear)
        self.pub_gear.publish(gear)

    def main(self, stdscr: curses.window):
        stdscr.timeout(100)
        stdscr.keypad(True)
        stdscr.addstr(0, 0, "Command Accel:")
        stdscr.addstr(1, 0, "Current Accel:")
        stdscr.addstr(2, 0, "Command Speed:")
        stdscr.addstr(3, 0, "Current Speed:")
        stdscr.addstr(4, 0, "Command Steer:")
        stdscr.addstr(5, 0, "Current Steer:")
        stdscr.addstr(6, 0, "Command Gear :")
        stdscr.addstr(7, 0, "Current Gear :")
        while True:
            self.keyinput(stdscr.getch())
            self.update()
            addstr_and_clrtoeol(stdscr, 0, 15, f"{self.command_accel:.2f}")
            addstr_and_clrtoeol(stdscr, 1, 15, "----")
            addstr_and_clrtoeol(stdscr, 2, 15, f"{self.command_speed:.2f}")
            addstr_and_clrtoeol(stdscr, 3, 15, f"{self.current_speed:.2f}")
            addstr_and_clrtoeol(stdscr, 4, 15, f"{self.command_steer:.2f}")
            addstr_and_clrtoeol(stdscr, 5, 15, f"{self.current_steer:.2f}")
            addstr_and_clrtoeol(stdscr, 6, 15, self.command_gear)
            addstr_and_clrtoeol(stdscr, 7, 15, self.current_gear)

    def keyinput(self, key):
        if key == curses.KEY_UP:
            self.command_speed = self.command_speed + 1.0
        if key == curses.KEY_DOWN:
            self.command_speed = self.command_speed - 1.0
        if key == curses.KEY_LEFT:
            self.command_steer = self.command_steer + 1.0
        if key == curses.KEY_RIGHT:
            self.command_steer = self.command_steer - 1.0
        if key == ord("z"):
            self.command_speed = 0.0
        if key == ord("x"):
            self.command_steer = 0.0
        if key == ord("p"):
            self.command_gear = "P"
        if key == ord("n"):
            self.command_gear = "N"
        if key == ord("d"):
            self.command_gear = "D"
        if key == ord("r"):
            self.command_gear = "R"

    def update(self):
        if self.command_gear == "D":
            self.command_speed = max(0.0, self.command_speed)
        if self.command_gear == "R":
            self.command_speed = min(0.0, self.command_speed)


if __name__ == "__main__":
    control = VehicleControl()
    control.ros_init()
    try:
        curses.wrapper(control.main)
    except KeyboardInterrupt:
        pass
    control.ros_join()
