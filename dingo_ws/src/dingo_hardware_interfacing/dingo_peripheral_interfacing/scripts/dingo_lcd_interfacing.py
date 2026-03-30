#!/usr/bin/env python3
import logging
import os
import socket
import time

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from PIL import Image, ImageDraw, ImageFont

import LCD_1inch47
from dingo_peripheral_msgs.msg import ElectricalMeasurements


class DingoDisplayNode:
    def __init__(self, node: Node):
        self._node = node
        self._rate = node.create_rate(50)
        self.RST = 27
        self.DC = 25
        self.BL = 18
        self.bus = 0
        self.device = 0
        self.ssid = ''
        self.ipAddress = ''
        self.disp = LCD_1inch47.LCD_1inch47()
        self.disp.Init()
        self.disp.clear()

        self._sub = node.create_subscription(
            ElectricalMeasurements,
            '/electrical_measurements',
            self.update_battery_percentage,
            10,
        )

        self.battery_percentage = 0.7
        self._share = get_package_share_directory('dingo_peripheral_interfacing')
        self._batt_image_path = os.path.join(self._share, 'lib', 'emptybatterystatus_white.png')

    def update_battery_percentage(self, message):
        max_voltage = 16.8
        min_voltage = 14.0

        battery_voltage_level = max(min(message.battery_voltage_level, max_voltage), min_voltage)

        self.battery_percentage = (battery_voltage_level - min_voltage) / (max_voltage - min_voltage)

    def run(self):
        try:
            image1 = Image.new('RGB', (self.disp.height, self.disp.width), 'black')
            draw = ImageDraw.Draw(image1)

            Font1 = ImageFont.truetype('/usr/share/fonts/truetype/Font02.ttf', 25)
            Font1_small = ImageFont.truetype('/usr/share/fonts/truetype/Font02.ttf', 20)
            Font1_large = ImageFont.truetype('/usr/share/fonts/truetype/Font02.ttf', 60)
            Font2 = ImageFont.truetype('/usr/share/fonts/truetype/Font01.ttf', 35)
            Font3 = ImageFont.truetype('/usr/share/fonts/truetype/Font02.ttf', 120)

            draw.text((20, 110), 'SSID: ' + self.ssid, fill='WHITE', font=Font1)
            draw.text((20, 135), 'IP: ' + self.ipAddress, fill='WHITE', font=Font1)
            current_time = time.strftime('%I:%M:%S%p')
            draw.text((220, 0), current_time, fill='WHITE', font=Font1_small)

            if os.path.isfile(self._batt_image_path):
                batt_status = Image.open(self._batt_image_path)
            else:
                batt_status = Image.new('RGB', (320, 172), 'black')
                self._node.get_logger().warning(
                    'Battery overlay image missing at %s; using blank' % self._batt_image_path
                )

            batt_draw = ImageDraw.Draw(batt_status)

            if self.battery_percentage <= 0.20:
                batt_fill = 'RED'
            elif 0.20 < self.battery_percentage <= 0.60:
                batt_fill = '#d49b00'
            else:
                batt_fill = '#09ab00'

            batt_draw.rounded_rectangle([(42, 92), (42 + (153 * self.battery_percentage), 170)], 8, fill=batt_fill)
            batt_draw.text((68, 95), str(int(self.battery_percentage * 100)) + '%', fill='WHITE', font=Font1_large)
            batt_scale_factor = 0.8
            resized_batt_status = batt_status.resize(
                (int(batt_status.size[0] * batt_scale_factor), int(batt_status.size[1] * batt_scale_factor))
            )
            image1.paste(resized_batt_status, (62, -40), resized_batt_status.convert('RGBA'))

            image1 = image1.rotate(0)
            image1 = image1.transpose(Image.ROTATE_270)
            self.disp.ShowImage(image1)

            try:
                self.ssid = os.popen('iwgetid -r').read().strip()
            except Exception as e:
                self._node.get_logger().error(str(e))
                self.ssid = 'N/A'

            try:
                hostname = socket.gethostname()
                self.ipAddress = socket.gethostbyname(hostname)
            except Exception as e:
                self._node.get_logger().error(str(e))
                self.ipAddress = '-:-:-:-'

        except Exception as e:
            self._node.get_logger().error(str(e))

    def loop(self):
        try:
            while rclpy.ok():
                self.run()
                self._rate.sleep()
                rclpy.spin_once(self._node, timeout_sec=0.0)
        finally:
            self._node.get_logger().info('Quitting...')
            self.disp.clear()
            self.disp.module_exit()


def main():
    rclpy.init()
    node = rclpy.create_node('dingo_display_node')
    display = DingoDisplayNode(node)
    node.get_logger().info('Display node started, outputting to display')
    display.loop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
