#!/usr/bin/env python3

import sys
import serial
import pynmea2
import threading
import time
import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class GPSNMEANode(Node):

    def __init__(self):
        super().__init__('gps_nmea_node')
        # Declare parameters
        self.declare_parameter('port', '/dev/gps_nmea')
        self.declare_parameter('baudrate', 115200)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # Set up serial communication
        self.uart = serial.Serial(port=self.port, baudrate=self.baudrate)
        self.streamreader = pynmea2.NMEAStreamReader(self.uart)

        # ROS2 Publisher
        self.publisher_ = self.create_publisher(String, '/gps/dict', 10)
        
        self.prev_heading = 0.0
        self.clear_position()
        self.update_time = time.time()
        self.stopped = False
        
        # Start the thread
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        # Run the update loop
        while rclpy.ok():
            update = self.get_update()
            if update:
                self.get_logger().info(f'Update: {update}')
                msg = String()
                msg.data = json.dumps(update)
                self.publisher_.publish(msg)
            time.sleep(0.1)  # Add a small delay to prevent high CPU usage

    def stop(self):
        self.stopped = True
        if self.thread.is_alive():
            self.thread.join()

    def clear_position(self):
        self.timestamp = None
        self.latitude = None
        self.longitude = None
        self.heading = None
        self.mode_indicator = None
        self.correction_age = None
        self.course_over_ground = None
        self.speed_over_ground = None
        self.accel_north = None
        self.accel_east = None

    def clear_position_if_new_timestamp(self, timestamp):
        if not timestamp == self.timestamp:
            self.clear_position()
            self.timestamp = timestamp

    def nvg_timestamp(self, s):
        return datetime.time(
            hour=int(s[0:2]),
            minute=int(s[2:4]),
            second=int(s[4:6]),
            microsecond=int(s[7:]) * 10000,
        )

    def get_update(self):
        self.stopped = False
        while not self.stopped:
            message = self.get_message()
            if message:
                self.get_logger().info(f'-> {message}')
                message_type = type(message)
                
                if message_type == pynmea2.types.talker.RMC:
                    self.handle_rmc(message)
                elif message_type == pynmea2.nmea.ProprietarySentence:
                    self.handle_proprietary_sentence(message)
                elif message_type == pynmea2.types.talker.VTG:
                    self.handle_vtg(message)
                elif message_type == pynmea2.types.talker.HDT:
                    self.handle_hdt(message)

                if (self.latitude is not None) and (self.longitude is not None) and (self.heading is not None) and (self.correction_age is not None):
                    now = time.time()
                    period = now - self.update_time
                    if period > 1.0:
                        raise Exception(f'period: {period:.2f}')

                    update = {
                        'latitude': self.latitude,
                        'longitude': self.longitude,
                        'heading': (self.heading + 180) % 360,
                        'course_over_ground': self.course_over_ground,
                        'speed_over_ground': self.speed_over_ground,
                        'mode_indicator': self.mode_indicator,
                        'timestamp': str(self.timestamp),
                        'period': period,
                        'correction_age': self.correction_age,
                    }
                    self.update_time = now
                    self.clear_position()
                    return update

    def handle_rmc(self, message):
        if not message.is_valid:
            self.get_logger().warn('Discard invalid message')
            return
        timestamp = message.timestamp
        self.clear_position_if_new_timestamp(timestamp)
        self.mode_indicator = message.data[11]
        if not self.mode_indicator in ('R', 'F', 'D'):
            self.get_logger().warn(f'Mode indicator: {self.mode_indicator}, Correction age: {self.correction_age}')
            return
        self.latitude = message.latitude
        self.longitude = message.longitude
        self.speed_over_ground = float(message.data[6]) * 1.852  # knots to Km/hour
        self.course_over_ground = float(message.data[7])

    def handle_proprietary_sentence(self, message):
        message_id = message.data[0]
        if message_id == 'BLS':
            if len(message.data) < 9:
                self.get_logger().warn(f'Bad message: {message.data}')
                return
            mode_indicator = message.data[8]
            if not mode_indicator in ('R', 'F'):
                self.get_logger().warn(f'Mode indicator: {mode_indicator}, Correction age: {self.correction_age}')
                return
            timestamp = self.nvg_timestamp(message.data[1])
            self.clear_position_if_new_timestamp(timestamp)
            try:
                self.heading = float(message.data[6])
                self.get_logger().info(f'BLS heading: {self.heading}')
            except:
                self.get_logger().error('Bad BLS message')
                return
        elif message_id == 'BSS':
            correction_age_s = message.data[5]
            if correction_age_s:
                self.correction_age = float(correction_age_s)

    def handle_vtg(self, message):
        self.course_over_ground = float(message.data[0])
        self.speed_over_ground = float(message.data[6]) * 1000

    def handle_hdt(self, message):
        heading_s = message.data[0]
        if heading_s:
            self.heading = float(heading_s)
            self.get_logger().info(f'HDT heading: {self.heading}')

    def send_command(self, command):
        command_string = str(pynmea2.parse(command))
        self.get_logger().info(f'Command string: {command_string}')
        self.uart.write(b'\r\n\r\n')
        self.uart.write(command_string.encode())
        self.uart.write(b'\r\n')

    def get_message(self):
        sentence = None
        while sentence is None:
            try:
                sentence = self.streamreader.next()
            except Exception as e:
                self.get_logger().error(f'Sentence exception: {e}')
                continue
        if len(sentence) != 1:
            raise Exception(f'Sentence: {str(sentence)}')
        return sentence[0]

def main(args=None):
    rclpy.init(args=args)
    gps_nmea_node = GPSNMEANode()

    try:
        rclpy.spin(gps_nmea_node)
    except KeyboardInterrupt:
        gps_nmea_node.get_logger().info('Shutting down GPS NMEA Node...')
    finally:
        gps_nmea_node.stop()
        gps_nmea_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
