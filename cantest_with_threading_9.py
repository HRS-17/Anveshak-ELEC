import can
import threading
import time
import rospy
import struct
import signal 
import sys
from std_msgs.msg import Int8, Float32, Int32MultiArray, Float32MultiArray, String
from sensor_msgs.msg import Joy
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension

class CAN_Class:
    def __init__(self):
        rospy.init_node("can_test")
        rospy.Subscriber("/motor_pwm", Int32MultiArray, self.joyCallback)
        rospy.Subscriber("/test_can", Int8, self.int8callback)

        self.pub_1 = rospy.Publisher("/final_message", Float32, queue_size=1000)
        self.enc_pub = rospy.Publisher("/enc_auto", Float32MultiArray, queue_size=1000)
        self.voltage_pub = rospy.Publisher("/esp_voltage", String, queue_size=10)

        self.enc_msg = Float32MultiArray()
        self.enc_msg.layout = MultiArrayLayout()
        self.enc_msg.layout.data_offset = 0
        self.enc_msg.layout.dim = [MultiArrayDimension()]
        self.enc_msg.layout.dim[0].size = 0
        self.enc_msg.layout.dim[0].stride = 0
        self.enc_msg.layout.dim[0].label = 'write'

        self.msg = can.Message(
            arbitration_id=0x0B1,
            is_extended_id=False,
            dlc=7,
            data=[1, 2, 3, 4, 5, 6, 0]
        )

        self.decoded_encoder_value = 0
        self.enc_factor = [90/268, 90/268, -90/268, 90/268]
        self.all_msgs = []
        self.enc_data = [0, 0, 0, 0, 0, 0]
        self.voltage_map = {}

        self.bus = can.ThreadSafeBus(channel='can0', bustype='socketcan', fd=True)
        self.data_lock = threading.Lock()
        self.event = threading.Event()

        receiver_thread = threading.Thread(target=self.receive_messages)
        receiver_thread.daemon = True
        receiver_thread.start()

        sender_thread = threading.Thread(target=self.send_messages)
        sender_thread.daemon = True
        sender_thread.start()

        publisher_thread = threading.Thread(target=self.publish_messages)
        publisher_thread.daemon = True
        publisher_thread.start()

        self.drive_dir = [1, -1, -1, 1, -1, -1, -1, -1]

    def int8callback(self, msg):
        if not self.data_lock.locked():
            self.msg.arbitration_id = 0x0B1
            self.msg.data = [msg.data]
            self.msg.dlc = 1
            self.event.set()

    def joyCallback(self, msg):
        if not self.data_lock.locked():
            self.msg.arbitration_id = 0x001
            self.msg.data = [(self.drive_dir[i]*msg.data[i])//2 + 127 for i in range(len(msg.data))]
            self.msg.dlc = len(self.msg.data)
            self.event.set()

    def publish_messages(self):
        while not rospy.is_shutdown():
            try:
                self.pub_1.publish(self.decoded_encoder_value)
                time.sleep(0.1)
            except rospy.ROSInterruptException:
                break

    def receive_messages(self):
        while not rospy.is_shutdown():
            try:
                msg = self.bus.recv()
                data = msg.data

                # Encoder data
                if msg.arbitration_id == 0x1b:
                    self.decoded_encoder_value = int.from_bytes(data[0:2], "little", signed=True)
                    self.enc_data[0] = int(self.enc_factor[0] * self.decoded_encoder_value)
                elif msg.arbitration_id == 0x1c:
                    self.decoded_encoder_value = int.from_bytes(data[0:2], "little", signed=True)
                    self.enc_data[1] = int(self.enc_factor[1] * self.decoded_encoder_value)
                elif msg.arbitration_id == 0x1d:
                    self.decoded_encoder_value = int.from_bytes(data[0:2], "little", signed=True)
                    self.enc_data[2] = int(self.enc_factor[2] * self.decoded_encoder_value)
                elif msg.arbitration_id == 0x1e:
                    self.decoded_encoder_value = int.from_bytes(data[0:2], "little", signed=True)
                    self.enc_data[3] = int(self.enc_factor[3] * self.decoded_encoder_value)

                self.enc_msg.data = self.enc_data
                self.all_msgs.append(self.decoded_encoder_value)
                self.enc_pub.publish(self.enc_msg)

                # ESP32 voltage messages
                elif msg.arbitration_id == 0x50 and len(msg.data) == 8:
                    try:
                        label = msg.data[0:4].decode('ascii')
                        voltage = struct.unpack('<f', msg.data[4:8])[0]
                        self.voltage_map[label] = voltage
                        log_msg = f"Voltage | {label} : {voltage:.3f} V"
                        print(log_msg)
                        self.voltage_pub.publish(log_msg)
                    except Exception as e:
                        print(f"Voltage decode error: {e}")

                # Debug print
                print(f"CAN msg received: ID={hex(msg.arbitration_id)} DATA={list(msg.data)}")

            except can.CanError as e:
                print(f"CAN receive error: {e}")

    def send_messages(self):
        while not rospy.is_shutdown():
            try:
                time.sleep(0.05)
                self.bus.send(self.msg)
                print("Message sent: ", list(self.msg.data))
            except can.CanError as e:
                print(f"CAN send error: {e}")

    def signal_handler(sig, frame):
        print("Interrupted, shutting down CAN.")
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    def mainthread(self):
        try:
            rospy.spin()
        finally:
            self.bus.shutdown()
            print("CAN bus shut down.")

if __name__ == "__main__":
    myObject = CAN_Class()
    myObject.mainthread()
