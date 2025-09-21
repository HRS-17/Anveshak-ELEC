import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile

import can
import threading
import time
import struct
#import rospy
import signal 
import sys
# from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, Float32, Int32MultiArray
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32MultiArray

class CAN_Class(Node):
    def __init__(self, node_name: str, *, context: rclpy.Context | None = None, cli_args: rclpy.List[str] | None = None, namespace: str | None = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: rclpy.List[rclpy.Parameter] | None = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        print("super hello")
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
        print("hello")
        self.subscriber_1 = self.create_subscription(Int32MultiArray, "/motor_pwm", self.joyCallback, 10)        
        self.subscriber_2 = self.create_subscription(Int8, "/test_can", self.int8callback, 10)

        self.pub_1 = self.create_publisher(Float32, "/final_message", QoSProfile(depth=10))
        self.enc_pub = self.create_publisher(Float32MultiArray, "/enc_auto", QoSProfile(depth=10))

        self.enc_msg = Float32MultiArray()
        self.enc_msg.layout = MultiArrayLayout()
        self.enc_msg.layout.data_offset = 0
        self.enc_msg.layout.dim = [ MultiArrayDimension() ]
        self.enc_msg.layout.dim[0].size = self.enc_msg.layout.dim[0].stride = len(self.enc_msg.data)
        self.enc_msg.layout.dim[0].label = 'write'

        self.msg = can.Message(
            arbitration_id=0x0B1,
            is_extended_id=False,
            dlc=7,  
            data= [1,2,3,4,5,6,0]
        )

        self.decoded_encoder_value = 0.0
        self.enc_factor = [90/268,90/268,-90/268,90/268]	
        self.all_msgs = []
        self.bus = can.ThreadSafeBus(channel='can0', bustype='socketcan', fd=True)

        self.data_lock = threading.Lock()

        self.event = threading.Event()
        receiver_thread = threading.Thread(target=self.receive_messages, args=())
        receiver_thread.daemon = True
        receiver_thread.start()

        self.get_logger().info("I am about to send")
        self.total_start_time = time.time()
        sender_thread = threading.Thread(target=self.send_messages, args=())
        sender_thread.daemon = True
        sender_thread.start()
        self.drive_dir = [1,1,1,1,-1,-1,-1,-1]

        self.get_logger().info("I am about to publish")
        publisher_thread = threading.Thread(target=self.publish_messages, args=())
        publisher_thread.daemon = True
        publisher_thread.start()
        self.enc_data=[0.0,0.0,0.0,0.0,0.0,0.0]

        # BMS Part
        self.bms_can_msg_ids = [0x37, 0x38, 0x39]
        self.bms_pub = self.create_publisher(Float32MultiArray, '/bms/battery_values', QoSProfile(depth=10))
        self.cell_voltages = [0.0]*12
        self.get_logger().info("Created BMS Publisher")
        self.received_bms = set()
        self.bms_debug=False
    
    def int8callback(self, msg: Int8):
        self.get_logger().info("In Callback")
        #self.data_lock.acquire()
        #if(self.data_lock.locked() == True):
        with self.data_lock:
            self.msg.arbitration_id = 0x0B1
            self.msg.data = bytearray([msg.data])
            self.msg.dlc = 1
            self.event.set()
            #self.data_lock.release()


    def joyCallback(self, msg: Int32MultiArray):
        #self.data_lock.acquire()
        #if(self.data_lock.locked() == False):
            with self.data_lock:
                self.msg.arbitration_id = 0x001
                self.msg.data =  bytearray([(self.drive_dir[i] * msg.data[i]) // 2 + 127 for i in range(min(8, len(msg.data)))])
                self.msg.dlc = len(self.msg.data)
                self.event.set()
            #    self.data_lock.release()
            
            
    def publish_messages(self):
        while True:
           try:
               #self.get_logger().info("----------------------------------------------")
               self.pub_1.publish(Float32(data=self.decoded_encoder_value))
               #self.get_logger().info(f"I published the message {self.decoded_encoder_value}")
               time.sleep(0.1)
           except KeyboardInterrupt:
               self.get_logger().info("Interrupted by user")
               break
           

    def receive_messages(self):
        while True:
            try:
                msg = self.bus.recv()
                self.get_logger().info(f"Found id: {msg.arbitration_id}")
                self.get_logger().warn(f"msg={msg}")
                data = msg.data
                # Steering motor part
                    #self.enc_msg.data=[0,0,0,0]
                
                # BMS Part
                if msg.arbitration_id in self.bms_can_msg_ids:
                    try:
                        frame_index = msg.arbitration_id - 0x37
                        base = frame_index*4
                        
                        for j in range(4):
                            raw_bytes = msg.data[j*2:j*2+2]
                            scaled, = struct.unpack('<H', raw_bytes)
                            voltage = scaled / 10000.0
                            self.cell_voltages[base+j] = voltage

                        self.received_bms.add(frame_index)
                        if len(self.received_bms) == 3:
                            arr_msg = Float32MultiArray()
                            arr_msg.data = self.cell_voltages
                            self.bms_pub.publish(arr_msg)

                            self.get_logger().info("BMS Data published")
                            self.received_bms.clear()

                    except Exception as e:
                        self.get_logger().error(f"BMS: Error processing message: {e}")                    
                    
                    continue
                
                if index in [0x1b, 0x1c, 0x1d, 0x1e]:
                    self.decoded_encoder_value = int.from_bytes(data[0:2],"little", signed=True)
                    index = msg.arbitration_id - 0x1b
                    self.enc_data[index] = int(self.enc_factor[index] * self.decoded_encoder_value)
                    self.enc_msg.data = self.enc_data
                
                    #self.decoded_encoder_value = int.from_bytes(data[0:1],"big", signed=True) 
                    self.all_msgs.append(self.decoded_encoder_value)
                    self.get_logger().info(f"Message Received = {self.decoded_encoder_value}")
                    if msg:
                        self.get_logger().info(f"Received message: {msg}")
                        self.get_logger().info(f"Recieved value is {self.decoded_encoder_value}")
                        self.enc_pub.publish(self.enc_msg)
            except can.CanError as e:
                self.get_logger().info(f"Error receiving message: {e}")


    def send_messages(self):
        while True:
            try: 
                #event_set = self.event.wait() 
                time.sleep(0.05)
                with self.data_lock:
                    self.bus.send(self.msg)
                    #self.event.clear()
                    self.get_logger().info("Message sent successfully:")
                    #self.get_logger().info(self.msg.data)
                    msg_data = bytearray(self.msg.data)
                    print("msg_data ", msg_data)
                    self.get_logger().info("Data sent is:")
                    for i in range(len(self.msg.data)):
                        print("msg_data ", msg_data[i])
                        #self.get_logger().info(int(msg_data[i]))
                    self.get_logger().info("")
            except can.CanError as e:
                self.get_logger().info(f"Error sending message: {e}")


                
def signal_handler(sig, frame):

    print("**********************Keyboard interrupt************************")
    sys.exit(0)


signal.signal(signal.SIGINT,signal_handler)


def main():
    rclpy.init()
    node = CAN_Class("Hello")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received.")
    finally:
        print("Shutting down...")
        try:
            node.bus.shutdown()  # Or node.bus.close(), depending on CAN lib
        except Exception as e:
            print(f"Error during bus shutdown: {e}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



# def mainthread(self):

#     rate = rospy.Rate(10)
#     try:
#         while not rospy.is_shutdown():
#             rospy.spin()
#     #except KeyboardInterrupt:
#     #    self.get_logger().info("Interrupted by user")



#     #    sys.exit(0)
#     finally:
#         sys.exit(0)
#         self.bus.shutdown()
#         self.get_logger().info("In finally block")


