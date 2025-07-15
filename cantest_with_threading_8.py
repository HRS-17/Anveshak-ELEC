import can
import threading
import time
import rospy
import struct
import signal 
import sys
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, Float32, Int32MultiArray,String
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32MultiArray


class CAN_Class:
    def __init__(self):

        rospy.init_node("can_test")
        rospy.Subscriber("/motor_pwm", Int32MultiArray, self.joyCallback)
        rospy.Subscriber("/test_can", Int8, self.int8callback)
        self.pub_1 = rospy.Publisher("/final_message",Float32, queue_size = 1000)
        self.enc_pub = rospy.Publisher("/enc_auto",Float32MultiArray, queue_size = 1000)
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
        self.decoded_encoder_value = 0
        self.enc_factor = [90/268,90/268,-90/268,90/268]	
        self.all_msgs = []
        self.bus = can.ThreadSafeBus(channel='can0', bustype='socketcan', fd=True)
        
        self.data_lock = threading.Lock()

        self.event = threading.Event()
        receiver_thread = threading.Thread(target=self.receive_messages, args=())
        receiver_thread.daemon = True
        receiver_thread.start()
        
        print("I am about to send")
        self.total_start_time = time.time()
        sender_thread = threading.Thread(target=self.send_messages, args=())
        sender_thread.daemon = True
        sender_thread.start()
        self.drive_dir = [1,-1,-1,1,-1,-1,-1,-1]
        print("I am about to publish")
        publisher_thread = threading.Thread(target=self.publish_messages, args=())
        publisher_thread.daemon = True
        publisher_thread.start()
        self.enc_data=[0,0,0,0,0,0]
    
    def int8callback(self, msg):
        print("In Callback")
        #self.data_lock.acquire()
        if(self.data_lock.locked() == True):
            self.msg.arbitration_id = 0x0B1
            self.msg.data = [msg.data]
            self.msg.dlc = 1
            self.event.set()
            #self.data_lock.release()

    def joyCallback(self,msg):
       # self.data_lock.acquire()
        if(self.data_lock.locked() == False):
            self.msg.arbitration_id = 0x001
            self.msg.data =  [(self.drive_dir[i]*msg.data[i])//2 + 127 for i in range(len(msg.data))]
            self.msg.dlc = len(self.msg.data)
            self.event.set()
        #    self.data_lock.release()
            
    def publish_messages(self):
        while True:
           try:
              
               #print("----------------------------------------------")
               self.pub_1.publish(self.decoded_encoder_value)
               #print(f"I published the message {self.decoded_encoder_value}")
               time.sleep(0.1)
           except KeyboardInterrupt:
               print("Interrupted by user")
               break

    def receive_messages(self):
        while True:
            try:
                msg = self.bus.recv()
                data = msg.data
                #self.enc_msg.data=[0,0,0,0]
                if msg.arbitration_id == 0x1b:
                    self.decoded_encoder_value = int.from_bytes(data[0:2],"little", signed=True)
                    self.enc_data[0] = int(self.enc_factor[0] * self.decoded_encoder_value)
                elif msg.arbitration_id == 0x1c:
                    self.decoded_encoder_value = int.from_bytes(data[0:2],"little", signed=True)
                    self.enc_data[1] = int(self.enc_factor[1] *self.decoded_encoder_value)
                elif msg.arbitration_id == 0x1d:
                    self.decoded_encoder_value = int.from_bytes(data[0:2],"little", signed=True)
                    self.enc_data[2] = int(self.enc_factor[2] *self.decoded_encoder_value)
                elif msg.arbitration_id == 0x1e:
                    self.decoded_encoder_value = int.from_bytes(data[0:2],"little", signed=True)
                    self.enc_data[3] = int(self.enc_factor[3] *self.decoded_encoder_value)
                self.enc_msg.data=self.enc_data
                
                #self.decoded_encoder_value = int.from_bytes(data[0:1],"big", signed=True) 
                self.all_msgs.append(self.decoded_encoder_value)
                print(f"Message Received = {self.decoded_encoder_value}")
                if msg:
                    print(f"Received message: {msg}")
                    print("Recieved value is ",self.decoded_encoder_value)
                    self.enc_pub.publish(self.enc_msg)
            except can.CanError as e:
                print(f"Error receiving message: {e}")
                

    def send_messages(self):
        start_time = time.time()
        while True:
            try: 
                #event_set = self.event.wait() 
                time.sleep(0.05)
                self.bus.send(self.msg)
                #self.event.clear()
                start_time = time.time()
                print("Message sent successfully:") 
                #print(self.msg.data)
                msg_data = bytearray(self.msg.data)
                print("Data sent is:", end = " ")
                for i in range(len(self.msg.data)):
                    print(msg_data[i], end= " ")
                print()
            except can.CanError as e:
                print(f"Error sending message: {e}")
                
    def signal_handler(sig,frame):
        print("**********************Keyboard interrupt************************")
        sys.exit(0)
    signal.signal(signal.SIGINT,signal_handler)

    def mainthread(self):

        rate = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                rospy.spin()
        #except KeyboardInterrupt:
        #    print("Interrupted by user")
        #    sys.exit(0)
        finally:
            sys.exit(0)
            self.bus.shutdown()
            print("In finally block")

if __name__ == "__main__":
    myObject = CAN_Class()
    myObject.mainthread()


