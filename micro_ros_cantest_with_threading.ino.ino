// SCRAP // use CAN_testwiht threading _MICROROS

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int32_multi_array.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <CAN.h> 


rcl_subscription_t sub_test_can;
std_msgs__msg__Int8 test_can_msg;

rcl_subscription_t sub_motor_pwm;
std_msgs__msg__Int32MultiArray motor_pwm_msg;

rcl_publisher_t publisher_final_msg;
std_msgs__msg__Float32 final_msg;

rcl_publisher_t publisher_enc_auto;
std_msgs__msg__Float32MultiArray enc_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

float enc_factor[4] = {90.0/268, 90.0/268, -90.0/268, 90.0/268};
float enc_data[6] = {0, 0, 0, 0, 0, 0};
float decoded_encoder_value= 0.0;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


void sub_callback_motor_pwm(const void *msgin)
{
  const std_msgs__msg_Int32MultiArray *msg = (const std_msgs__msg_Int32MultiArray *)msgin;
  
  size_t n = (size_t)msg->data.size;
  if (n > 8) n = 8;

  CAN.beginPacket(0x001);
  for (size_t i = 0; i < n; i++) {
    
    int val = (msg->data.data[i] / 2) + 127;
    if (val < 0) val = 0;
    if (val > 255) val = 255;
    CAN.write((uint8_t)val);
  }
  
  // for (int i = 0; i < msg->data.size && i < 8; i++) {
  //   int val = msg->data.data[i] / 2 + 127;  
  //   CAN.write(val);
  // }
  CAN.endPacket();
  
}

void sub_callback_test_can(const void *msgin)
{
  const std_msgs__msg_Int8 *msg = (const std_msgs__msg_Int8 *)msgin;
  CAN.beginPacket(0x0B1);
  CAN.write(msg->data);
  CAN.endPacket();
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) timer;
  (void) last_call_time;

  int packet_len = CAN.parsePacket();
  if (packet_len > 0) {
    int id = CAN.packetId();
    uint8_t data[8] = {0};
    int len = packet_len;
    if (len > 8) len = 8;

    for (int i = 0; i < len; i++) {
      int v = CAN.read();
      if (v < 0) v = 0;
      data[i] = (uint8_t)v;
    }

    // make sure there are at least two bytes before decoding
    if (len >= 2) {
      int16_t raw_val = (int16_t)((data[0] & 0xFF) | ((data[1] & 0xFF) << 8));
      decoded_encoder_value = (float)raw_val;
    } else {
      decoded_encoder_value = 0.0f;
    }

    int index = id - 0x1b; // same mapping as Python
    if (index >= 0 && index < 6) {
      enc_data[index] = enc_factor[index % 4] * decoded_encoder_value;
    }
        // Publish single encoder value
    final_msg.data = decoded_encoder_value;
    RCSOFTCHECK(rcl_publish(&publisher_final_msg, &final_msg, NULL));

    // Publish enc_data array
    // copy into enc_msg.data (already initialized in setup)
    for (size_t i = 0; i < 6; ++i) {
      enc_msg.data.data[i] = enc_data[i];
    }
    enc_msg.data.size = 6;
    RCSOFTCHECK(rcl_publish(&publisher_enc_auto, &enc_msg, NULL));
  }
}

void setup() {
  set_microros_transports();

// CAN.setPins(TX_PIN, RX_PIN);
  if (!CAN.begin(500E3)) {  // Start CAN at 500kbps
    while (1);
  }
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_cantest_with_threading", "", &support));

  // create subscriber for test_Can
  RCCHECK(rclc_subscription_init_default(
    &sub_test_can,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/test_can"));
  // create subscriber for motor_pwm values
  RCCHECK(rclc_subscription_init_default(
    &sub_motor_pwm,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/motor_pwm"));

  // create publisher for enc_msg
  RCCHECK(rclc_publisher_init_default(
  &publisher_final_msg,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
  "/final_message"));
  // create publisher for 
  RCCHECK(rclc_publisher_init_default(
  &publisher_enc_auto,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
  "/enc_auto"));


  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_test_can, &test_can_msg, &sub_callback_test_can, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor_pwm, &motor_pwm_msg, &sub_callback_motor_pwm, ON_NEW_DATA));

  // Timer (100 ms)
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback);
  rclc_executor_add_timer(&executor, &timer);

}

void loop() {
  
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
