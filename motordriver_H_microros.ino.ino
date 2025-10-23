#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32.h>


#define LED_PIN 13

#define NUM_ENC 6
#define NUM_Motor 8
#define ENC_Calib 720/562000

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


rcl_publisher_t publisher_enc_msg;
std_msgs__msg__Float32MultiArray enc_msg;

rcl_subscription_t sub_motor_pwm;
rcl_subscription_t sub_esp_reset;
std_msgs__msg__Int32MultiArray motor_pwm;
std_msgs__msg__Int32 esp_reset;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

int PWMpin[NUM_Motor] = {40,38,36,34,9,18,16,7};   //front two, followed by back two, LRLR   //47 becomes 33 for S2 and 48 becomes 34
int dirpin[NUM_Motor] = {41,39,37,35,10,8,17,15};   //first four are drive.
int defaultdir[NUM_Motor] = {1,1,0,0,0,0,0,1};  // change only when hardware changes are made to rover or to standardize python code
int enc_dir[NUM_ENC] = {1,1,1,1,1,1};

int encA[NUM_ENC] = {13,11,4,1,33,20};
int encB[NUM_ENC] = {14,12,5,2,21,19};

volatile long enc_pos[NUM_ENC]={0};
volatile int lastEncoded[NUM_ENC] ={0};

int drive_buf[NUM_Motor] = {0};
float enc_feed[NUM_ENC] = {0};

// void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
// {  
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL) {
//     RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//     msg.data++;
//   }
// }

void sub_callback_motor_pwm_and_publisher(const void * msgin)
{  
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;


  // publish enc_values 
  for(int i=0;i<NUM_ENC;i++) enc_feed[i] = (enc_pos[i]*ENC_Calib*enc_dir[i]);
  enc_msg->data.data = (enc_feed);
  RCSOFTCHECK(rcl_publish(&publisher_enc_msg, &enc_msg, NULL));

  // subscribe and give motor pwm
  for(int i=0;i<NUM_Motor;i++)
  {
    drive_buf[i] = msg->data.data[i];
  }
  
  for(int i=0;i<4;i++)
  {
    if(drive_buf[i]>=0)
      {
        analogWrite(PWMpin[i],min(255,drive_buf[i]));
        digitalWrite(dirpin[i], defaultdir[i]);
      }

    else if(drive_buf[i]<0)
      {
        analogWrite(PWMpin[i],min(255,-drive_buf[i]));
        digitalWrite(dirpin[i], 1-defaultdir[i]);
      }
  }
  for(int i=4;i<8;i++)
  {
    if(drive_buf[i]>=0)
      {
        analogWrite(PWMpin[i],(min(255,drive_buf[i])));
        digitalWrite(dirpin[i], defaultdir[i]);
      }

    else if(drive_buf[i]<0)
      {
        analogWrite(PWMpin[i],(min(255,-drive_buf[i])));
        digitalWrite(dirpin[i], 1-defaultdir[i]);
      }
  }
  
}
void sub_callback_esp_reset(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32*)msgin;
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher for the encoder values
  RCCHECK(rclc_publisher_init_default(
    &publisher_enc_msg,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "enc_auto"));

  // create subscriber for motor pwm values
  RCCHECK(rclc_subscription_init_default(
    &sub_motor_pwm,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "motor_pwm"));

  // create subscriber for ESP reset 
  RCCHECK(rclc_subscription_init_default(
    &sub_esp_reset,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "esp_reset"));


  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  //RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_motor_pwm, &motor_pwm, &sub_callback_motor_pwm_and_publisher, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_esp_reset, &esp_reset, &sub_callback_esp_reset, ON_NEW_DATA));


  for(int i=0;i<NUM_ENC;i++)
  {
    pinMode(encA[i],INPUT_PULLUP);
    pinMode(encB[i],INPUT_PULLUP);
  }
  for(int i=0;i<NUM_Motor;i++)
  {
    pinMode(PWMpin[i], OUTPUT);
    pinMode(dirpin[i], OUTPUT);
  }
  attachInterrupt(digitalPinToInterrupt(encA[0]),updateEncoder0,CHANGE);    
  attachInterrupt(digitalPinToInterrupt(encA[1]),updateEncoder1,CHANGE);    
  attachInterrupt(digitalPinToInterrupt(encA[2]),updateEncoder2,CHANGE);    
  attachInterrupt(digitalPinToInterrupt(encA[3]),updateEncoder3,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[4]),updateEncoder4,CHANGE);    
  attachInterrupt(digitalPinToInterrupt(encA[5]),updateEncoder5,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB[0]),updateEncoder0,CHANGE);    
  attachInterrupt(digitalPinToInterrupt(encB[1]),updateEncoder1,CHANGE);    
  attachInterrupt(digitalPinToInterrupt(encB[2]),updateEncoder2,CHANGE);    
  attachInterrupt(digitalPinToInterrupt(encB[3]),updateEncoder3,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB[4]),updateEncoder4,CHANGE);    
  attachInterrupt(digitalPinToInterrupt(encB[5]),updateEncoder5,CHANGE);

  enc_msg.data.capacity = NUM_ENC;
  enc_msg.data.size = NUM_ENC;


}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  
}


//ISR for encoders
void updateEncoder0()
{
  int MSB = digitalRead(encA[0]); //MSB = most significant bit
  int LSB = digitalRead(encB[0]); //LSB = least significant bit
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[0] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) enc_pos[0] ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) enc_pos[0] --;
  
  lastEncoded[0] = encoded; //store this value for next time
}

void updateEncoder1()
{
  int MSB = digitalRead(encA[1]); //MSB = most significant bit
  int LSB = digitalRead(encB[1]); //LSB = least significant bit
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[1] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) enc_pos[1] --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) enc_pos[1] ++;
  
  lastEncoded[1] = encoded; //store this value for next time
}

void updateEncoder2()
{
  int MSB = digitalRead(encA[2]); //MSB = most significant bit
  int LSB = digitalRead(encB[2]); //LSB = least significant bit
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[2] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) enc_pos[2] ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) enc_pos[2] --;
  
  lastEncoded[2] = encoded; //store this value for next time
}

void updateEncoder3()
{
  int MSB = digitalRead(encA[3]); //MSB = most significant bit
  int LSB = digitalRead(encB[3]); //LSB = least significant bit
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[3] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) enc_pos[3] --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) enc_pos[3] ++;
  
  lastEncoded[3] = encoded; //store this value for next time
}

void updateEncoder4()
{
  int MSB = digitalRead(encA[4]); //MSB = most significant bit
  int LSB = digitalRead(encB[4]); //LSB = least significant bit
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[4] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) enc_pos[4] --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) enc_pos[4] ++;
  
  lastEncoded[4] = encoded; //store this value for next time
}

void updateEncoder5()
{
  int MSB = digitalRead(encA[5]); //MSB = most significant bit
  int LSB = digitalRead(encB[5]); //LSB = least significant bit
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[5] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) enc_pos[5] --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) enc_pos[5] ++;
  
  lastEncoded[5] = encoded; //store this value for next time
}
