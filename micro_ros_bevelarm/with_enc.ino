#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>

#define LED_PIN 13
#define NUM_ENC 3
#define NUM_MOTORS 6
#define MAX_PWM_VALUE 255
#define TIMER_PERIOD_MS 25  // 40Hz for better control response

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { (void)fn; }

// ROS components
rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray pwm_int;  

rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray enc_msg;  

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Encoder variables with interrupt protection
volatile long enc_pos[NUM_ENC] = {0};
volatile bool A_set[NUM_ENC] = {false};
volatile bool B_set[NUM_ENC] = {false};
volatile bool enc_updated = false;

// Motor pin configurations
int PWMpin[NUM_MOTORS] = {4, 6, 16, 18, 9, 11};
int dirpin[NUM_MOTORS] = {5, 7, 15, 17, 8, 10};
int defaultdir[NUM_MOTORS] = {1, 1, 1, 1, 1, 1};

// ESP32-S3 encoder pins
int encB[NUM_ENC] = {42, 38, 47};
int encA[NUM_ENC] = {41, 37, 21};

// Data buffers
int pwm_int_buffer[NUM_MOTORS] = {0};
float enc_feed[NUM_ENC] = {0};
int32_t pwm_data_array[NUM_MOTORS] = {0};  // Static array for subscriber

// Conversion factor - adjust based on your encoder specifications
const float ENC_TO_DEGREES = 90.0 / 55157.0;

// Safety limits
const int MIN_PWM = -MAX_PWM_VALUE;
const int MAX_PWM = MAX_PWM_VALUE;

// LED blink on error
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Enhanced subscriber callback with bounds checking
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *msg = 
      (const std_msgs__msg__Int32MultiArray *)msgin;
  
  // Bounds checking and safe copying
  int copy_size = min((int)msg->data.size, NUM_MOTORS);
  for (int i = 0; i < copy_size; i++) {
    // Apply safety limits
    pwm_int_buffer[i] = constrain(msg->data.data[i], MIN_PWM, MAX_PWM);
  }
  
  // Zero remaining motors if fewer commands received
  for (int i = copy_size; i < NUM_MOTORS; i++) {
    pwm_int_buffer[i] = 0;
  }
}

// Enhanced timer callback with interrupt protection
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  // Apply PWM commands to motors with enhanced safety
  for (int i = 0; i < NUM_MOTORS; i++) {
    int pwm_value = abs(pwm_int_buffer[i]);
    pwm_value = constrain(pwm_value, 0, MAX_PWM_VALUE);
    
    if (pwm_int_buffer[i] >= 0) {
      analogWrite(PWMpin[i], pwm_value);
      digitalWrite(dirpin[i], defaultdir[i]);
    } else {
      analogWrite(PWMpin[i], pwm_value);
      digitalWrite(dirpin[i], 1 - defaultdir[i]);
    }
  }

  // Protected encoder reading to prevent race conditions
  long enc_copy[NUM_ENC];
  noInterrupts();
  for (int j = 0; j < NUM_ENC; j++) {
    enc_copy[j] = enc_pos[j];
  }
  bool updated = enc_updated;
  enc_updated = false;
  interrupts();

  // Convert encoder positions to degrees using copied values
  for (int j = 0; j < NUM_ENC; j++) {
    enc_feed[j] = enc_copy[j] * ENC_TO_DEGREES;
  }

  // Publish encoder data
  enc_msg.data.data = enc_feed;
  enc_msg.data.size = NUM_ENC;
  enc_msg.data.capacity = NUM_ENC;
  RCSOFTCHECK(rcl_publish(&publisher, &enc_msg, NULL));
}

void setup() {
  set_microros_transports();
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);  // Wait for micro-ROS agent

  // Initialize ROS components
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "microros_bevel_arm", "", &support));

  // Initialize subscriber with proper message setup
  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "stm_write"));

  // Initialize publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "enc_arm"));

  // Setup timer with improved frequency for better control
  RCCHECK(rclc_timer_init_default(
    &timer, &support,
    RCL_MS_TO_NS(TIMER_PERIOD_MS),
    timer_callback));

  // Initialize executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &pwm_int,
                                         &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Setup motor pins
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(PWMpin[i], OUTPUT);
    pinMode(dirpin[i], OUTPUT);
    analogWrite(PWMpin[i], 0);  // Initialize to zero
    digitalWrite(dirpin[i], defaultdir[i]);
  }

  // Setup encoder pins and interrupts
  for (int i = 0; i < NUM_ENC; i++) {
    pinMode(encA[i], INPUT_PULLUP);
    pinMode(encB[i], INPUT_PULLUP);
  }
  
  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(encA[0]), callback_0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[1]), callback_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[2]), callback_2, CHANGE);

  // Initialize subscriber message structure properly
  pwm_int.layout.data_offset = 0;
  pwm_int.layout.dim.data = NULL;
  pwm_int.layout.dim.size = 0;
  pwm_int.layout.dim.capacity = 0;
  pwm_int.data.data = pwm_data_array;  // Use static array
  pwm_int.data.size = 0;
  pwm_int.data.capacity = NUM_MOTORS;

  // Initialize publisher message structure
  enc_msg.layout.data_offset = 0;
  enc_msg.layout.dim.data = NULL;
  enc_msg.layout.dim.size = 0;
  enc_msg.layout.dim.capacity = 0;
  enc_msg.data.data = enc_feed;
  enc_msg.data.size = NUM_ENC;
  enc_msg.data.capacity = NUM_ENC;

  Serial.println("micro-ROS bevel arm controller initialized");
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  
  // Optional: Add watchdog or health monitoring here
  static unsigned long last_heartbeat = 0;
  if (millis() - last_heartbeat > 5000) {  // 5 second heartbeat
    Serial.println("System running...");
    last_heartbeat = millis();
  }
}

// Enhanced encoder interrupt service routines with better edge detection
void IRAM_ATTR callback_0() {
  static unsigned long last_interrupt_0 = 0;
  unsigned long interrupt_time = micros();
  
  // Debounce protection (adjust if needed)
  if (interrupt_time - last_interrupt_0 > 100) {
    int i = 0;
    bool A_current = digitalRead(encA[i]);
    bool B_current = digitalRead(encB[i]);
    
    // Quadrature decoding
    if (A_current != A_set[i]) {
      A_set[i] = A_current;
      if (A_current == B_current) {
        enc_pos[i]++;
      } else {
        enc_pos[i]--;
      }
      enc_updated = true;
    }
  }
  last_interrupt_0 = interrupt_time;
}

void IRAM_ATTR callback_1() {
  static unsigned long last_interrupt_1 = 0;
  unsigned long interrupt_time = micros();
  
  if (interrupt_time - last_interrupt_1 > 100) {
    int i = 1;
    bool A_current = digitalRead(encA[i]);
    bool B_current = digitalRead(encB[i]);
    
    if (A_current != A_set[i]) {
      A_set[i] = A_current;
      if (A_current == B_current) {
        enc_pos[i]++;
      } else {
        enc_pos[i]--;
      }
      enc_updated = true;
    }
  }
  last_interrupt_1 = interrupt_time;
}

void IRAM_ATTR callback_2() {
  static unsigned long last_interrupt_2 = 0;
  unsigned long interrupt_time = micros();
  
  if (interrupt_time - last_interrupt_2 > 100) {
    int i = 2;
    bool A_current = digitalRead(encA[i]);
    bool B_current = digitalRead(encB[i]);
    
    if (A_current != A_set[i]) {
      A_set[i] = A_current;
      if (A_current == B_current) {
        enc_pos[i]++;
      } else {
        enc_pos[i]--;
      }
      enc_updated = true;
    }
  }
  last_interrupt_2 = interrupt_time;
}
