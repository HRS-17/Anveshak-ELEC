


#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SparkFun_u-blox_GNSS_v3.h>

// micro-ROS includes
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/imu.h>

// ------------------- Hardware setup -------------------
int sda_mpu = 35;
int scl_mpu = 36;
int sda_gps = 17;
int scl_gps = 16;
int freq = 100000;

Adafruit_MPU6050 mpu;
SFE_UBLOX_GNSS myGNSS;

TwoWire I2CBusMPU = TwoWire(0);
TwoWire I2CBusGPS = TwoWire(1);

sensors_event_t a, g, temp;

// ------------------- micro-ROS setup -------------------
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t gps_pub;
rcl_publisher_t imu_pub;

sensor_msgs__msg__NavSatFix gps_msg;
sensor_msgs__msg__Imu imu_msg;

rclc_executor_t executor;
rcl_timer_t timer;

// ------------------- Timer callback -------------------
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;

  // ==== Read IMU ====
  mpu.getEvent(&a, &g, &temp);

  imu_msg.linear_acceleration.x = a.acceleration.x;
  imu_msg.linear_acceleration.y = a.acceleration.y;
  imu_msg.linear_acceleration.z = a.acceleration.z;

  imu_msg.angular_velocity.x = g.gyro.x;
  imu_msg.angular_velocity.y = g.gyro.y;
  imu_msg.angular_velocity.z = g.gyro.z;

  imu_msg.orientation.w = 1.0; // dummy (no fusion yet)
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;

  rcl_publish(&imu_pub, &imu_msg, NULL);

  // ==== Read GNSS ====
  double lat = myGNSS.getLatitude() / 1e7;   // degrees
  double lon = myGNSS.getLongitude() / 1e7;  // degrees
  double alt = myGNSS.getAltitude() / 1000.0; // meters

  gps_msg.latitude = lat;
  gps_msg.longitude = lon;
  gps_msg.altitude = alt;

  gps_msg.position_covariance[0] = 0.0; // fill if needed
  gps_msg.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_UNKNOWN;
  gps_msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_FIX;
  gps_msg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;

  rcl_publish(&gps_pub, &gps_msg, NULL);
}

// ------------------- Arduino setup -------------------
void setup() {
  // micro-ROS transport
  set_microros_transports();
  delay(2000);
  pinMode(sda_mpu,INPUT_PULLUP);
  pinMode(sda_gps,INPUT_PULLUP);
  // I2C buses
  I2CBusMPU.begin(sda_mpu, scl_mpu, freq);
  I2CBusGPS.begin(sda_gps, scl_gps, freq);



  // Init IMU
  if (!mpu.begin(0x68, &I2CBusMPU)) {
    while (1) { Serial.println("MPU6050 not found!"); delay(1000); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // Init GNSS
  if (!myGNSS.begin(I2CBusGPS)) {
    while (1) { Serial.println("GNSS not found!"); delay(1000); }
  }

  // micro-ROS init
  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);

  // Publishers
  rclc_publisher_init_default(
    &gps_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
    "gps/fix");

  rclc_publisher_init_default(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data_raw");

  // Timer (500ms)
  const unsigned int timer_timeout = 500;
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback);

  // Executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
