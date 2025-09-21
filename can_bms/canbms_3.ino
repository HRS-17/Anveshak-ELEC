#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "driver/twai.h"

#define SDA_PIN 21
#define SCL_PIN 22

#define RX_PIN 25
#define TX_PIN 26

#define TRANSMIT_RATE_MS 10000

#define CAN_BASE_ID 0x37

Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;
Adafruit_ADS1115 ads3;

bool ads1_ready = false, ads2_ready = false, ads3_ready = false, can_ok = false;

unsigned long txPreviousMillis = 0, canRetryPreviousMillis = 0, currentMillis = 0;

typedef struct {
  float ads1_volts[4];
  float ads2_volts[4];
  float ads3_volts[4];
} ads_readings_t;

ads_readings_t adc_readings = {0};
float cell_voltages[12] = {0.0f};

void setup() {
  Serial.begin(115200);       // ✅ Serial debug at 115200 baud
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);

  can_ok = CAN_Init();

  ads1_ready = ads1.begin(0x48, &Wire);
  if (ads1_ready) ads1.setGain(GAIN_ONE);
  ads2_ready = ads2.begin(0x49, &Wire);
  if (ads2_ready) ads2.setGain(GAIN_ONE);
  ads3_ready = ads3.begin(0x4A, &Wire);
  if (ads3_ready) ads3.setGain(GAIN_ONE);

  // if (ads1_ready) {Serial.println("ADS1115 0x48 OK"); }
  // else { Serial.println("ADS1115 0x48 ERROR"); }

  // if (ads2_ready) { send_status_message("I:49"); Serial.println("ADS1115 0x49 OK"); }
  // else { send_status_message("E:49"); Serial.println("ADS1115 0x49 ERROR"); }

  // if (ads3_ready) { send_status_message("I:4A"); Serial.println("ADS1115 0x4A OK"); }
  // else { send_status_message("E:4A"); Serial.println("ADS1115 0x4A ERROR"); }

  // if (can_ok) { send_status_message("I:CAN"); Serial.println("CAN bus OK"); }
  // else { send_status_message("E:CAN"); Serial.println("CAN bus ERROR"); }
}

void loop() {
  currentMillis = millis();

  if (!can_ok) {
    if (currentMillis - canRetryPreviousMillis >= 1000){
      canRetryPreviousMillis = currentMillis;
      can_ok = CAN_Init();
    }
    return;
  }

  read_ADS();
  changeCumulativeToDifference();

  if (currentMillis - txPreviousMillis >= TRANSMIT_RATE_MS) {
    txPreviousMillis = currentMillis;
    send_cell_voltages();
  }

  uint32_t alerts;
  if (twai_read_alerts(&alerts, 0) == ESP_OK) {
    if (alerts & TWAI_ALERT_BUS_OFF) {
      can_ok = false;
      twai_initiate_recovery(); // try recovering
    }
    if (alerts & TWAI_ALERT_BUS_RECOVERED) {
      can_ok = true; // bus is back
    }
  }

  delay(10);
}

void read_ADS() {
  if (ads1_ready) {
    for (int i = 0; i < 4; i++) {
      int16_t adc = ads1.readADC_SingleEnded(i);
      adc_readings.ads1_volts[i] = adc * 4.096 * 11.0 / 32768.0;
    }
  }

  if (ads2_ready) {
    for (int i = 0; i < 4; i++) {
      int16_t adc = ads2.readADC_SingleEnded(i);
      adc_readings.ads2_volts[i] = adc * 4.096 * 11.0 / 32768.0;
    }
  }
  
  if (ads3_ready) {
    for (int i = 0; i < 4; i++) {
      int16_t adc = ads3.readADC_SingleEnded(i);
      adc_readings.ads3_volts[i] = adc * 4.096 * 11.0 / 32768.0 ;
    }
  }
}

bool CAN_Init() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return false;
  if (twai_start() != ESP_OK) return false;

  uint32_t alerts = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_FAILED;
  if (twai_reconfigure_alerts(alerts, NULL) != ESP_OK) return false;

  return true;
}

// void send_status_message(const char* text) {
//   if (!can_ok) return;

//   twai_message_t msg = {0};
//   msg.identifier = diagnostic_identifier;
//   msg.data_length_code = strlen(text) > 8 ? 8 : strlen(text);
//   memcpy(msg.data, text, msg.data_length_code);

//   twai_transmit(&msg, pdMS_TO_TICKS(100));

//   Serial.print("Status CAN Msg Sent: ");
//   Serial.println(text);
// }

void send_cell_voltages() {
  if (!can_ok) return;

  for (int frame_id=0; frame_id<3; frame_id++){
    twai_message_t msg = {0};
    msg.identifier = CAN_BASE_ID + frame_id;
    msg.extd = 0;
    msg.rtr = 0;
    msg.ss = 0;
    msg.self = 0;
    msg.data_length_code = 8;

    int base = frame_id*4;
    for (int j=0; j<4; j++){
      uint16_t scaled = (uint16_t)(cell_voltages[base+j] * 10000.0f);
      msg.data[j*2] = scaled & 0xFF;
      msg.data[j*2+1] = (scaled>>8) & 0xFF;
    }
    
    if (twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {
      Serial.print("Sent Frame ID ");
      Serial.print(frame_id);
      Serial.print(": ");
      for (int j = 0; j < 4; j++) {
        Serial.print(cell_voltages[base + j], 4);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.print("CAN TX Fail Frame: ");
      Serial.println(frame_id);
      can_ok = false;
      break;
    }
  }
}

inline float sanitize(float v, float threshold = 0.2f, float fallback = 5.0f) {
  if (v < threshold) return fallback;
  return v;
}

void changeCumulativeToDifference(){
  // Battery 1
  cell_voltages[0]   = sanitize(adc_readings.ads1_volts[3]);                                  // b1c1
  cell_voltages[1]   = sanitize(adc_readings.ads1_volts[2] - adc_readings.ads1_volts[3]);    // b1c2
  cell_voltages[2]   = sanitize(adc_readings.ads1_volts[1] - adc_readings.ads1_volts[2]);    // b1c3
  cell_voltages[3]   = sanitize(adc_readings.ads1_volts[0] - adc_readings.ads1_volts[1]);    // b1c4
  cell_voltages[4]   = sanitize(adc_readings.ads2_volts[0] - adc_readings.ads1_volts[0]);    // b1c5
  cell_voltages[5]   = sanitize(adc_readings.ads2_volts[1] - adc_readings.ads2_volts[0]);    // b1c6

  // Battery 2
  cell_voltages[6]   = sanitize(adc_readings.ads2_volts[2]);                                  // b2c1
  cell_voltages[7]   = sanitize(adc_readings.ads2_volts[3] - adc_readings.ads2_volts[2]);    // b2c2
  cell_voltages[8]   = sanitize(adc_readings.ads3_volts[0] - adc_readings.ads2_volts[3]);    // b2c3
  cell_voltages[9]   = sanitize(adc_readings.ads3_volts[1] - adc_readings.ads3_volts[0]);    // b2c4
  cell_voltages[10]  = sanitize(adc_readings.ads3_volts[2] - adc_readings.ads3_volts[1]);    // b2c5
  cell_voltages[11]  = sanitize(adc_readings.ads3_volts[3] - adc_readings.ads3_volts[2]);    // b2c6
}