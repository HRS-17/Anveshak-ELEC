/*fixes:
  1.less redendent
  2. Remove pwm multiplier
  3.Can and button simultaneously
/*Flow:
    1.Initizlization
    2.Check if the CAN driver is installed.
    3.Receive encoder values from PCNT.
    4.Convert encoder values and accumulator data into angles.
    5.Calculate DIP switch values.(goes into if statement only if interrupt occured)
    6.Read alerts and status of can bus
    7.handling of errors and alerts
    8.If a new CAN message is received, process it using alerts and provide PWM to the motor immediately.
    9.else check for a button press; if pressed, provide PWM to the motor.
    10.if delay is more than transmit rate then feedback message is sent.
    11.delay of 10ms
*/

#include "driver/twai.h"
#include "driver/pcnt.h"
#include "math.h"

#define PCNT_UNIT PCNT_UNIT_0
#define PCNT_H_LIM 32767
#define PCNT_L_LIM -32768

#define ENC_DIR 1
#define ENC_Calib 3*360/float(562000/3.9665)

#define RX_PIN 25
#define TX_PIN 26
#define ENCODER_GPIO_A 32
#define ENCODER_GPIO_B 33

#define TRANSMIT_RATE_MS 100
#define POLLING_RATE_MS 500

#define MA 14
#define MB 13
#define PWM 22
#define INA 23
#define INB 21

// Defines for the DIPswitch pins
#define dP0 4
#define dP1 17
#define dP2 5
#define dP3 19

#define DEF_SPEED 50 // Default PWM written to the motors when the buttons on the board are pressed manually

int dipPins[] = {dP0, dP1, dP2, dP3}; // The pins connected to the DIPswitches in the same order

// Directions of rotation of the wheels depending on dip switch values
// Assumed that there are only 8 motor drivers and  0 <= dipValues <= 7
//int pwm_multiplier[] = {1, 1, 1, 1, 1, 1, 1, 1}; // Modify this array to choose directions for the motor drivers   
// Suppose the motor drivers have to rotate normally(positive rotation with ma high and mb low), set the corresponding value to +1
// Else set it to -1, representing opposite configuration. This will be used in the part where pwm_values are sent to the motors

int16_t count = 0;
volatile int accumulator = 0;
float angles=0;

static bool driver_installed = false;
uint32_t alerts_triggered;
twai_status_info_t twaistatus;
unsigned long previousMillis=0; 
unsigned long currentMillis=millis();
int pwm_value=0;

int dipValue = 0; // Variable to hold the state of the DIPswitches
// Note that the default dipS value is zero. The first state in the array will be used here
int switch_toggled = 0; // Variable showing that switch was triggered

int ma_state = 0; // Variable to read the states of the motor control switches
int mb_state = 0;
int button_pressed = 0; // Flag to indicate button press interrupt
int button_pwm = 0; // Controls direction of rotation when button is pressed
int dir=1; //controls direction of motor


typedef struct {
  int16_t encoder_angles;
  bool ls1;
  bool ls2;
  float current;
} feedback_t;

feedback_t feedback={0};

void PCNT_Init();
float Get_Angles(volatile int enc_accumulator,int16_t enc_count);
static void IRAM_ATTR pcnt_intr_handler(void *arg);
static void IRAM_ATTR MAMB_handler(); // Unified motor control button handling

static void IRAM_ATTR dP_handler();

bool CAN_Init();
static void send_message();
static void handle_rx_message(twai_message_t &message);
void Handle_Errors(uint32_t alerts_triggered,twai_status_info_t twaistatus);

void setup() {
  Serial.begin(115200);
  Serial.println("code started");
  pinMode(INA, OUTPUT);
  pinMode(INB,OUTPUT);
  pinMode(PWM,OUTPUT);
  pinMode(MA,INPUT_PULLDOWN);
  pinMode(MB,INPUT_PULLDOWN);
  
  // Setting up all the DIP switches as INPUT pins
  for(int i = 0; i < 4; i++){
    pinMode(dipPins[i], INPUT_PULLDOWN);
  }
  
  // Attaching interrupts to all the dipPins to monitor the state of the switches
  for(int i = 0; i < 4; i++){
    attachInterrupt(digitalPinToInterrupt(dipPins[i]), dP_handler, CHANGE);
  }
  attachInterrupt(digitalPinToInterrupt(MA),MAMB_handler,CHANGE);
  attachInterrupt(digitalPinToInterrupt(MB),MAMB_handler,CHANGE);

  PCNT_Init();

  driver_installed = CAN_Init();

  Serial.println("setup done");
  // analogWriteFrequency(PWM, 10000);
  // analogWriteResolution(PWM,8);
  vTaskDelay(pdMS_TO_TICKS(100));

  dipValue = (digitalRead(dP0) * 1) + (digitalRead(dP1) * 2) + (digitalRead(dP2) * 4) ;
}

void loop() {
  currentMillis=millis();
  if (!driver_installed) {
    Serial.println("driver not installed");
    
    return;
  }
  
  pcnt_get_counter_value(PCNT_UNIT, &count);
  angles=Get_Angles(accumulator,count);
  Serial.printf("Count: %d, Accumulator: %d, Angle: %.2f \n", count, accumulator, angles);

  if (switch_toggled == 1) { // Reading the new config of the switches
      
      dipValue = (digitalRead(dP0) * 1) + (digitalRead(dP1) * 2) + (digitalRead(dP2) * 4) ;
      if (digitalRead(dP3)){
        dir=1;
        }
      else{
        dir=-1;
        }
      switch_toggled = 0; // Reset the interrupt flag to not force reads again
  }
    
  Serial.printf("DIP switch state is: %d \n", dipValue);
  
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
  twai_get_status_info(&twaistatus);
  Handle_Errors(alerts_triggered,twaistatus);

  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    twai_message_t message;
    
    while (twai_receive(&message, pdMS_TO_TICKS(0)) == ESP_OK) { //comes out after 10ms
      handle_rx_message(message);
      
      if (pwm_value>0){
      Serial.println("inside positive loop");
      digitalWrite(INB,LOW);
      digitalWrite(INA,HIGH);
      analogWrite(PWM,pwm_value);
      }
      else if(pwm_value<0){
      Serial.println("inside negative loop");
      digitalWrite(INB,HIGH);
      digitalWrite(INA,LOW);
      analogWrite(PWM,abs(pwm_value));
      }
      else{
      Serial.println("rest");
      digitalWrite(INB,LOW);
      digitalWrite(INA,LOW);
      analogWrite(PWM,0);
      }
    }
  }
  if(button_pressed == 1){
    ma_state = digitalRead(MA);
    mb_state = digitalRead(MB);

    if(ma_state == HIGH && mb_state == LOW){
      // button_pwm = pwm_multiplier[dipValue];
      button_pwm = 1*dir;
    }
    else if(ma_state == LOW && mb_state == HIGH){
      // button_pwm = -1 * pwm_multiplier[dipValue];
      button_pwm = -1*dir;
    }
    else if(ma_state == LOW && mb_state == LOW){
      button_pressed = 0;
      button_pwm = 0;
    }
    
    if(button_pwm > 0){
      digitalWrite(INB, LOW);
      digitalWrite(INA, HIGH);
      analogWrite(PWM, DEF_SPEED);
    }
    else if(button_pwm < 0){
      digitalWrite(INB, HIGH);
      digitalWrite(INA, LOW);
      analogWrite(PWM, DEF_SPEED);
    }
    else{
      digitalWrite(INB, LOW);
      digitalWrite(INA, LOW);
      analogWrite(PWM, 0);
    }
  }
  if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
    previousMillis = currentMillis;
    send_message();
  }
  vTaskDelay(pdMS_TO_TICKS(10));
  }

void PCNT_Init(){

  pcnt_config_t pcnt_config = {
  .pulse_gpio_num = ENCODER_GPIO_A,  
  .ctrl_gpio_num = ENCODER_GPIO_B, 
  .lctrl_mode = PCNT_MODE_REVERSE,  
  .hctrl_mode = PCNT_MODE_KEEP,     
  .pos_mode = PCNT_COUNT_INC,       
  .neg_mode = PCNT_COUNT_DEC,       
  .counter_h_lim = 32767,           
  .counter_l_lim = -32768,          
  .unit = PCNT_UNIT,                
  .channel = PCNT_CHANNEL_0         
  };

  pcnt_unit_config(&pcnt_config);

  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);

  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(PCNT_UNIT, pcnt_intr_handler , NULL);

  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);

}

float Get_Angles(volatile int enc_accumulator,int16_t enc_count){

    float angle=( enc_accumulator * PCNT_H_LIM + enc_count) * ENC_Calib * ENC_DIR;
    return angle;

}

static void IRAM_ATTR pcnt_intr_handler(void *arg) {
    uint32_t evt_status;
    pcnt_get_event_status(PCNT_UNIT, &evt_status);

    if (evt_status & PCNT_EVT_H_LIM) {
        accumulator ++;  
    }
    if (evt_status & PCNT_EVT_L_LIM) {
        accumulator --;  
    }

}



bool CAN_Init(){

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } 
  else {
    Serial.println("Failed to install driver");
    return false;
  }

    if (twai_start() == ESP_OK) {
      Serial.println("Driver started");
  } else {
      Serial.println("Failed to start driver");
      return false;
    }

  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_FAILED | TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS  ; //last 2 not used
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
      Serial.println("CAN Alerts reconfigured");
  } else {
      Serial.println("Failed to reconfigure alerts");
      return false;
  }

  return true;

}

static void send_message() {
  // Send message

  // Configure message to transmit
  Serial.println("sent");
  twai_message_t message={0};
  message.extd=0;
  message.identifier = 0x17+dipValue; // identifier based on dip switch value
  message.data_length_code = 8;
  feedback.encoder_angles=angles;
  feedback.current=random(0,10); //random values being sent as code not being integrated
  feedback.ls1=random(0,2);
  feedback.ls2=random(0,2);
  memcpy(&message.data,&feedback,sizeof(feedback_t));

  // Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(0)) == ESP_OK) {
    printf("Message queued for transmission\n");
  } else {
    printf("Failed to queue message for transmission\n");
  }

}

static void handle_rx_message(twai_message_t &message) {
  Serial.printf("ID: %lx\nByte:", message.identifier);
  if (message.identifier==0x01){   // default pwm message id
/*    for (int i = 0; i < message.data_length_code; i++) {
      pwm_value=(message.data[i]-127.5)*2*dir;
      Serial.print("Pwm: ");
      Serial.println(pwm_value);
      }
      Serial.println();*/
    pwm_value=(message.data[dipValue]-127.5)*2*dir;
    if (pwm_value<=2 & pwm_value>=-2){
      pwm_value=0;
    }
    Serial.print("Pwm: ");
    Serial.println(pwm_value);
    // pwm_value=(message.data[dipValue]);
    // Serial.print("Pwm: ");
    // Serial.println(pwm_value);
  }
}

void Handle_Errors(uint32_t alerts_triggered,twai_status_info_t twaistatus){

  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
      Serial.println("Alert: TWAI controller has become error passive.");
  }
  
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      Serial.println("Alert: A bus error has occurred.");
      Serial.printf("Bus error count: %lu\n", twaistatus.bus_error_count);
     
      
  }
  
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
      Serial.printf("RX buffered: %lu\t", twaistatus.msgs_to_rx);
      Serial.printf("RX missed: %lu\t", twaistatus.rx_missed_count);
      Serial.printf("RX overrun %lu\n", twaistatus.rx_overrun_count);
  }

  if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
    Serial.println("Alert: The Transmission failed.");
    Serial.printf("TX buffered: %lu\t", twaistatus.msgs_to_tx);
    Serial.printf("TX error: %lu\t", twaistatus.tx_error_counter);
    Serial.printf("TX failed: %lu\n", twaistatus.tx_failed_count);
  }

}

static void IRAM_ATTR MAMB_handler(){
  button_pressed = 1;
}

// Interrupt routine sets the flag indicating the switch was toggled and will be processed in the main loop
static void IRAM_ATTR dP_handler(){
  switch_toggled = 1;
}
