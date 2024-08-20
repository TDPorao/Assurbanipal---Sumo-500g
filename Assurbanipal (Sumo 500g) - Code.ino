#include <IRremote.h>

#define DECODE_SONY

#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
    
///////////////////////////////////////////COMPONENTS////////////////////////////////////////////////////////////////////////

//MOTORS
#define PWM1 23
#define PWM2 19
#define AIN1 3
#define AIN2 1
#define BIN1 5
#define BIN2 18

//PORTS
#define IR_RECEIVE_PIN 36

int IRSensor1 = 26; // INFRARED PORTS
int IRSensor2 = 39;
int IRSensor3 = 14;

int LSensor1 = 20;
int LSensor2 = 21;

///////////////////////////////////////////VARIABLES/////////////////////////////////////////////////////////////////////////
bool ON; // Robot Status

int IRV1; // INFRARED VALUES
int IRV2;
int IRV3;

int LSV1;
int LSV2;

int Strategy = 0;

String device_name = "Assurbanipal";

///////////////////////////////////////////SETUP/LOOP////////////////////////////////////////////////////////////////////////

void setMotorDirection(int AIN1_state, int AIN2_state, int BIN1_state, int BIN2_state, int PWM1_value = 0, int PWM2_value = 0) {
  digitalWrite(AIN1, AIN1_state);
  digitalWrite(AIN2, AIN2_state);
  digitalWrite(BIN1, BIN1_state);
  digitalWrite(BIN2, BIN2_state);
  ledcWrite(1, PWM1_value);
  ledcWrite(2, PWM2_value);
}

void setup() {
  Serial.begin(115200); // Serial communication started with 115200 bits per second.
  SerialBT.begin(device_name); //Bluetooth device name
  setMotorDirection(LOW, LOW, LOW, LOW, 0, 0); // Define os pinos e velocidades iniciais dos motores

  // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  pinMode(IRSensor1, INPUT); // ir sensors ports are declared as input
  pinMode(IRSensor2, INPUT);
  pinMode(IRSensor3, INPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  
  ledcAttachPin(PWM1, 1); // Canal 1 para PWM1
  ledcAttachPin(PWM2, 2); // Canal 2 para PWM2
  ledcSetup(1, 5000, 8);  // 5000 Hz, resolução de 8 bits para o canal 1
  ledcSetup(2, 5000, 8);  // 5000 Hz, resolução de 8 bits para o canal 2
}

void loop() {
  MainFunction();  // Função do robô quando está ligado

  IRV1 = digitalRead(IRSensor1); // Leitura dos sensores IR
  IRV2 = digitalRead(IRSensor2);
  IRV3 = digitalRead(IRSensor3);

  LSV1 = digitalRead(LSensor1);
  LSV2 = digitalRead(LSensor2);

  if (IrReceiver.decode()) {  // Decodifica o sinal digital recebido do controle

    if (IrReceiver.decodedIRData.command == 0x1) {
      ON = true; // Robô é ligado quando o botão 2 é pressionado
    } else if (IrReceiver.decodedIRData.command == 0x2) {
      ON = false;
            setMotorDirection(HIGH, HIGH, HIGH, HIGH, 0, 0);
    }
    IrReceiver.resume(); // Habilita a recepção do próximo valor
  }}

///////////////////////////////////////////FUNCTIONS/////////////////////////////////////////////////////////////////////////

void MainFunction() {
  if (ON == true) {
    if(Strategy == 1){
    // Verificações dos sensores IR
      if (IRV1 == 1 && IRV2 == 0 && IRV3 == 0) {
        setMotorDirection(HIGH, LOW, LOW, LOW, 180, 0);
      } if (IRV1 == 0 && IRV2 == 1 && IRV3 == 0) {
        setMotorDirection(HIGH, LOW, HIGH, LOW, 180, 180);
      } if (IRV1 == 1 && IRV2 == 1 && IRV3 == 0) {
        setMotorDirection(HIGH, LOW, HIGH, LOW, 180, 180);
      } if (IRV1 == 0 && IRV2 == 1 && IRV3 == 1) {
        setMotorDirection(HIGH, LOW, HIGH, LOW, 180, 180);
      } if (IRV1 == 0 && IRV2 == 0 && IRV3 == 1) {
        setMotorDirection(HIGH, HIGH, HIGH, LOW, 0, 180);}
        if (IRV1 == 1 && IRV2 == 1 && IRV3 == 1) {
        setMotorDirection(HIGH, LOW, HIGH, LOW, 180, 180);
      } if(IRV1 == 0 && IRV2 == 0 && IRV3 == 0){
        setMotorDirection(LOW, HIGH, HIGH, LOW, 60, 60);
      }
    if(Strategy == 2){
    // Verificações dos sensores IR
      if (IRV1 == 1 && IRV2 == 0 && IRV3 == 0) {
        setMotorDirection(HIGH, LOW, LOW, LOW, 180, 0);
      } if (IRV1 == 0 && IRV2 == 1 && IRV3 == 0) {
        setMotorDirection(HIGH, LOW, HIGH, LOW, 180, 180);
      } if (IRV1 == 1 && IRV2 == 1 && IRV3 == 0) {
        setMotorDirection(HIGH, LOW, HIGH, LOW, 180, 180);
      } if (IRV1 == 0 && IRV2 == 1 && IRV3 == 1) {
        setMotorDirection(HIGH, LOW, HIGH, LOW, 180, 180);
      } if (IRV1 == 0 && IRV2 == 0 && IRV3 == 1) {
        setMotorDirection(HIGH, HIGH, HIGH, LOW, 0, 180);}
        if (IRV1 == 1 && IRV2 == 1 && IRV3 == 1) {
        setMotorDirection(HIGH, LOW, HIGH, LOW, 180, 180);
      } if(IRV1 == 0 && IRV2 == 0 && IRV3 == 0){
        setMotorDirection(LOW, HIGH, HIGH, LOW, 60, 60);
      }
    }
    if(Strategy == 3){
    // Verificações dos sensores IR
      if (IRV1 == 1 && IRV2 == 0 && IRV3 == 0) {
        setMotorDirection(HIGH, LOW, LOW, LOW, 180, 0);
      } if (IRV1 == 0 && IRV2 == 1 && IRV3 == 0) {
        setMotorDirection(HIGH, LOW, HIGH, LOW, 180, 180);
      } if (IRV1 == 1 && IRV2 == 1 && IRV3 == 0) {
        setMotorDirection(HIGH, LOW, HIGH, LOW, 180, 180);
      } if (IRV1 == 0 && IRV2 == 1 && IRV3 == 1) {
        setMotorDirection(HIGH, LOW, HIGH, LOW, 180, 180);
      } if (IRV1 == 0 && IRV2 == 0 && IRV3 == 1) {
        setMotorDirection(HIGH, HIGH, HIGH, LOW, 0, 180);}
        if (IRV1 == 1 && IRV2 == 1 && IRV3 == 1) {
        setMotorDirection(HIGH, LOW, HIGH, LOW, 180, 180);
      } if(IRV1 == 0 && IRV2 == 0 && IRV3 == 0){
        setMotorDirection(LOW, HIGH, HIGH, LOW, 60, 60);
      }
  }
    if(Strategy == 4){
    // Verificações dos sensores IR
      if (IRV1 == 1 && IRV2 == 0 && IRV3 == 0) {
        setMotorDirection(HIGH, LOW, LOW, LOW, 180, 0);
      } if (IRV1 == 0 && IRV2 == 1 && IRV3 == 0) {
        setMotorDirection(HIGH, LOW, HIGH, LOW, 180, 180);
      } if (IRV1 == 1 && IRV2 == 1 && IRV3 == 0) {
        setMotorDirection(HIGH, LOW, HIGH, LOW, 180, 180);
      } if (IRV1 == 0 && IRV2 == 1 && IRV3 == 1) {
        setMotorDirection(HIGH, LOW, HIGH, LOW, 180, 180);
      } if (IRV1 == 0 && IRV2 == 0 && IRV3 == 1) {
        setMotorDirection(HIGH, HIGH, HIGH, LOW, 0, 180);}
        if (IRV1 == 1 && IRV2 == 1 && IRV3 == 1) {
        setMotorDirection(HIGH, LOW, HIGH, LOW, 180, 180);
      } if(IRV1 == 0 && IRV2 == 0 && IRV3 == 0){
        setMotorDirection(LOW, HIGH, HIGH, LOW, 60, 60);
      }  
  if(ON==false){
      setMotorDirection(LOW, LOW, LOW, LOW, 0, 0);
  }}
