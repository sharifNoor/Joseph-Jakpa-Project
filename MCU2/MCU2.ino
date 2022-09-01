/*******************************************************  Including Libraies  ***********************************************************/
#include <Arduino.h>
#include <driver/adc.h>
#include <FunctionalInterrupt.h>
#include <Servo.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <LoRa.h>
#include <SPI.h>
/*******************************************************     Defining Pins    ***********************************************************/
#define BUTTON_PIN_BITMASK 0x200000000  // 2^33 in hex
#define laserDiode 27                   // D27 Pin ESP32 
#define tiltSensor 4                    // D4 Pin ESP32
#define servoPin 13                     // D13 Pin ESP32
#define ss 5                            // LoRa NSS pin
#define rst 14                          // LoRa Reset pin
#define dio0 2                          // LoRa Digital I/O 0
#define RXD2 17
#define TXD2 16

TinyGPSPlus gps;
Servo servoMotor;                       // create servo object to control a servo.
/*******************************************************  Defining Veriables  ***********************************************************/
int pos = 0;    // variable to store the servo position.
String outgoing;              // outgoing message
byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xCC;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends
byte Latitude;
byte Longitude;
byte BatteryLevel;
byte ActualMsg;
int recipient;          // recipient address
byte sender;            // sender address
byte incomingMsgId;     // incoming msg ID
byte incomingLength;    // incoming msg length
String incoming;
byte PushBTN1;
byte PushBTN2;

RTC_DATA_ATTR int bootCount = 0;
/*******************************************************       Functions      ***********************************************************/

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

//                                                              LoRa Send Message
void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.write(Latitude);
  LoRa.write(Longitude);
  LoRa.write(BatteryLevel);
  LoRa.write(ActualMsg);
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

//                                                              LoRa Receive Message
void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  recipient = LoRa.read();          // recipient address
  sender = LoRa.read();            // sender address
  incomingMsgId = LoRa.read();     // incoming msg ID
  incomingLength = LoRa.read();    // incoming msg length
  PushBTN1 = LoRa.read();
  PushBTN2 = LoRa.read();
  incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
//  Serial.println("Received from: 0x" + String(sender, HEX));
//  Serial.println("Sent to: 0x" + String(recipient, HEX));
//  Serial.println("Message ID: " + String(incomingMsgId));
//  Serial.println("Message length: " + String(incomingLength));
//  Serial.println("Message: " + incoming);
//  Serial.println("RSSI: " + String(LoRa.packetRssi()));
//  Serial.println("Snr: " + String(LoRa.packetSnr()));
//  Serial.println();

  if(PushBTN1){
    servoMotor.write(pos); // Code for moving servo to the pre defined position.
  }

  if(PushBTN2){
    digitalWrite(laserDiode, HIGH); // Code for Highing tha Laser Diode.
  }
  
}

void tilt_sensor() {
  Serial.println("Tilt Sensor value changed...!");
  // Will publish MQTT message from here on Tilt Sensor Value change.
}

class Button
{
public:
  Button(uint8_t reqPin) : PIN(reqPin){
    pinMode(PIN, INPUT_PULLUP);
    attachInterrupt(PIN, std::bind(&Button::isr,this), FALLING);
  };
  ~Button() {
    detachInterrupt(PIN);
  }

  void IRAM_ATTR isr() {
    numberKeyPresses += 1;
    pressed = true;
  }

  void checkPressed() {
    if (pressed) {
      Serial.printf("Button on pin %u has been pressed %u times\n", PIN, numberKeyPresses);
      pressed = false;
      if (PIN == tiltSensor){
        tilt_sensor();
      }
    }
  }

private:
  const uint8_t PIN;
    volatile uint32_t numberKeyPresses;
    volatile bool pressed;
};

Button tiltSensorBtn(tiltSensor);

//                                                               Battery Percentage Function
float battery_read()
{
    //read battery voltage per %
    long sum = 0;                  // sum of samples taken
    float voltage = 0.0;           // calculated voltage
    float output = 0.0;            //output value
    const float battery_max = 4.0; //maximum voltage of battery
    const float battery_min = 2.5; //minimum voltage of battery before shutdown

    float R1 = 100000.0; // resistance of R1 (100K)
    float R2 = 10000.0;  // resistance of R2 (10K)

    for (int i = 0; i < 500; i++)
    {
        sum += adc1_get_voltage(ADC1_CHANNEL_0); // ESP32 GPIO-36 or D36
        delayMicroseconds(1000);
    }
    // calculate the voltage
    voltage = sum / (float)500;
    voltage = (voltage * 1.1) / 4096.0; //for internal 1.1v reference
    // use if added divider circuit
    // voltage = voltage / (R2/(R1+R2));
    //round value by two precision
    voltage = roundf(voltage * 100) / 100;
//    Serial.print("voltage: ");
//    Serial.println(voltage, 2);
    output = ((voltage - battery_min) / (battery_max - battery_min)) * 100;
    if (output < 100)
        return output;
    else
        return 100.0f;
}

/*******************************************************         Setup        ***********************************************************/
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1); //gps baud
  while (!Serial);
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1); //1 = High, 0 = Low
  
  // Battery Percentage Code Init.
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_0db); //set reference voltage to internal
  //                                                LoRa Pin Initialization
  LoRa.setPins(ss, rst, dio0);    //setup LoRa transceiver module
 //                                                 LoRa Band Defination
  while (!LoRa.begin(433E6))     //433E6 - Asia, 866E6 - Europe, 915E6 - North America
  {
    Serial.println(".");
    delay(500);
  }
//  LoRa.setSyncWord(0xA5);
  Serial.println("LoRa Initializing OK!");
  //                                                Servo Motor Pin defination
  servoMotor.attach(servoPin);  // attaches the servo on pin 13 to the servo object.
}

/*******************************************************          Loop         ***********************************************************/
void loop() {
  tiltSensorBtn.checkPressed();
  // Will get data from MCU-1 and move the servo and operate the Laser Diode accordingly here.

  // Reading Battery Percentage
  Serial.println("Battery Level: " + String(battery_read(), 2));
//  Serial.println(battery_read(), 2);
  BatteryLevel = battery_read(), 2;
  


  // Getting GPS Cordinates...!
  bool recebido = false;
  while (Serial1.available()) {
    char cIn = Serial1.read();
    recebido = gps.encode(cIn);
    }
  if (gps.location.isUpdated() && gps.altitude.isUpdated())
  {
//    Serial.print("D/M/A: ");
//    Serial.print(gps.date.value());
//    Serial.print(" | alt: ");
//    Serial.print(gps.altitude.feet());
//    Serial.print(" | satellites: ");
//    Serial.println(gps.satellites.value());
//    Serial.print(" LATTITUDE: ");
    Serial.println(gps.location.lat());
    Latitude = gps.location.lat();
    Serial.print(" LOGITUDE: ");
    Serial.println(gps.location.lng());
    Longitude = gps.location.lng();
  }

//------------------------------------------------------------------ Code for Sending data to MCU-1 --------------------------------------------------------------// 
  String message = String(Latitude) + String(Longitude) + String(BatteryLevel) + String(ActualMsg);
  sendMessage(message);
//------------------------------------------------------------------------------- End ----------------------------------------------------------------------------// 

//---------------------------------------------------------------- Code for Receiving data From MCU-1 ------------------------------------------------------------//
  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
//------------------------------------------------------------------------------- End ----------------------------------------------------------------------------// 
  
  delay(1000);

  // Setting to Deep Sleep mode...!
  esp_deep_sleep_start();
}
