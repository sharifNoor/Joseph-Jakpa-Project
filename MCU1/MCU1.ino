/*******************************************************  Including   Libraies  ***********************************************************/
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <driver/adc.h>
#include <FunctionalInterrupt.h>
#include <LoRa.h>
#include <SPI.h>
/*******************************************************     Defining Pins    ***********************************************************/
#define photoresistor 13  // D13 Pin ESP32 
#define tiltSensor 17     // D17 Pin ESP32
#define pushBtn1 15       // D15 Pin ESP32
#define pushBtn2 12       // D12 Pin ESP32
#define ss 5              // LoRa NSS pin
#define rst 14            // LoRa Reset pin
#define dio0 2            // LoRa Digital I/O 0

WiFiClient espClient;
PubSubClient client(espClient);
/*******************************************************  Defining Veriables  ***********************************************************/
int photoresistorValue;
int pushBtn1Value;
int pushBtn2Value;
int counter = 0;
String outgoing;              // outgoing message
byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xFF;     // address of this device
byte destination = 0xBB;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends
int recipient;          // recipient address
byte sender;            // sender address
byte incomingMsgId;     // incoming msg ID
byte incomingLength;    // incoming msg length
String incoming;
byte PushBTN1 = 0;
byte PushBTN2 = 0;
byte Latitude;
byte Longitude;
byte BatteryLevel_MCU2;
byte BatteryLevel_MCU1;
int TiltSensorMCU1;
long lastMsg = 0;
char msg[50];
int value = 0;
byte ActualMsg;
// Replace the next variables with your SSID/Password combination
const char* ssid = "REPLACE_WITH_YOUR_SSID";
const char* password = "REPLACE_WITH_YOUR_PASSWORD";
// Replace with your MQTT Broker Address
const char* mqtt_server = "YOUR_MQTT_BROKER_ADDRESS";


/*******************************************************         Class        ***********************************************************/

bool tilt_sensor() {
  Serial.println("Tilt Sensor value changed...!");
  // Will publish MQTT message from here on Tilt Sensor Value change
  TiltSensorMCU1 = 1;
  return true;
}

bool push_btn_1(){
  Serial.println("Push Button 1 Pressed...!");
  // Will send message via Lora to MCU-2 for Servo Reset
  PushBTN1 = 1;
}

bool push_btn_2(){
  Serial.println("Push Button 2 Pressed...!");
  // Will send message via Lora to MCU-2 for Activating Laser Diode
  PushBTN2 = 1;
}

//                                                              LoRa Send Message
void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.write(PushBTN1);
  LoRa.write(PushBTN2);
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
  Latitude = LoRa.read();
  Longitude = LoRa.read();
  BatteryLevel_MCU2 = LoRa.read();
  ActualMsg = LoRa.read();
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

  char LatitudeString[8];
  char LongitudeString[8];
  char BatteryLevel_MCU2String[8];
  char ActualMsgString[8];
  char BatteryLevel_MCU1String[8];
  char TiltSensorMCU1String[8];
  char photoresistorValueString[8];

  dtostrf(Latitude, 1, 2, LatitudeString);
  dtostrf(Longitude, 1, 2, LongitudeString);
  dtostrf(BatteryLevel_MCU2, 1, 2, BatteryLevel_MCU2String);
  dtostrf(ActualMsg, 1, 2, ActualMsgString);
  dtostrf(BatteryLevel_MCU1, 1, 2, BatteryLevel_MCU1String);
  dtostrf(TiltSensorMCU1, 1, 2, TiltSensorMCU1String);
  dtostrf(photoresistorValue, 1, 2, photoresistorValueString);

  client.publish("esp32/Latitude", LatitudeString);
  client.publish("esp32/Longitude", LongitudeString);
  client.publish("esp32/BatteryLevel_MCU2", BatteryLevel_MCU2String);
  client.publish("esp32/ActualMsg", ActualMsgString);
  client.publish("esp32/BatteryLevel_MCU1", BatteryLevel_MCU1String);
  client.publish("esp32/TiltSensorMCU1", TiltSensorMCU1String);
  client.publish("esp32/photoresistorValue", photoresistorValueString);
  
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

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
      else if(PIN == pushBtn1){
        push_btn_1();
      }
      else if(PIN == pushBtn2){
        push_btn_2();
      }
    }
  }

private:
  const uint8_t PIN;
    volatile uint32_t numberKeyPresses;
    volatile bool pressed;
};

Button tiltSensorBtn(tiltSensor);
Button push_Btn_one(pushBtn1);
Button push_Btn_two(pushBtn2);

/*******************************************************         Setup        ***********************************************************/
void setup() {
  Serial.begin(115200);
  while (!Serial);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
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
  
  pinMode(photoresistor, INPUT);
  pinMode(pushBtn1, INPUT);
  pinMode(pushBtn2, INPUT);
}

/*******************************************************          Loop         ***********************************************************/
void loop() {
  // read the value from the photoresistor sensor
  photoresistorValue = analogRead(photoresistor);
  tiltSensorBtn.checkPressed();
  push_Btn_one.checkPressed();
  push_Btn_two.checkPressed();

  // Reading Battery Percentage
  Serial.println("Battery Level: " + String(battery_read(), 2));
//  Serial.println(battery_read(), 2);
  BatteryLevel_MCU1 = battery_read(), 2;

  
  
  if (photoresistorValue > 350){
    // Will publish MQTT message from here on Photoresistor Threshold drop
  }

  if (!client.connected()) {
      reconnect();
  }
  client.loop();

//------------------------------------------------------------------ Code for Sending data to MCU-1 --------------------------------------------------------------// 
  String message = String(Latitude) + String(Longitude) + String(BatteryLevel_MCU2) + String(ActualMsg);
  sendMessage(message);
//------------------------------------------------------------------------------- End ----------------------------------------------------------------------------// 

//---------------------------------------------------------------- Code for Receiving data From MCU-1 ------------------------------------------------------------//
  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
//------------------------------------------------------------------------------- End ----------------------------------------------------------------------------// 
  
  delay(1000);
}
