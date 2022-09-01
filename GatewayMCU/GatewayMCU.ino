/*******************************************************  Including   Libraies  ***********************************************************/
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <LoRa.h>
#include <SPI.h>
/*******************************************************     Defining Pins    ***********************************************************/
#define ss 5              // LoRa NSS pin
#define rst 14            // LoRa Reset pin
#define dio0 2            // LoRa Digital I/O 0

WiFiClient espClient;
PubSubClient client(espClient);
/*******************************************************  Defining Veriables  ***********************************************************/
byte MasterNode = 0xFF;
byte Node1 = 0xBB;          // MCU-1
byte Node2 = 0xCC;          // MCU-2
String SenderNode = "";
String outgoing;              // outgoing message
byte msgCount = 0;            // count of outgoing messages
String incoming = "";
// Tracks the time since last event fired
unsigned long previousMillis = 0;
unsigned long int previoussecs = 0;
unsigned long int currentsecs = 0;
unsigned long currentMillis = 0;
int interval = 1 ; // updated every 1 second
int Secs = 0;
String photoresistorValue;
byte PushBTN1;
byte PushBTN2;
byte Latitude;
byte Longitude;
byte BatteryLevel_MCU2;
byte BatteryLevel_MCU1;
int TiltSensorMCU1;
// Replace the next variables with your SSID/Password combination
const char* ssid = "REPLACE_WITH_YOUR_SSID";
const char* password = "REPLACE_WITH_YOUR_PASSWORD";
// Replace with your MQTT Broker Address
const char* mqtt_server = "YOUR_MQTT_BROKER_ADDRESS";
/*******************************************************   Class & Functions   ***********************************************************/
//                                                          Get Value Function
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
//                                                              LoRa Send Message
void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(Node2);              // add destination address
  LoRa.write(MasterNode);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.write(PushBTN1);
  LoRa.write(PushBTN2);
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID 
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  if ( sender == 0XBB )
    SenderNode = "MCU1:";
  if ( sender == 0XCC )
    SenderNode = "MCU2:";
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length


  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    //Serial.println("error: message length does not match length");
    ;
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != Node1 && recipient != MasterNode) {
    // Serial.println("This message is not for me.");
    ;
    return;                             // skip rest of function
  }
  if ( sender == 0XCC ) // from MCU-2
  {
    String BatteryLevel_MCU2 = getValue(incoming, ',', 0); // BatteryLevel_MCU2
    String Latitude = getValue(incoming, ',', 1); // Latitude
    String Longitude = getValue(incoming, ',', 2); // Longitude
    incoming = "";

    char LatitudeString[8];
    char LongitudeString[8];
    char BatteryLevel_MCU2String[8];

    int Latitude_len = Latitude.length() + 1; 
    int Longitude_len = Longitude.length() + 1; 
    int BatteryLevel_MCU2_len = BatteryLevel_MCU2.length() + 1; 

    Latitude.toCharArray(LatitudeString, Latitude_len);
    Longitude.toCharArray(LongitudeString, Longitude_len);
    BatteryLevel_MCU2.toCharArray(BatteryLevel_MCU2String, BatteryLevel_MCU2_len);

    client.publish("esp32/Latitude", LatitudeString);
    client.publish("esp32/Longitude", LongitudeString);
    client.publish("esp32/BatteryLevel_MCU2", BatteryLevel_MCU2String);
  }

  if ( sender == 0XBB )
  {
    String BatteryLevel_MCU1 = getValue(incoming, ',', 0); // BatteryLevel_MCU1
    String PushBTN1 = getValue(incoming, ',', 1); // PushBTN1
    String PushBTN2 = getValue(incoming, ',', 2); // PushBTN2
    if(photoresistorValue) {
      photoresistorValue = getValue(incoming, ',', 3); // photoresistorValue
    }
    else {
      photoresistorValue = "0";
    }
    incoming = "";
//------------------------------------------------------------------ Code for Sending data to MCU-1 --------------------------------------------------------------// 
    String message = PushBTN1 + PushBTN2;
    sendMessage(message);
//------------------------------------------------------------------------------- End ----------------------------------------------------------------------------// 
    
    char photoresistorValueString[8];
    char BatteryLevel_MCU1String[8];

    int photoresistorValue_len = photoresistorValue.length() + 1; 
    int BatteryLevel_MCU1_len = BatteryLevel_MCU1.length() + 1; 

    photoresistorValue.toCharArray(photoresistorValueString, photoresistorValue_len);
    BatteryLevel_MCU1.toCharArray(BatteryLevel_MCU1String, BatteryLevel_MCU1_len);
    
    client.publish("esp32/photoresistorValue", photoresistorValueString);
    client.publish("esp32/BatteryLevel_MCU1", BatteryLevel_MCU1String);
  }
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


/*******************************************************         Setup        ***********************************************************/
void setup() {
  Serial.begin(115200);
  while (!Serial);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

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
}
/*******************************************************          Loop         ***********************************************************/
void loop() {

//---------------------------------------------------------------- Code for Receiving data From MCU-1 ------------------------------------------------------------//
  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
//------------------------------------------------------------------------------- End ----------------------------------------------------------------------------// 
//------------------------------------------------------------------ Code for Sending data to MCU-1 --------------------------------------------------------------// 
//  currentMillis = millis();
//  currentsecs = currentMillis / 1000;
//  if ((unsigned long)(currentsecs - previoussecs) >= interval) {
//    Secs = Secs + 1;
//    //Serial.println(Secs);
//    if ( Secs >= 11 )
//    {
//      Secs = 0;
//    }
//    if ( (Secs >= 1) && (Secs <= 5) )
//    {
//
//      String message = "10";
//      sendMessage(message, MasterNode, Node1);
//    }
//
//    if ( (Secs >= 6 ) && (Secs <= 10))
//    {
//
//      String message = "20";
//      sendMessage(message, MasterNode, Node2);
//    }
//
//    previoussecs = currentsecs;
//  }
//------------------------------------------------------------------------------- End ----------------------------------------------------------------------------//  
  
  if (!client.connected()) {
      reconnect();
  }
  client.loop();
  delay(1000);
}
