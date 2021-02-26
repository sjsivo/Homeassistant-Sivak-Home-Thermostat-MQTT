

//Include additional libraries

//#include "DHT.h"//Arduino - Temperature & humidity sensor - DHT22
//#include "DHTesp.h"//ESP 32 - Temperature & humidity sensor - DHT22
#include <DS18B20.h>

DS18B20 ds(17);
#define LED_BUILTIN 2
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>//MQTT protocol between ESP32 & Raspberry Pi

//Wifi
const char* ssid     = "Yournetworkname";
const char* password = "Yournetworkpassword";

WiFiServer server(80);//webserver used to exchange information

//MQTT
const char* mqtt_server = "192.168.1.42";//Local IP of your raspberry
WiFiClient espClient;
PubSubClient client(espClient);

// Setpoint values
float heating_setpoint = 20.0;
float night_setpoint = 15.0;
float anti_freezing = 8.0;
float boost_value = 1.5;
float deadband = 2.0;
float setpoint = 0.0;
float trigger_point = 99.0;

// Operating mode
boolean heating = false;
boolean Auto = true;
boolean heating_period = true;
boolean boost = false;
boolean venting = false;//ventilation
boolean vent_forced = false;//ventilation
boolean vent_auto = true;//ventilation

// led of the card
boolean led = false;

// Push button
boolean Last_pushbutton_auto = false;//state of the BP_Manu button at the last cycle. Used to detect a rising edge.
boolean Last_pushbutton_boost = false;//state of the BP_boost button at the last cycle. Used to detect a rising edge.
boolean Last_pushbutton_vent = false;//state of the BP_vent button at the last cycle. Used to detect a rising edge.

// Cycle time
int timecycle = 1000;//forced time of cycle
unsigned long msLastCycle;//time at the end of the last cycle
unsigned long actualTime;//time of cycle before correction

// Pin use
// Arduino - Avoid the use of digital Pin 0 and 1. There are used for USB communication.
// ESP32 - Avoid the use of Pin 2. It's used for the blue led.
//#define DHTPIN 22//Arduino & ESP32 - Temperature & humidity module
const int heating_relay = 16;//pin of heating relay
const int venting_relay = 17;//pin of venting relay
const int Pushbutton_auto = 33;//pin of the auto pushbutton
const int Pushbutton_boost = 32;//pin of the boost pushbutton
const int Pushbutton_vent = 35;//pin of the boost pushbutton
//#define DHTTYPE DHT22//Arduino - Temperature & humidity module type - DHT 22(AM2302)
// DHT dht(DHTPIN, DHTTYPE); //Arduino
//DHTesp dht;// ESP32

// initialization / setup
void setup()
{
    //Serial port
  Serial.begin(115200);//baudrate

  //pin & DHT sensor
  pinMode(LED_BUILTIN, OUTPUT);// built in led (on D2 for ESP32, on D13 for Arduino Uno)
  pinMode(heating_relay, OUTPUT);
  digitalWrite(heating_relay, LOW);//There are two types of arduino relay. If your relay need an high input to be in its default position, you need to write "HIGH". If it's not the case, write "LOW"
  pinMode(venting_relay, OUTPUT);
  digitalWrite(venting_relay, LOW);
  pinMode(Pushbutton_auto, INPUT_PULLUP);// set the digital pin as input
  pinMode(Pushbutton_boost, INPUT_PULLUP);// set the digital pin as input
  pinMode(Pushbutton_vent, INPUT_PULLUP);// sets the digital pin as input
  //Wire.begin();
 // dht.setup(DHTPIN, DHTesp::DHT22);//ESP32
Serial.print("Devices: ");
  Serial.println(ds.getNumberOfDevices());
  Serial.println();
    while (ds.selectNext()) {
    switch (ds.getFamilyCode()) {
      case MODEL_DS18S20:
        Serial.println("Model: DS18S20/DS1820");
        break;
      case MODEL_DS1822:
        Serial.println("Model: DS1822");
        break;
      case MODEL_DS18B20:
        Serial.println("Model: DS18B20");
        break;
      default:
        Serial.println("Unrecognized Device");
        break;
    }

    uint8_t address[8];
    ds.getAddress(address);

    Serial.print("Address:");
    for (uint8_t i = 0; i < 8; i++) {
      Serial.print(" ");
      Serial.print(address[i]);
    }
    Serial.println();

    Serial.print("Resolution: ");
    Serial.println(ds.getResolution());

    Serial.print("Power Mode: ");
    if (ds.getPowerMode()) {
      Serial.println("External");
    } else {
      Serial.println("Parasite");
    }
    }


  //Wifi & MQTT
  delay(10);//waiting in ms
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  Serial.println("Started"); //write a message for Serial Monitor
}



// infinite loop of the program
void loop()
{
  //Temperature & humidity reading
  //float humidity = dht.readHumidity();//Arduino - read humidity
  //float temperature = dht.readTemperature() + 20; //Arduino - read temperature
  //float humidity = dht.getHumidity();//ESP32 - read humidity
  float temperature = ds.getTempC(); //ESP32 - read temperature
   Serial.print("Temperature: ");
   // Serial.print();
    Serial.print(" C / ");
    Serial.print(ds.getTempF());
    Serial.println(" F");
    Serial.println();
  // Check if any reads failed and exit early (to try again).
  if (isnan(temperature))
  {
    Serial.println("Failed to read from DS sensor!");
    return;
  }
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("*C ");

  //Connexion checking
  if (!client.connected())//test if we loose the connexion
  {
    reconnect();
  }

  //read subscribed topics --> see function "void reconnect()" for subscribing and function "callback (...)" for reading
  client.loop();

  //Button
  if ((digitalRead(Pushbutton_auto) == true) && (Last_pushbutton_auto == 0))
  {
    Last_pushbutton_auto = 1;
    if (Auto == 1)
    {
      Auto = 0;
    }
    else
    {
      Auto = 1;
    }
  }
  if ((digitalRead(Pushbutton_auto) == false) && (Last_pushbutton_auto == 1))
  {
    Last_pushbutton_auto = 0;
  }
  Serial.print("auto heating state : ");
  Serial.print(Auto);

  if ((digitalRead(Pushbutton_boost) == true) && (Last_pushbutton_boost == 0))
  {
    Last_pushbutton_boost = 1;
    boost = 1;
  }
  if ((digitalRead(Pushbutton_boost) == false) && (Last_pushbutton_boost == 1))
  {
    Last_pushbutton_boost = 0;
  }

  Serial.print(" | boost state : ");
  Serial.print(boost);

  if ((digitalRead(Pushbutton_vent) == true) && (Last_pushbutton_vent == 0))
  {
    Last_pushbutton_vent = 1;
    if (vent_auto == 1)
    {
      vent_auto = 0;
    }
    else
    {
      vent_auto = 1;
      vent_forced = 0;
    }
  }
  if ((digitalRead(Pushbutton_vent) == false) && (Last_pushbutton_vent == 1))
  {
    Last_pushbutton_vent = 0;
  }
  Serial.print("| auto venting state : ");
  Serial.print(vent_auto);
  Serial.println("");

  // stepoint calculation
  if (Auto == true)
  {
    if (heating_period == true)
    {
      if (boost == true)
      {
        setpoint = heating_setpoint + deadband / 2 + boost_value;
        trigger_point = heating_setpoint - deadband / 2;
      }
      else
      {
        setpoint = heating_setpoint + deadband / 2;
        trigger_point = heating_setpoint - deadband / 2;
      }
    }
    else
    {
      setpoint = night_setpoint + deadband / 2;
      trigger_point = night_setpoint - deadband / 2;
    }
  }
  else
  {
    setpoint = anti_freezing + deadband / 2;
    trigger_point = anti_freezing - deadband / 2;
  }

  Serial.print("heating_setpoint: ");
  Serial.print(heating_setpoint);
  Serial.print("*C | ");
  Serial.print("night_setpoint: ");
  Serial.print(night_setpoint);
  Serial.print("*C | ");
  Serial.print("anti_freezing: ");
  Serial.print(anti_freezing);
  Serial.print("*C | ");
  Serial.print("boost_value: ");
  Serial.print(boost_value);
  Serial.print("*C | ");
  Serial.print("deadband: ");
  Serial.print(deadband);
  Serial.println("*C | ");


  Serial.print("setpoint: ");
  Serial.print(setpoint);
  Serial.print("*C | ");
  Serial.print("trigger_point: ");
  Serial.print(trigger_point);
  Serial.println("*C ");

  // heating, venting, boost & led
  if (Auto == true)
  {
    if (((heating == false) && (temperature < trigger_point)) || (((heating == false) && (temperature < setpoint)) && (boost == true)))
    {
      heating = 1;//start heating
    }
    if ((heating == true) && (temperature > setpoint))
    {
      heating = 0;//stop heating
    }
  }
  else
  {
    heating = 0;//stop heating
  }

  if ((boost == true) && (temperature > setpoint))
  {
    boost = 0; //auto-reset boost
  }

  if ((heating == false) && (Auto == true))
  {
    led = 1;
  }

  if (Auto == false)
  {
    led = 0;
  }

  if (heating == true)//blink led while heating
  {
    if (led == true)
    {
      led = 0;
    }
    else
    {
      led = 1;
    }
  }

  //output relay and led
  if (heating == true)
  {
    digitalWrite(heating_relay, HIGH);//launch heating
    Serial.print("heating on | ");
  }
  else
  {
    digitalWrite(heating_relay, LOW);//stop heating
    Serial.print("heating off | ");
  }

  if ( (vent_forced == true) || ((heating == true) && (vent_auto == true)))
  {
    digitalWrite(venting_relay, HIGH);//start venting
    Serial.println("venting on");
    venting = 1;
  }
  else
  {
    digitalWrite(venting_relay, LOW);//stop venting
    Serial.println("venting off");
    venting = 0;
  }

  if (led == true)
  {
    digitalWrite(LED_BUILTIN, HIGH);//led
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }

  //Writing topics
 // String humidity_string = String(humidity);
//  client.publish("heating/humidity", (char*) humidity_string.c_str() );
  String temperature_string = String(temperature);
  client.publish("heating/temperature", (char*) temperature_string.c_str() );
  String stateToPi_string = String(heating) + String(venting) + String(Auto) + String(boost);
  client.publish("heating/stateToPi", (char*) stateToPi_string.c_str() );
  String setpointToPi_string = String(heating_setpoint) + String(night_setpoint) + String(anti_freezing) + String(boost_value) + String(deadband);
  client.publish("heating/setpointToPi", (char*) setpointToPi_string.c_str() );

  // Forcing the cycle time
  actualTime = millis() - msLastCycle;
  if (actualTime > 10000)// needed to avoid rollover bug
  {
    actualTime = 0;
  }
  if (actualTime < timecycle)
  {
    delay(timecycle - actualTime); //forcing
  }
  msLastCycle = millis();
}






void setup_wifi()
{
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)// Loop until we're reconnected
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("ESP32 IP : ");
  Serial.println(WiFi.localIP());// Print local IP address
}



void callback(char* topic, byte* message, unsigned int length) //read MQTT message
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(" --> Message: ");
  String messageTemp;
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println("");/*
  //  for (int i = 0; i < length; i++)//used for diagnostics
  //  {
  //    Serial.println(messageTemp[i]);
  //  }

  //Status
  if (((int)messageTemp[0] - '0') == 5) //the first number is at the position [3] //We used the first number to make a distinction between differents topics
  {
    Auto = (int)messageTemp[1] - '0';
    boost = (int)messageTemp[2] - '0';
    if (((int)messageTemp[3] - '0') == 2)
    {
      vent_forced = 1;
      vent_auto = 1;
    }
    else if (((int)messageTemp[3] - '0') == 1)
    {
      vent_forced = 0;
      vent_auto = 1;
    }
    else
    {
      vent_forced = 0;
      vent_auto = 0;
    }
  }

  //parameters
  if (((int)messageTemp[0] - '0') == 7)
  {
    heating_setpoint = ((((int)messageTemp[1] - '0') * 100) + (((int)messageTemp[2] - '0') * 10) + ((int)messageTemp[3] - '0')) / 10.0;
    night_setpoint = ((((int)messageTemp[4] - '0') * 100) + (((int)messageTemp[5] - '0') * 10) + ((int)messageTemp[6] - '0')) / 10.0;
    anti_freezing = ((((int)messageTemp[7] - '0') * 10) + ((int)messageTemp[8] - '0')) / 10.0;
    boost_value = ((((int)messageTemp[9] - '0') * 10) + ((int)messageTemp[10] - '0')) / 10.0;
    deadband = ((((int)messageTemp[11] - '0') * 10) + ((int)messageTemp[12] - '0')) / 10.0;
  }*/
}



void reconnect()//Connect to MQTT & Subscribe to MQTT topics
{
  while (!client.connected())// Loop until we're reconnected
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32SivakThermostat","homeassistant","yourpassword"))
    {
      Serial.println("connected");
      //Place here your subscribed topic
      client.subscribe("heating/parameters");
      client.subscribe("heating/stateToESP");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);// Wait 5 seconds before retrying
    }
  }
}
