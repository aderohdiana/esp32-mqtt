#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <MQ2.h>

#define ledPin 2
#define mqPin 34

float lpg, co, smoke;
MQ2 mq2(mqPin);

#define SEALEVELPRESSURE_HPA (1013.25) //nilai awal untuk pressure
Adafruit_BME280 bme; //penggunaan I2c

// Replace the next variables with your SSID/Password combination
const char* ssid = "WIFI";
const char* password = "password";
// Add your MQTT Broker IP address, example:
const char* mqtt_server = "test.com";
const char* mqtt_uname = "user";
const char* mqtt_passwd = "user.pass";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

// temp internal
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();
float tempInt = ((temprature_sens_read() - 32) / 1.8);

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  mq2.begin();
  pinMode(ledPin, OUTPUT);

  if (!bme.begin(0x76)) {
    Serial.println("tidak ada sensor BME280, Coba cek rangkaianmu!");
    while (1);
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
  Serial.println(topic);
  Serial.print("Message: ");
  String messageTemp;
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  // Feel free to add more if statements to control more GPIOs with MQTT
  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".\
  // Changes the output state according to the message
  if (String(topic) == "/topic1") {
    Serial.println("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if (messageTemp == "off") {
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
}
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("GMNClient", mqtt_uname, mqtt_passwd)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("/topic1");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void sendData() {
  float temp = bme.readTemperature();
  float hum = bme.readHumidity();
  float pressu = (bme.readPressure() / 100.0F);
  float* values = mq2.read(false); //set it false if you don't want to print the values to the Serial

  // lpg = values[0];
  lpg = mq2.readLPG();
  // co = values[1];
  co = mq2.readCO();
  // smoke = values[2];
  smoke = mq2.readSmoke();

  // Convert the value to a char array
  // char tempIntString[8];
  // dtostrf(tempInt, 1, 2, tempIntString);

  StaticJsonDocument<200> doc;
  doc["tempInt"] = tempInt;
  doc["temp"] = temp;
  doc["hum"] = hum;
  doc["press"] = pressu;
  doc["lpg"] = lpg;
  doc["co"] = co;
  doc["smoke"] = smoke;

  char buffer[256];
  size_t n = serializeJson(doc, buffer);
  client.publish("/topic1", buffer, n);
  
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  long now = millis();
  if (now - lastMsg > 10000) {
    lastMsg = now;
     sendData();
  }
}
