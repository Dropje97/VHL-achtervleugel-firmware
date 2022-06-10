/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com
*********/

#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_GPS.h>
#include <cstring>

double GPSspeed_filter;
char speedbuf[8];

// == GPS Settings ==
//HardwareSerial Serial1(2);
#define GPSSerial Serial2
Adafruit_GPS GPS(&Serial2);
#define GPSECHO false
uint32_t timer = millis();

// == buffer ==
//char* fix2;

// == WiFi Settings ==
// Replace the next variables with your SSID/Password combination
char* ssid = "Zonneboot";
char* password = "Zonnepanelen1";


// == MQTT Broker settings ==
const char* mqtt_server = "telemetrie.zonnebootteam.nl";
const int mqtt_server_port = 1883;
const char* mqtt_username = "Zonneboot";
const char* mqtt_password = "Zonnepanelen1";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

//== LED Settings ==
const int ledPin = 22;
const char* test = "";

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_server_port);
  client.setCallback(callback);

  pinMode(ledPin, OUTPUT);
  // == SETUP GPS ==
  Serial.println("GPS Serial started");
  GPS.begin(9600);
  // setting up to recieve RMC and GGA data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Setting update rate, either 1HZ, 5HZ or 10HZ
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ); // 5 Hz update rate for sending data from gps to micocontroller
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ); // 5 Hz update rate for the gps it self
  //Request updates on antenna status, we have no clue what it does....
  //GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
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
  if (String(topic) == "esp32/gps") {
    Serial.print("Changing output to ");
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
    if (client.connect("espClient", mqtt_username, mqtt_password)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/gps");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // == GPS Main Loop ==
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  long now = millis();

  static long lastSpeed;

  if (now - lastSpeed > 100) {
    lastSpeed = now;
    // == Speed ==
    //char speedbuf[8];
     // GPSspeed_filter
    double GPSspeed = GPS.speed;
    GPSspeed_filter = GPSspeed_filter * 0.80 + GPSspeed * 0.20;
    dtostrf(GPSspeed_filter, 3, 2, speedbuf);

    Serial.print("Speed: ");
    Serial.println(GPSspeed, 2);
   // client.publish("esp32/gps/speed", speedbuf);
  }

  if (now - lastMsg > 1000) {
    lastMsg = now;
    // == FIX ==
    char fixbuf[8];
    double GPSfix = GPS.fix;
    dtostrf(GPSfix, 2, 0, fixbuf);
    Serial.print("Fix: ");
    Serial.println(GPSfix);
    client.publish("esp32/gps/fix", fixbuf);

    // == Quality ==
    char qualbuf[8];
    double GPSqual = GPS.fixquality;
    dtostrf(GPSqual, 2, 0, qualbuf);
    Serial.print("Quality: ");
    Serial.println(GPSqual);
    client.publish("esp32/gps/quality", qualbuf);

    // == Latitude ==
    char latitudebuf[16];
    double GPSlatitude = GPS.latitude;
    dtostrf(GPSlatitude, 6, 4, latitudebuf); // todo: test hogere resolutie dtostrf(GPSlatitude, 8, 5, latitudebuf);
    Serial.print("Latitude: ");
    Serial.println(GPSlatitude, 5);
    client.publish("esp32/gps/latitude", latitudebuf);

    // == Longitude ==
    //    char longitudebuf[16];
    //    double GPSlongitude = GPS.longitude;
    //    dtostrf(GPSlongitude,7,5,longitudebuf);
    //    Serial.print("Longitude: ");
    //    Serial.println(GPSlongitude, 4);
    //    client.publish("esp32/gps/longitude", longitudebuf);

    // == Longitude v2 ==
    client.publish("esp32/gps/longitude", String(GPS.longitude).c_str());

    // == Poles ==
    client.publish("esp32/gps/latitude/pole", String(GPS.lat).c_str());
    client.publish("esp32/gps/longitude/pole", String(GPS.lon).c_str());

    // == Speed ==
/*  char speedbuf[8];
    double GPSspeed_filter;
    double GPSspeed = GPS.speed;
    GPSspeed_filter = GPSspeed_filter * 0.0 + GPSspeed * 1.0;
    dtostrf(GPSspeed_filter, 3, 2, speedbuf);
*/
    Serial.print("Speed_filter: ");
    Serial.println(GPSspeed_filter, 2);
    client.publish("esp32/gps/speed", speedbuf);

    // == Angle ==
    char anglebuf[8];
    double GPSangle = GPS.angle;
    dtostrf(GPSangle, 3, 2, anglebuf);
    Serial.print("Angle: ");
    Serial.println(GPSangle, 2);
    client.publish("esp32/gps/angle", anglebuf);

    // == Altitude ==
    char altbuf[8];
    double GPSalt = GPS.altitude;
    dtostrf(GPSalt, 3, 2, altbuf);
    Serial.print("Altitude: ");
    Serial.println(GPSalt, 2);
    client.publish("esp32/gps/altitude", altbuf);

    // == Satellites ==
    char satbuf[8];
    double GPSsat = GPS.satellites;
    dtostrf(GPSsat, 2, 0, satbuf);
    Serial.print("Satellites: ");
    Serial.println(GPSsat, 2);
    client.publish("esp32/gps/satellites", satbuf);

    //    humidity = bme.readHumidity();

    // Convert the value to a char array
    //    char humString[8];
    //    dtostrf(humidity, 1, 2, humString);
    //    Serial.print("Humidity: ");
    //    Serial.println(humString);
    //    client.publish("esp32/gps", humString);
  }
}
