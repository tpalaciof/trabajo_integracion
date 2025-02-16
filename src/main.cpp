#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <stdlib.h>
#include "UbidotsEsp32Mqtt.h"
#include "data.h"
#include "Settings.h"

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <TFT_eSPI.h>
#include <SPI.h>

#define BUTTON_LEFT 0        // btn activo en bajo
#define LONG_PRESS_TIME 3000 // 3000 milis = 3s

WebServer server(80);

Settings settings;
int lastState = LOW; // para el btn
int currentState;    // the current reading from the input pin
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;

#define DHTPIN 32
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);
TFT_eSPI tft = TFT_eSPI();

const char *UBIDOTS_TOKEN = "BBUS-4hv6Actifdoq4CA5DBQzL7f3r0njat"; // Put here your Ubidots TOKEN
const char *DEVICE_LABEL = "esp32";                                // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL = "temp";                               // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL1 = "Temperatura";                       // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL2 = "Humedad";                           // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL3 = "sw1";                               // Variable a subscribir
const char *VARIABLE_LABEL4 = "sw2";                               // Variable a subscribir
const char *CUSTOM_TOPIC = "/v2.0/devices/demo/+";                 // This will subscribe to all the messages received by the Device labeled as "demo"
const int PUBLISH_FREQUENCY = 5000;                                // Update rate in milliseconds

unsigned long timer;

const int LED = 15; // Pin used to write data based on 1's and 0's coming from Ubidots
Ubidots ubidots(UBIDOTS_TOKEN);

void load404();
void loadIndex();
void loadFunctionsJS();
void restartESP();
void saveSettings();
bool is_STA_mode();
void AP_mode_onRst();
void STA_mode_onRst();
void detect_long_press();
void callback(char *topic, byte *payload, unsigned int length);

// Rutina para iniciar en modo AP (Access Point) "Servidor"
void startAP()
{
  WiFi.disconnect();
  delay(19);
  Serial.println("Starting WiFi Access Point (AP)");
  WiFi.softAP("fabio_AP", "facil123");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

// Rutina para iniciar en modo STA (Station) "Cliente"
void start_STA_client()
{
  WiFi.softAPdisconnect(true);
  WiFi.disconnect();
  delay(100);
  Serial.println("Starting WiFi Station Mode");
  WiFi.begin((const char *)settings.ssid.c_str(), (const char *)settings.password.c_str());
  WiFi.mode(WIFI_STA);

  int cnt = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    // Serial.print(".");
    if (cnt == 100) // Si después de 100 intentos no se conecta, vuelve a modo AP
      AP_mode_onRst();
    cnt++;
    Serial.println("attempt # " + (String)cnt);
  }

  WiFi.setAutoReconnect(true);
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
  pressedTime = millis();

  pinMode(LED, OUTPUT);
  dht.begin();

  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.drawString("Temperatura :", 2, 2, 2);
  tft.drawString("o", 2, 83, 2);
  tft.drawString("C", 10, 89, 4);
  tft.drawString("Humedad :", 2, 120, 2);
  tft.drawString("%", 2, 210, 4);

  tft.fillCircle(80, 218, 15, TFT_DARKGREEN);
  tft.fillCircle(120, 218, 15, TFT_BROWN);

  // Rutinas de Ubidots
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL3);
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL4);
  timer = millis();
}

void setup()
{

  Serial.begin(115200);
  delay(2000);

  EEPROM.begin(4096);                 // Se inicializa la EEPROM con su tamaño max 4KB
  pinMode(BUTTON_LEFT, INPUT_PULLUP); // btn activo en bajo

  // settings.reset();
  settings.load(); // se carga SSID y PWD guardados en EEPROM
  settings.info(); // ... y se visualizan

  Serial.println("");
  Serial.println("starting...");

  if (is_STA_mode())
  {
    start_STA_client();
  }
  else // Modo Access Point & WebServer
  {
    startAP();

    /* ========== Modo Web Server ========== */

    /* HTML sites */
    server.onNotFound(load404);

    server.on("/", loadIndex);
    server.on("/index.html", loadIndex);
    server.on("/functions.js", loadFunctionsJS);

    /* JSON */
    server.on("/settingsSave.json", saveSettings);
    server.on("/restartESP.json", restartESP);

    server.begin();
    Serial.println("HTTP server started");
  }
}

void loop()
{
  if (is_STA_mode()) // Rutina para modo Station (cliente Ubidots)
  {
    if (!ubidots.connected())
    {
      ubidots.reconnect();
    }
    if ((millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
    {
      float t = dht.readTemperature();
      float h = dht.readHumidity();
      float value = (float)random(300);
      ubidots.add(VARIABLE_LABEL1, t); // Insert your variable Labels and the value to be sent
      ubidots.add(VARIABLE_LABEL2, h);
      ubidots.add(VARIABLE_LABEL, value); // Insert your variable Labels and the value to be sent
      ubidots.publish(DEVICE_LABEL);

      tft.drawString(String(t), 0, 30, 7);
      tft.drawString(String(h), 0, 150, 7);
      Serial.print(F("Temperatura: "));
      Serial.print(t);
      Serial.print(F("°C"));
      Serial.print('\n');
      Serial.print(F("Humedad: "));
      Serial.print(h);
      Serial.print(F("% "));
      Serial.print('\n');
      timer = millis();
    }
    ubidots.loop();
  }
  else // rutina para AP + WebServer
    server.handleClient();

  delay(10);
  detect_long_press();
}

// funciones para responder al cliente desde el webserver:
// load404(), loadIndex(), loadFunctionsJS(), restartESP(), saveSettings()

void load404()
{
  server.send(200, "text/html", data_get404());
}

void loadIndex()
{
  server.send(200, "text/html", data_getIndexHTML());
}

void loadFunctionsJS()
{
  server.send(200, "text/javascript", data_getFunctionsJS());
}

void restartESP()
{
  server.send(200, "text/json", "true");
  ESP.restart();
}

void saveSettings()
{
  if (server.hasArg("ssid"))
    settings.ssid = server.arg("ssid");
  if (server.hasArg("password"))
    settings.password = server.arg("password");

  settings.save();
  server.send(200, "text/json", "true");
  STA_mode_onRst();
}

// Rutina para verificar si ya se guardó SSID y PWD del cliente
// is_STA_mode retorna true si ya se guardaron
bool is_STA_mode()
{
  if (EEPROM.read(flagAdr))
    return true;
  else
    return false;
}

void AP_mode_onRst()
{
  EEPROM.write(flagAdr, 0);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void STA_mode_onRst()
{
  EEPROM.write(flagAdr, 1);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void detect_long_press()
{
  // read the state of the switch/button:
  currentState = digitalRead(BUTTON_LEFT);

  if (lastState == HIGH && currentState == LOW) // button is pressed
    pressedTime = millis();
  else if (lastState == LOW && currentState == HIGH)
  { // button is released
    releasedTime = millis();

    // Serial.println("releasedtime" + (String)releasedTime);
    // Serial.println("pressedtime" + (String)pressedTime);

    long pressDuration = releasedTime - pressedTime;

    if (pressDuration > LONG_PRESS_TIME)
    {
      Serial.println("(Hard reset) returning to AP mode");
      delay(500);
      AP_mode_onRst();
    }
  }

  // save the the last state
  lastState = currentState;
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  // Construir los topics esperados para sw1 y sw2 dinámicamente
  char sw1Topic[50];
  char sw2Topic[50];
  sprintf(sw1Topic, "/v2.0/devices/%s/%s/lv", DEVICE_LABEL, VARIABLE_LABEL3); // Topic para sw1
  sprintf(sw2Topic, "/v2.0/devices/%s/%s/lv", DEVICE_LABEL, VARIABLE_LABEL4); // Topic para sw2

  // Comparar el topic recibido con los esperados
  if (strcmp(topic, sw1Topic) == 0)
  { // Si el topic es de sw1
    for (int i = 0; i < length; i++)
    {
      if ((char)payload[0] == '1')
      {
        Serial.println("on - sw1");
        tft.fillCircle(80, 218, 15, TFT_RED); // Ejemplo: Círculo izquierdo
        digitalWrite(LED, HIGH);
      }
      else
      {
        Serial.println("off - sw1");
        tft.fillCircle(80, 218, 15, TFT_BROWN);
        digitalWrite(LED, LOW);
      }
    }
  }
  else if (strcmp(topic, sw2Topic) == 0)
  { // Si el topic es de sw2
    for (int i = 0; i < length; i++)
    {
      if ((char)payload[0] == '1')
      {
        Serial.println("on - sw2");
        tft.fillCircle(120, 218, 15, TFT_GREEN); // Ejemplo: Círculo derecho
        // digitalWrite(LED, HIGH);
      }
      else
      {
        Serial.println("off - sw2");
        tft.fillCircle(120, 218, 15, TFT_DARKGREEN);
        // digitalWrite(LED, LOW);
      }
    }
  }
  Serial.println();
}
