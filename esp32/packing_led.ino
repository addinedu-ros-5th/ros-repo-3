#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

const char* ssid = "ssid";
const char* password = "password";

const int LED_R = 14;
const int LED_G = 12;
const int LED_B = 13;

AsyncWebServer server(80);

void set_color(int red, int green, int blue)
{
  analogWrite(LED_R, red);
  analogWrite(LED_G, green);
  analogWrite(LED_B, blue);
}

void setup() 
{
  Serial.begin(115200);
  set_color(0, 255, 0);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request)
  {
    request->send(200, "text/plain", "Server on");
  });

  server.on("/led_control", HTTP_POST, [](AsyncWebServerRequest * request)
  {
    String color;
    if (request->hasParam("color"))
    {
      color = request->getParam("color")->value();
      Serial.print("Received color: ");
      Serial.println(color);

      if (color == "red")
      {
        set_color(255, 0, 0);
      }
      else if (color == "green")
      {
        set_color(0, 255, 0);
      }
    }
    request->send(200, "text/plain", "LED color changed to " + color);
  });

  server.begin();
  Serial.println("HTTP Server Started");
  delay(100);
}

void loop() 
{

}
