#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#include <SPI.h>
#include <MFRC522.h>
#include <List.hpp>

const char* ssid = "ssid";
const char* password = "password";

const char* server = "http://address:port/tag/product";

const int RST_PIN = 22;
const int SS_PIN = 21;

List<MFRC522::Uid> tag_list;
MFRC522 mfrc522(SS_PIN, RST_PIN);

void send_data()
{
  if (WiFi.status() != WL_CONNECTED) { return; }

  HTTPClient http;

  http.begin(server);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<200> jsonDoc;
  jsonDoc["tag_data"] = "Success";

  String requestBody;
  serializeJson(jsonDoc, requestBody);

  int httpResponseCode = http.POST(requestBody);

  if (httpResponseCode != 200)
  {
    Serial.print("Send fail: ");
    Serial.println(httpResponseCode);
    http.end();
  }
  else
  {
    Serial.print("Send Success");
    http.end();
  }
}

void setup() 
{
  Serial.begin(115200);

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

  SPI.begin();
  mfrc522.PCD_Init();
  Serial.println("RFID reader initialized");

}

void loop() 
{
  if (!mfrc522.PICC_IsNewCardPresent()) { return; }
  
  if (!mfrc522.PICC_ReadCardSerial()) { return; }

  bool registered = false;

  for (int i = 0; i < tag_list.getSize(); i++)
  {
    if (memcmp(tag_list.get(i).uidByte, mfrc522.uid.uidByte, 4) == 0)
    {
      registered = true;
      break;
    }
  }

  if (!registered) 
  {
    tag_list.addLast(mfrc522.uid);
    Serial.println("tag added");
    send_data();
  }

  delay(100);
}
