/* NODE Mesin Petik V1.2 (1)
   WENDA YUSUP
   PT.MAKERINDO PRIMA SOLUSI*/

/*------------Source Framework-----------*/
#include <Arduino.h>

/*------------Source Timing--------------*/
unsigned long awal = millis();

/*------------Source LoRa----------------*/
#include <SPI.h>
#include <LoRa.h>
#define SS 18
#define RST 14
#define DIO0 26
#define SCK 5
#define MISO 19
#define MOSI 27

/*------------Source GPS-----------------*/
#include <TinyGPS++.h>
#define RX 34
#define TX 12
HardwareSerial neogps(1);
TinyGPSPlus gps;

/*------------Source DHT22----------------*/
#include <DHT.h>
#define DHTPIN 14
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

/*------------Source MPU9250---------------*/
#include "sub_module/MPU9250_RPY.h"
#include "sub_module/MPU9250.h"
extern MPU9250 IMU;
int roll_calibrate = 0;
int pitch_calibrate = 0;
int yaw_calibrate = 0;
int roll_c = 0;
int pitch_c = 0;
int yaw_c = 0;

/*------------Source Battery-----------------*/
#include <axp20x.h>
AXP20X_Class axp;
const uint8_t slave_address = AXP192_SLAVE_ADDRESS;

/*------------Source Mac Address-------------*/
#include <esp_system.h>
#include <regex.h>
String macAddress = "";

/*------------Source led pin-------------*/
#define ledgps 13
#define ledlora 25
#define ledkiri 15
#define ledkanan 2

//-----Payload Data
String PAYLOAD = "";
String NodeID = "MPGMBG082300(nan)";
String RepeaterID = "nan";
int RepeaterRSSI = -200;
String OtherNodeID = "nan";
int OtherNodeRSSI = -200;
String RSSI = "nan";
String Latitude = "0";
String Longitude = "0";
String Altitude = "0";
String Temp = "0";
String Humidity = "0";
extern int roll;
extern int pitch;
extern int yaw;
String VBatt = "0";
//-----sendMessage
String SenderNode = "";
String outgoing;
byte msgCount = 0;
String incoming = "";
String message = "";
//-----CodeMesssage
byte Noderepeater = 0xB0;
byte NodeNetral = 0x00;
byte NodepetikA = 0xA0;

// BOOL
bool noderptmacval = true;
bool noderptpayloadval = false;
bool nodetoother = false;
bool success = false;
int x = 0;
int y = 0;
int mac = 0;

//---------OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define NUMFLAKES 10 // Number of snowflakes in the animation example
#define LOGO_HEIGHT 16
#define LOGO_WIDTH 16

//---------------------------------------------------------VOIDSETUP()
void setup()
{
  /*------------PEMBACAAN SERIAL MAIN-----------*/
  Serial.begin(115200);

  /*------------PEMBACAAN MAC ADDRESS-----------*/
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);

  // Menyimpan MAC address dalam variabel global
  for (int i = 0; i < 6; ++i)
  {
    macAddress += String(mac[i], HEX);
    if (i < 5)
      macAddress += ":";
  }

  /*------------PEMBACAAN LoRa-----------*/
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);
  LoRa.begin(915E6);
  LoRa.setSpreadingFactor(11);
  LoRa.setCodingRate4(2);
  LoRa.setSignalBandwidth(500E3);

  /*------------PEMBACAAN GPS-----------*/
  neogps.begin(9600, SERIAL_8N1, RX, TX);

  /*------------PEMBACAAN DHT22-----------*/
  dht.begin();

  Wire.begin(21, 22);
  /*------------PEMBACAAN BATERAI-----------*/
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS))
    ;
  {
    Serial.println("Battery Failed");
  }

  axp.adc1Enable(AXP202_VBUS_VOL_ADC1 |
                     AXP202_VBUS_CUR_ADC1 |
                     AXP202_BATT_CUR_ADC1 |
                     AXP202_BATT_VOL_ADC1,
                 true);

  /*------------PEMBACAAN MPU9250-----------*/
  MPU9250Setup();
  while (millis() < 5000)
  {
    MPU9250Loop();

    roll_calibrate = roll;
    pitch_calibrate = pitch;
    yaw_calibrate = yaw;
  }

  /*------------PEMBACAAN LED----------------*/
  pinMode(ledgps, OUTPUT);
  pinMode(ledlora, OUTPUT);
  pinMode(ledkiri, OUTPUT);
  pinMode(ledkanan, OUTPUT);

  /*------------PEMBACAAN OLED----------------*/
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
}

//------------Proc SendMessage-----------//
void sendMessage(String outgoing, byte Noderepeater, byte NodepetikA)
{
  LoRa.beginPacket();            // start packet
  LoRa.write(Noderepeater);      // add destination address
  LoRa.write(NodepetikA);        // add sender address
  LoRa.write(msgCount);          // add message ID
  LoRa.write(outgoing.length()); // add payload length
  LoRa.print(outgoing);          // add payload
  LoRa.endPacket();              // finish packet and send it
  msgCount++;                    // increment message ID
}
void onReceive(int packetSize)
{
  if (packetSize == 0)
    return;
  int recipient = LoRa.read();
  byte sender = LoRa.read();
  byte incomingMsgId = LoRa.read();
  byte incomingLength = LoRa.read();

  while (LoRa.available())
  {
    incoming = LoRa.readStringUntil('\n');
  }
  if (incomingLength != incoming.length())
  {
    // Serial.println("error: message length does not match length");
    ;
    return;
  }

  // COMMUNICATION NODE TO REPEATER
  if (recipient != NodepetikA && recipient != Noderepeater)
  {
    // Serial.println("This message is not for me().");
    ;
    return;
  }

  // // COMMUNICATION NODE TO NODE
  // if (recipient != NodepetikA && recipient != NodeNetral)
  // {
  //   // Serial.println("This message is not for me().");
  //   ;
  //   return;
  // }

  if (incoming == macAddress + " " + "1")
  {
    Serial.println(incoming);
    RepeaterRSSI = LoRa.packetRssi();
    RSSI = RepeaterRSSI;
    mac++;
    digitalWrite(ledlora, HIGH);
    delay(200);
    digitalWrite(ledlora, LOW);
    if (mac == 2)
    {
      RepeaterID = "RPGMBG0823001";
      NodeID = "MPGMBG0823001";
      noderptmacval = false;
      noderptpayloadval = true;
      success = true;
    }
  }

  if (incoming == macAddress + " " + "2")
  {
    Serial.println(incoming);
    RepeaterRSSI = LoRa.packetRssi();
    RSSI = RepeaterRSSI;
    mac++;
    digitalWrite(ledlora, HIGH);
    delay(200);
    digitalWrite(ledlora, LOW);
    if (mac == 2)
    {
      RepeaterID = "RPGMBG0823001";
      NodeID = "MPGMBG0823002";
      noderptmacval = false;
      noderptpayloadval = true;
      success = true;
    }
  }

  if (incoming == macAddress + " " + "3")
  {
    Serial.println(incoming);
    RepeaterRSSI = LoRa.packetRssi();
    RSSI = RepeaterRSSI;
    mac++;
    digitalWrite(ledlora, HIGH);
    delay(200);
    digitalWrite(ledlora, LOW);
    if (mac == 2)
    {
      RepeaterID = "RPGMBG0823001";
      NodeID = "MPGMBG0823003";
      noderptmacval = false;
      noderptpayloadval = true;
      success = true;
    }
  }

  if (incoming == macAddress + " " + "4")
  {
    Serial.println(incoming);
    RepeaterRSSI = LoRa.packetRssi();
    RSSI = RepeaterRSSI;
    mac++;
    digitalWrite(ledlora, HIGH);
    delay(200);
    digitalWrite(ledlora, LOW);
    if (mac == 2)
    {
      RepeaterID = "RPGMBG0823001";
      NodeID = "MPGMBG0823004";
      noderptmacval = false;
      noderptpayloadval = true;
      success = true;
    }
  }

  if (incoming == macAddress + " " + "5")
  {
    Serial.println(incoming);
    RepeaterRSSI = LoRa.packetRssi();
    RSSI = RepeaterRSSI;
    mac++;
    digitalWrite(ledlora, HIGH);
    delay(200);
    digitalWrite(ledlora, LOW);
    if (mac == 2)
    {
      RepeaterID = "RPGMBG0823001";
      NodeID = "MPGMBG0823005";
      noderptmacval = false;
      noderptpayloadval = true;
      success = true;
    }
  }

  if (success)
  {
    if (NodeID == "MPGMBG0823001" && incoming == "success 1")
    {
      Serial.println(incoming);
       RepeaterRSSI = LoRa.packetRssi();
    RSSI = RepeaterRSSI;
      digitalWrite(ledlora, HIGH);
      delay(200);
      digitalWrite(ledlora, LOW);
    }
    else if (NodeID == "MPGMBG0823002" && incoming == "success 2")
    {
      Serial.println(incoming);
       RepeaterRSSI = LoRa.packetRssi();
    RSSI = RepeaterRSSI;
      digitalWrite(ledlora, HIGH);
      delay(200);
      digitalWrite(ledlora, LOW);
    }
  }
}

//------------Proc MAc Address-----------//
void macproc()
{
  Serial.print("Mac Address Node 1 : ");
  Serial.println(macAddress);
}

//------------Proc Device---------------//

/*---------DHT22*/
void dhtdata()
{
  Temp = dht.readTemperature();
  Humidity = dht.readHumidity();
  // Serial.print(Suhu);
  // Serial.println(Kelembaban);
}

/*---------MPU9250*/
void mpudata()
{
  MPU9250Loop();
  roll = roll - roll_calibrate;
  pitch = pitch - pitch_calibrate;
  yaw = yaw - yaw_calibrate;
  // Serial.println(pitch);
  // if (pitch < -3)
  // {
  //   digitalWrite(ledkiri, HIGH);
  // }
  // else
  // {
  //   digitalWrite(ledkiri, LOW);
  // }
  // if (pitch > 3)
  // {
  //   digitalWrite(ledkanan, HIGH);
  // }
  // else
  // {
  //   digitalWrite(ledkanan, LOW);
  // }
}

/*---------Batt*/
void batterydata()
{
  VBatt = axp.getBattVoltage();
}

/*---------GPS*/
void gpsdata()
{
  while (neogps.available())
    if (gps.encode(neogps.read()))
      if (gps.location.isValid())
      {
        Latitude = String(gps.location.lat(), 6);
        Longitude = String(gps.location.lng(), 6);
        Altitude = String(gps.altitude.meters() - 25.00);
        digitalWrite(ledgps, HIGH);
        delay(100);
        digitalWrite(ledgps, LOW);
        // Serial.println("GPS Ada");
      }
      else
      {
        digitalWrite(ledgps, LOW);
        //     Serial.println("Nyarii..");
      }
}

/*---------Display*/
void displaydata()
{
  int BatteryPercent = axp.getBattVoltage();
  BatteryPercent = map(BatteryPercent, 0, 4200, 0, 100);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(String() + "    " + NodeID);
  display.println(String() + "---------------------");
  display.println(String() + "Uplink: " + RepeaterID);
  display.println(String() + "Neigh : " + OtherNodeID);
  display.println(String() + "RSSI  : " + RSSI);
  display.println(String() + "Lat   : " + Latitude);
  display.println(String() + "Lon   : " + Longitude);
  display.println(String() + "Batt  : " + BatteryPercent + " %");
  display.display();
}

bool isMACAddressValidA(const String &mac)
{
  const char *validFormatPatternA = "^([0-9A-Fa-f]{1,2}[:-]){5}([0-9A-Fa-f]{1,2})$";

  regex_t regex;
  regcomp(&regex, validFormatPatternA, REG_EXTENDED);

  int result = regexec(&regex, mac.c_str(), 0, NULL, 0);

  regfree(&regex);

  return result == 0;
}

bool isMACAddressValidB(const String &mac)
{
  const char *validFormatPatternB = "^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$";
  regex_t regex;

  // Compile pola regex
  if (regcomp(&regex, validFormatPatternB, REG_EXTENDED) != 0)
  {
    Serial.println("Error compiling regex");
    return false;
  }

  // Mengecek apakah MAC address sesuai dengan pola regex
  bool isValid = (regexec(&regex, mac.c_str(), 0, NULL, 0) == 0);

  // Bebaskan sumber daya yang digunakan oleh pola regex
  regfree(&regex);

  return isValid;
}

void loop()
{

  // lOOPING DEVICE
  dhtdata();
  gpsdata();
  mpudata();
  batterydata();
  displaydata();

  // macvalidation();
  // PAYLOAD
  PAYLOAD = String() +
            NodeID +
            "," +
            RepeaterID +
            "," +
            RepeaterRSSI +
            "," +
            OtherNodeID +
            "," +
            OtherNodeRSSI +
            "," +
            Latitude +
            "," +
            Longitude +
            "," +
            Altitude +
            "," +
            Temp +
            "," +
            Humidity +
            "," +
            roll +
            "," +
            pitch +
            "," +
            yaw +
            "," +
            VBatt +
            ",*";

  // PENGIRIMAN
  unsigned long sekarang = millis();
  if (noderptmacval)
  {
    if (sekarang - awal >= 5000)
    {
      awal = sekarang;
      sendMessage(macAddress, Noderepeater, NodepetikA);
    }
  }

  if (noderptpayloadval)
  {
    if (sekarang - awal >= 5000)
    {
      awal = sekarang;
      sendMessage(PAYLOAD, Noderepeater, NodepetikA);
    }
  }

  // PENERIMAAN
  onReceive(LoRa.parsePacket());
}
