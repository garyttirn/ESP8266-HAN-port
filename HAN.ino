
/**
 * ESP8266 Power Meter HAN-Port H1 reader with Vera (https://home.getvera.com/) and collectd (influxDB) integration.
 *
 * Partly based on
 *  https://github.com/UdoK/esp8266_p1meter_sv
 *  https://github.com/nuclearcat/collectd-embedded
 *  https://github.com/Lestat-GitHub/CollectdPacket
 *
 * To be powered by the Power Meter 
 * 
 * Create vera devices :
 * Temperature sensor :
 *   curl "http://VERAIP:3480/data_request?id=action&serviceId=urn:micasaverde-com:serviceId:HomeAutomationGateway1&action=CreateDevice&deviceType=urn:schemas-micasaverde-com:device:TemperatureSensor:1&internalID=&Description=WiFiTemp1&UpnpDevFilename=D_TemperatureSensor1.xml&UpnpImplFilename=&MacAddress=&RoomNum=0&Reload=1&IpAddress="
 * Humidity sensor :
 *   curl "http://VERAIP:3480/data_request?id=action&serviceId=urn:micasaverde-com:serviceId:HomeAutomationGateway1&action=CreateDevice&deviceType=urn:schemas-micasaverde-com:device:HumiditySensor:1&internalID=&Description=WiFiHum1&UpnpDevFilename=D_HumiditySensor1.xml&UpnpImplFilename=&MacAddress=&RoomNum=0&Reload=1&IpAddress="
 * Pressure sensor :
 *   curl "http://VERAIP:3480/data_request?id=action&serviceId=urn:micasaverde-com:serviceId:HomeAutomationGateway1&action=CreateDevice&deviceType=urn:schemas-micasaverde-com:device:BarometerSensor:1&internalID=&Description=WiFiPressure1&UpnpDevFilename=D_BarometerSensor1.xml&UpnpImplFilename=&MacAddress=&RoomNum=0&Reload=1&IpAddress="
 * Power Meter :
 *   curl "http://VERAIP:3480/data_request?id=action&serviceId=urn:micasaverde-com:serviceId:HomeAutomationGateway1&action=CreateDevice&deviceType=urn:schemas-micasaverde-com:device:PowerMeter:1&internalID=&Description=HAN&UpnpDevFilename=D_PowerMeter1.xml&UpnpImplFilename=&MacAddress=&RoomNum=0&Reload=1&IpAddress="
 *
 **/

#include <Arduino.h>
#include <FS.h>
#include <EEPROM.h>
#include <Ticker.h>
#include <ArduinoOTA.h>
#include <ezTime.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <coredecls.h> //crc32
#include <Adafruit_BME280.h>

//WebSerial
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerialLite.h>

//Larger Serial buffer
#define SERIAL_BUFFER_SIZE 1024

#define BME280_I2C_ADDRESS 0x76
// ESP8266 PIN 4 SDA
// ESP8266 PIN 5 SCL

Adafruit_BME280  bme280;  // initialize Adafruit BME280 library

//Atomic OTA Updates
#define ATOMIC_FS_UPDATE

#include <WiFiUdp.h>
#include "collectd-protocol.h"

// * Include settings
#include "settings.h"

float glb_temp=0,glb_hum=0,glb_pres=0 ;
int glb_batterylevel=100;
long glb_rssi=0;

//metric packet to send to collectd for detailed readings
struct collectd_packet *packet = collectd_init_packet("han", 30000);

// * Initiate ezTime library
Timezone myTZ;

char time_chbuf[16] = {""};

// * Initiate WIFI client
WiFiClient client;

// * Initiate HTTP client
HTTPClient http;

// * Initiate collectd UDP client
WiFiUDP udp_handler;

// WebSerial
// Initiate HTTP server
AsyncWebServer server(80);

/* Message callback of WebSerial */
void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);

  if (d == "REBOOT" ) {
    delay (2);
    ESP.restart();
  }
}

//OTA callback update_started()
void update_started() {
  Serial.println("HTTP update process started");
}

//OTA callback update_finished()
void update_finished() {
  Serial.println("HTTP update process finished");
}

//OTA callback update_progress()
void update_progress(int cur, int total) {
  Serial.printf("HTTP update process at %d of %d bytes...\n", cur, total);
}

//OTA callback update_error()
void update_error(int err) {
  Serial.printf("HTTP update fatal error code %d\n", err);
}

//OTA CHeck
int checkForUpdates() {

    // Add optional callback notifiers
    ESPhttpUpdate.onStart(update_started);
    ESPhttpUpdate.onEnd(update_finished);
    ESPhttpUpdate.onProgress(update_progress);
    ESPhttpUpdate.onError(update_error);
    
    Serial.print("\nChecking for update from " + String(UpdateURL) + ". Current FW version " + String(FWVersion) + "\n");
    t_httpUpdate_return ret = ESPhttpUpdate.update(client, String(UpdateURL),String(FWVersion));

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        delay(5000);
        ESP.restart();
        break;
    }

return 0;
}
void InitTempSensor() {
bme280.begin(BME280_I2C_ADDRESS);
  
bme280.setSampling(Adafruit_BME280::MODE_FORCED, // takeForcedMeasurement must be called before each reading
Adafruit_BME280::SAMPLING_X1, // Temp. oversampling
Adafruit_BME280::SAMPLING_X1, // Pressure oversampling
Adafruit_BME280::SAMPLING_X1, // Humidity oversampling
Adafruit_BME280::FILTER_OFF,
Adafruit_BME280::STANDBY_MS_1000);
}

int ReadTempSensor() {

  //Force sample
  bme280.takeForcedMeasurement();
  Serial.print("\nRequesting BME280 Sensor data...\n");
   
  glb_temp = bme280.readTemperature() - 4; // get temperature in °C, offset by 4 degrees.
  glb_hum = bme280.readHumidity(); // get humidity in rH%
  glb_pres = bme280.readPressure() / 100; // get pressure in Pa

  // Check if any reads failed
  if (isnan(glb_hum) || isnan(glb_temp) || isnan(glb_pres)) {
    Serial.println("Failed to read from SME280 sensor!");
    WebSerial.printf("\nFailed to read from SME280 sensor!");
    return 1;
  }

  Serial.println("Temp: " + String(glb_temp) + "  Hum: " + String(glb_hum) + "  Pres: " + String(glb_pres));
  WebSerial.printf("\nTemp: %s, Hum: %s, Pres: %s\n",String(glb_temp),String(glb_hum),String(glb_pres));
  return 0;
}


int getRSSI(){
  glb_rssi=WiFi.RSSI();
    
  Serial.println("WiFi RSSI: " + String(glb_rssi) + "dBm");
   
  return 0; 
}

int GetHttpURL(String MyURL){
  Serial.print("[HTTP] begin...\n");

  http.begin(client,String(VeraBaseURL) + MyURL);
    
  int httpCode = http.GET();

   if (httpCode > 0) {
     Serial.printf("[HTTP] GET... code: %d\n", httpCode);
    
     if (httpCode == HTTP_CODE_OK) {
        http.writeToStream(&Serial);
        }
  }
  else {
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
 
 return httpCode;
}

//send data to vera
void send_data_to_vera (void)
{
  Serial.println( "\nSending Vera update..." );
  WebSerial.printf("\nSending Vera update...");
      
  http.setReuse(true);

  //Environment
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraTempDeviceID) + "&serviceId=urn:upnp-org:serviceId:TemperatureSensor1&Variable=CurrentTemperature&Value=" + String(glb_temp));
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraTempDeviceID) + "&serviceId=urn:upnp-org:serviceId:TemperatureSensor1&Variable=CurrentRSSI&Value=" + String(glb_rssi));
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraTempDeviceID) + "&serviceId=urn:micasaverde-com:serviceId:HaDevice1&Variable=BatteryLevel&Value=" + String(glb_batterylevel));
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraHumDeviceID) + "&serviceId=urn:micasaverde-com:serviceId:HumiditySensor1&Variable=CurrentLevel&Value=" + String(glb_hum));
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraPresDeviceID) + "&serviceId=urn:upnp-org:serviceId:BarometerSensor1&Variable=CurrentPressure&Value=" + String(glb_pres));

  //Power
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraPowerDeviceID) + "&serviceId=urn:micasaverde-com:serviceId:EnergyMetering1&Variable=KWH&Value=" + String(CONSUMPTION/1000));
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraPowerDeviceID) + "&serviceId=urn:micasaverde-com:serviceId:EnergyMetering1&Variable=Watts&Value=" + String(ACTIVE_POWER));  

  http.end();
}

//send data to collectd
void send_data_to_collectd (void)
{
  Serial.println("\nSending collected packet...");
  WebSerial.printf("\nSending collected packet...");

  collectd_add_numeric(packet, TYPE_TIME, now());
  collectd_add_string(packet, TYPE_PLUGIN, "HAN");
  collectd_add_string(packet, TYPE_TYPE, "gauge");

  //kWh
  collectd_add_string(packet, TYPE_TYPE_INSTANCE, "CONSUMPTION");
  collectd_add_value(packet, COLLECTD_VALUETYPE_GAUGE, &CONSUMPTION);

 //kvarh
  collectd_add_string(packet, TYPE_TYPE_INSTANCE, "ACTIVE_POWER_REACT");
  collectd_add_value(packet, COLLECTD_VALUETYPE_GAUGE, &ACTIVE_POWER_REACT);

  //Watts
  collectd_add_string(packet, TYPE_TYPE_INSTANCE, "ACTIVE_POWER");
  collectd_add_value(packet, COLLECTD_VALUETYPE_GAUGE, &ACTIVE_POWER);
  collectd_add_string(packet, TYPE_TYPE_INSTANCE, "L1_INSTANT_POWER_USAGE");
  collectd_add_value(packet, COLLECTD_VALUETYPE_GAUGE, &L1_INSTANT_POWER_USAGE);
  collectd_add_string(packet, TYPE_TYPE_INSTANCE, "L2_INSTANT_POWER_USAGE");
  collectd_add_value(packet, COLLECTD_VALUETYPE_GAUGE, &L2_INSTANT_POWER_USAGE);
  collectd_add_string(packet, TYPE_TYPE_INSTANCE, "L3_INSTANT_POWER_USAGE");
  collectd_add_value(packet, COLLECTD_VALUETYPE_GAUGE, &L3_INSTANT_POWER_USAGE);
  collectd_add_string(packet, TYPE_TYPE_INSTANCE, "L1_REACT_POWER_USAGE");
  collectd_add_value(packet, COLLECTD_VALUETYPE_GAUGE, &L1_REACT_POWER_USAGE);
  collectd_add_string(packet, TYPE_TYPE_INSTANCE, "L2_REACT_POWER_USAGE");
  collectd_add_value(packet, COLLECTD_VALUETYPE_GAUGE, &L2_REACT_POWER_USAGE);
  collectd_add_string(packet, TYPE_TYPE_INSTANCE, "L3_REACT_POWER_USAGE");
  collectd_add_value(packet, COLLECTD_VALUETYPE_GAUGE, &L3_REACT_POWER_USAGE);

  //Amps
  collectd_add_string(packet, TYPE_TYPE_INSTANCE, "L1_INSTANT_POWER_CURRENT");
  collectd_add_value(packet, COLLECTD_VALUETYPE_GAUGE, &L1_INSTANT_POWER_CURRENT);
  collectd_add_string(packet, TYPE_TYPE_INSTANCE, "L2_INSTANT_POWER_CURRENT");
  collectd_add_value(packet, COLLECTD_VALUETYPE_GAUGE, &L2_INSTANT_POWER_CURRENT);
  collectd_add_string(packet, TYPE_TYPE_INSTANCE, "L3_INSTANT_POWER_CURRENT");
  collectd_add_value(packet, COLLECTD_VALUETYPE_GAUGE, &L3_INSTANT_POWER_CURRENT);

  //Volts
  collectd_add_string(packet, TYPE_TYPE_INSTANCE, "L1_VOLTAGE");
  collectd_add_value(packet, COLLECTD_VALUETYPE_GAUGE, &L1_VOLTAGE);
  collectd_add_string(packet, TYPE_TYPE_INSTANCE, "L2_VOLTAGE");
  collectd_add_value(packet, COLLECTD_VALUETYPE_GAUGE, &L2_VOLTAGE);
  collectd_add_string(packet, TYPE_TYPE_INSTANCE, "L3_VOLTAGE");
  collectd_add_value(packet, COLLECTD_VALUETYPE_GAUGE, &L3_VOLTAGE);
  
  Serial.println( "Sending packet to " + String(CollectdIP[0]) + "." + String(CollectdIP[1]) + "." + String(CollectdIP[2]) + "." + String(CollectdIP[3]) + ":" +  String(CollectdPort));
  
  udp_handler.beginPacket(CollectdIP, atoi(CollectdPort));
  udp_handler.write(packet->buffer, packet->current_offset);
  udp_handler.endPacket();
  
 collectd_reset_packet(packet,"han");
}

// **********************************
// * P1                             *
// **********************************

unsigned int CRC16(unsigned int crc, unsigned char *buf, int len)
{
  for (int pos = 0; pos < len; pos++)
    {
    crc ^= (unsigned int)buf[pos];    // * XOR byte into least sig. byte of crc
                                          // * Loop over each bit
        for (int i = 8; i != 0; i--)
        {
            // * If the LSB is set
            if ((crc & 0x0001) != 0)
            {
                // * Shift right and XOR 0xA001
                crc >>= 1;
        crc ^= 0xA001;
      }
            // * Else LSB is not set
            else
                // * Just shift right
                crc >>= 1;
    }
  }
  return crc;
}

bool isNumber(char *res, int len)
{
    for (int i = 0; i < len; i++)
    {
        if (((res[i] < '0') || (res[i] > '9')) && (res[i] != '.' && res[i] != 0))
            return false;
    }
    return true;
}

int FindCharInArrayRev(char array[], char c, int len)
{
    for (int i = len - 1; i >= 0; i--)
    {
        if (array[i] == c)
            return i;
    }
    return -1;
}

long getValue(char *buffer, int maxlen, char startchar, char endchar)
{
    int s = FindCharInArrayRev(buffer, startchar, maxlen - 2);
    int l = FindCharInArrayRev(buffer, endchar, maxlen - 2) - s - 1;

    char res[16];
    memset(res, 0, sizeof(res));

    if (strncpy(res, buffer + s + 1, l))
    {
        if (endchar == '*')
        {
            if (isNumber(res, l))
                // * Lazy convert string to double
                return (1000 * atof(res));
        }
        else if (endchar == ')')
        {
            if (isNumber(res, l))
                return atof(res);
        }
    }
    return 0;
}
// parsing of telegram according to Swedish ESMR 5.0 implementation //UKR 1220
bool decode_telegram(int len)
{
    int startChar = FindCharInArrayRev(telegram, '/', len);
    int endChar = FindCharInArrayRev(telegram, '!', len);
    bool validCRCFound = false;

    for (int cnt = 0; cnt < len; cnt++) {
        Serial.print(telegram[cnt]);
    }
    Serial.print("\n");

    if (startChar >= 0)
    {
        // * Start found. Reset CRC calculation
        currentCRC = CRC16(0x0000,(unsigned char *) telegram+startChar, len-startChar);
    }
    else if (endChar >= 0)
    {
        // * Add to crc calc
        currentCRC = CRC16(currentCRC,(unsigned char*)telegram+endChar, 1);

        char messageCRC[5];
        strncpy(messageCRC, telegram + endChar + 1, 4);

        messageCRC[4] = 0;   // * Thanks to HarmOtten (issue 5)
        validCRCFound = (strtol(messageCRC, NULL, 16) == currentCRC);

        if (validCRCFound)
            Serial.println(F("CRC Valid!"));
        else {
            Serial.println(F("CRC Invalid!"));
        }

        currentCRC = 0;
    }
    else
    {
        currentCRC = CRC16(currentCRC, (unsigned char*) telegram, len);
    }

    // 1-0:1.8.0(000992.992*kWh)
    // 1-0:1.8.0 = Cumulative hourly active import energy (A+) (Q1+Q4)
    if (strncmp(telegram, "1-0:1.8.0", 9) == 0)
    {
        CONSUMPTION = getValue(telegram, len, '(', '*');
    }

    // 1-0:2.8.0(000560.157*kWh)
    // 1-0:2.8.0 = Cumulative hourly active export energy (A-) (Q2+Q3)
    if (strncmp(telegram, "1-0:2.8.0", 9) == 0)
    {
        RETURNDELIVERY = getValue(telegram, len, '(', '*');
    }

    // 1-0:3.8.0(000560.157*kWh)
    // 1-0:3.8.0 = Cumulative hourly reactive import energy (R+) (Q1+Q2)
    if (strncmp(telegram, "1-0:3.8.0", 9) == 0)
    {
        CONSUMPTION_REACT = getValue(telegram, len, '(', '*');
    }

    // 1-0:4.8.0(000560.157*kWh)
    // 1-0:4.8.0 = Cumulative hourly reactive export energy (R-) (Q3+Q4)
    if (strncmp(telegram, "1-0:4.8.0", 9) == 0)
    {
        RETURNDELIVERY_REACT = getValue(telegram, len, '(', '*');
    }

    // 1-0:1.7.0(00.424*kW)
    // 1-0:1.7.x = Momentary Active power+ (Q1+Q4)
    if (strncmp(telegram, "1-0:1.7.0", 9) == 0)
    {
        ACTIVE_POWER = getValue(telegram, len, '(', '*');
    }

    // 1-0:2.7.0(00.000*kW) 
    // 1-0:2.7.x = Momentary Active power- (Q2+Q3)
    if (strncmp(telegram, "1-0:2.7.0", 9) == 0)
    {
        ACTIVE_POWER_RETURNDELIVERY = getValue(telegram, len, '(', '*');
    }

    // 1-0:3.7.0(00.424*kW)
    // 1-0:3.7.x = Momentary Reactive power + ( Q1+Q2)
    if (strncmp(telegram, "1-0:3.7.0", 9) == 0)
    {
        ACTIVE_POWER_REACT = getValue(telegram, len, '(', '*');
    }

    // 1-0:4.7.0(00.000*kW) 
    // 1-0:4.7.x = Momentary Reactive power - ( Q3+Q4)
    if (strncmp(telegram, "1-0:4.7.0", 9) == 0)
    {
       ACTIVE_POWER_RETURNDELIVERY_REACT = getValue(telegram, len, '(', '*');
    }

    // 1-0:21.7.0(00.378*kW)
    // 1-0:21.7.0 = Momentary Active power+ (L1)
    if (strncmp(telegram, "1-0:21.7.0", 10) == 0)
    {
        L1_INSTANT_POWER_USAGE = getValue(telegram, len, '(', '*');
    }

    // 1-0:22.7.0(00.378*kW)
    // 1-0:22.7.0 = Momentary Active power- (L1)
    if (strncmp(telegram, "1-0:22.7.0", 10) == 0)
    {
        L1_INSTANT_POWER_DELIVERY = getValue(telegram, len, '(', '*');
    }

    // 1-0:41.7.0(00.378*kW)
    // 1-0:41.7.0 = Momentary Active power+ (L2)
    if (strncmp(telegram, "1-0:41.7.0", 10) == 0)
    {
        L2_INSTANT_POWER_USAGE = getValue(telegram, len, '(', '*');
    }

    // 1-0:42.7.0(00.378*kW)
    // 1-0:42.7.0 = Momentary Active power- (L2)
    if (strncmp(telegram, "1-0:42.7.0", 10) == 0)
    {
        L2_INSTANT_POWER_DELIVERY = getValue(telegram, len, '(', '*');
    }

    // 1-0:61.7.0(00.378*kW)
    // 1-0:61.7.0 = Momentary Active power+ (L3)
    if (strncmp(telegram, "1-0:61.7.0", 10) == 0)
    {
        L3_INSTANT_POWER_USAGE = getValue(telegram, len, '(', '*');
    }

    // 1-0:62.7.0(00.378*kW)
    // 1-0:62.7.0 = Momentary Active power- (L3)
    if (strncmp(telegram, "1-0:62.7.0", 10) == 0)
    {
        L3_INSTANT_POWER_DELIVERY = getValue(telegram, len, '(', '*');
    }

    // 1-0:23.7.0(00.378*kW)
    // 1-0:23.7.0 = Momentary Reactive power+ (L1)
    if (strncmp(telegram, "1-0:23.7.0", 10) == 0)
    {
        L1_REACT_POWER_USAGE = getValue(telegram, len, '(', '*');
    }

    // 1-0:24.7.0(00.378*kW)
    // 1-0:24.7.0 = Momentary Reactive power- (L1)
    if (strncmp(telegram, "1-0:24.7.0", 10) == 0)
    {
        L1_REACT_POWER_DELIVERY = getValue(telegram, len, '(', '*');
    }

    // 1-0:43.7.0(00.378*kW)
    // 1-0:43.7.0 = Momentary Reactive power+ (L2)
    if (strncmp(telegram, "1-0:43.7.0", 10) == 0)
    {
        L2_REACT_POWER_USAGE = getValue(telegram, len, '(', '*');
    }

    // 1-0:44.7.0(00.378*kW)
    // 1-0:44.7.0 = Momentary Reactive power+ (L2)
    if (strncmp(telegram, "1-0:44.7.0", 10) == 0)
    {
        L2_REACT_POWER_DELIVERY = getValue(telegram, len, '(', '*');
    }

    // 1-0:63.7.0(00.378*kW)
    // 1-0:63.7.0 = Momentary Reactive power+ (L3)
    if (strncmp(telegram, "1-0:63.7.0", 10) == 0)
    {
        L3_REACT_POWER_USAGE = getValue(telegram, len, '(', '*');
    }

    // 1-0:64.7.0(00.378*kW)
    // 1-0:64.7.0 = Momentary Reactive power- (L3)
    if (strncmp(telegram, "1-0:64.7.0", 10) == 0)
    {
        L3_REACT_POWER_DELIVERY = getValue(telegram, len, '(', '*');
    }

    // 1-0:31.7.0(002*A)
    // 1-0:31.7.0 = Momentary RMS Current phase L1
    if (strncmp(telegram, "1-0:31.7.0", 10) == 0)
    {
        L1_INSTANT_POWER_CURRENT = getValue(telegram, len, '(', '*');
    }
    // 1-0:51.7.0(002*A)
    // 1-0:51.7.0 = Momentary RMS Current phase L2
    if (strncmp(telegram, "1-0:51.7.0", 10) == 0)
    {
        L2_INSTANT_POWER_CURRENT = getValue(telegram, len, '(', '*');
    }
    
    // 1-0:71.7.0(002*A)
    // 1-0:71.7.0 = Momentary RMS Current phase L3
    if (strncmp(telegram, "1-0:71.7.0", 10) == 0)
    {
        L3_INSTANT_POWER_CURRENT = getValue(telegram, len, '(', '*');
    }

    // 1-0:32.7.0(232.0*V)
    // 1-0:32.7.0 = Momentary RMS Phase voltage L1
    if (strncmp(telegram, "1-0:32.7.0", 10) == 0)
    {
        L1_VOLTAGE = getValue(telegram, len, '(', '*');
    }
    // 1-0:52.7.0(232.0*V)
    // 1-0:52.7.0 = Momentary RMS Phase voltage L2
    if (strncmp(telegram, "1-0:52.7.0", 10) == 0)
    {
        L2_VOLTAGE = getValue(telegram, len, '(', '*');
    }   
    // 1-0:72.7.0(232.0*V)
    // 1-0:72.7.0 = Momentary RMS Phase voltage L3
    if (strncmp(telegram, "1-0:72.7.0", 10) == 0)
    {
        L3_VOLTAGE = getValue(telegram, len, '(', '*');
    }

    return validCRCFound;
}

void processLine(int len) {
    telegram[len] = '\n';
    telegram[len + 1] = 0;
    yield();

    bool result = decode_telegram(len + 1);

    if (!result) {
      Serial.print("\nFailed to decoce serial data...");
    }
}

void read_p1_hardwareserial()
{
    if (Serial.available())
    {
        Serial.print("\nReading serial data...\n");
        memset(telegram, 0, sizeof(telegram));
        
        Serial.setTimeout(5000);

        while (Serial.available())
        {
            ESP.wdtDisable();
            int len = Serial.readBytesUntil('\n', telegram, P1_MAXLINELENGTH);
            ESP.wdtEnable(1);

            if (len == 0) {
              Serial.print("Empty P1 telegram.\n");
              break;
            }
            else {
              WebSerial.printf("P1 telegram received: %s\n",telegram);
              processLine(len);
            }
        }
    }
}

// **********************************
// * Setup Main                     *
// **********************************

void setup()
{ 
  Serial.setDebugOutput(false);
  Serial.setRxBufferSize(SERIAL_BUFFER_SIZE);
  
  // Setup a hw serial connection for communication with the P1 meter and logging (not using inversion)
  Serial.begin(BAUD_RATE, SERIAL_8N1, SERIAL_FULL);

  Serial.flush();
  Serial.println("Serial port is ready to receive.");
  Serial.print("\nHAN-port reader started\n");

  Serial.println("WiFi connecting");
    
  // Connect to WiFi
  WiFi.begin(WifiSSID, WifiPass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    //Too many failures -> ESP.reset();
  }
  Serial.println("");
  Serial.println("WiFi connected");

  //OTA Check
  checkForUpdates();

  //Init BME280 with slower sampling
  InitTempSensor(); 
  
  // Init ezTime and sync
  setServer("fi.pool.ntp.org");
  waitForSync();

  myTZ = UTC;

  //Init collectd UDP client
  udp_handler.begin(atoi(CollectdPort)); 

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  //WebSerial
  WebSerial.begin(&server);
  /* Attach Message Callback */
  WebSerial.onMessage(recvMsg);
  server.begin();

  WebSerial.print(F("IP address: "));
  WebSerial.println(WiFi.localIP().toString());
  Serial.println("Read loop started");
  WebSerial.printf("Read loop started");
}

// **********************************
// * Loop                           *
// **********************************

void loop()
{  
  events();

  long now = millis();

  if (now - LAST_UPDATE_SENT > UPDATE_INTERVAL) {
    // Length (with one extra character for the null terminator)
    int str_len = myTZ.dateTime("G:i").length() + 1; 
    myTZ.dateTime("G:i").toCharArray(time_chbuf,str_len);

    Serial.printf("\nTime now: %sZ\nReading BME280 sensor values\n",time_chbuf);
    WebSerial.printf("\nTime now: %sZ\nReading BME280 sensor values\n",time_chbuf);
  
    read_p1_hardwareserial();
    getRSSI();
    ReadTempSensor(); 
    send_data_to_vera(); 
    send_data_to_collectd();
    LAST_UPDATE_SENT = millis();
    }  
}
