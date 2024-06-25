# ESP8266-HAN-port
ESP8266 Power Meter HAN-Port H1 reader with Vera (https://home.getvera.com/) and collectd (influxDB) integration.
Also includes BME280 sensor and WebSerial for monitoring.

Partly based on
 * https://github.com/psvanstrom/esphome-p1reader
 * https://github.com/UdoK/esp8266_p1meter_sv
 * https://github.com/nuclearcat/collectd-embedded
 * https://github.com/Lestat-GitHub/CollectdPacket
 * https://github.com/asjdf/WebSerialLite

## Circuit ##

Essentially according to Pär Svanström's Barebone circuit. BME280 added to GPIO 4 (SDA) & 5 (SCL) 
![Pär Svanström Barebone ESP8266](https://github.com/psvanstrom/esphome-p1reader/blob/main/images/p1reader-barebone-ESP-12F.png)

## Vera Device Creation ##

Temperature sensor :
```
curl "http://VERAIP:3480/data_request?id=action&serviceId=urn:micasaverde-com:serviceId:HomeAutomationGateway1&action=CreateDevice&deviceType=urn:schemas-micasaverde-com:device:TemperatureSensor:1&internalID=&Description=WiFiTemp1&UpnpDevFilename=D_TemperatureSensor1.xml&UpnpImplFilename=&MacAddress=&RoomNum=0&Reload=1&IpAddress="
```
Humidity sensor :
```
curl "http://VERAIP:3480/data_request?id=action&serviceId=urn:micasaverde-com:serviceId:HomeAutomationGateway1&action=CreateDevice&deviceType=urn:schemas-micasaverde-com:device:HumiditySensor:1&internalID=&Description=WiFiHum1&UpnpDevFilename=D_HumiditySensor1.xml&UpnpImplFilename=&MacAddress=&RoomNum=0&Reload=1&IpAddress="
```
Pressure sensor :
```
 curl "http://VERAIP:3480/data_request?id=action&serviceId=urn:micasaverde-com:serviceId:HomeAutomationGateway1&action=CreateDevice&deviceType=urn:schemas-micasaverde-com:device:BarometerSensor:1&internalID=&Description=WiFiPressure1&UpnpDevFilename=D_BarometerSensor1.xml&UpnpImplFilename=&MacAddress=&RoomNum=0&Reload=1&IpAddress="
```

Power Meter :
```
curl "http://VERAIP:3480/data_request?id=action&serviceId=urn:micasaverde-com:serviceId:HomeAutomationGateway1&action=CreateDevice&deviceType=urn:schemas-micasaverde-com:device:PowerMeter:1&internalID=&Description=HAN&UpnpDevFilename=D_PowerMeter1.xml&UpnpImplFilename=&MacAddress=&RoomNum=0&Reload=1&IpAddress="
```

## Other Information ##

The implementation has been tested on Aidon 7534 Smart Meter.
BME280 showed elevated temperature readings that have compensated via hardcoded constant (-4 Degress) in the code. 


[SESKO SK 13-1:2022 Suositus sähköenergiamittareiden paikallista asiakasrajapintaa varten](https://sesko.fi/wp-content/uploads/2022/12/Suositus-SK-13-1_H1-asiakasrajapinta_suomenkielinen_2022-12-13.pdf)
