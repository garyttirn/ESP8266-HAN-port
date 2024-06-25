// **********************************
// * Settings                       *
// **********************************

const char* WifiSSID      = "SSID";
const char* WifiPass      = "PASSWORD";

int VeraTempDeviceID = 123;
int VeraHumDeviceID = 124;
int VeraPresDeviceID = 125;
int VeraPowerDeviceID = 126;

const char* ESPName = "p1meter";
const char* VeraBaseURL = "http://192.168.1.2:3480/";
const char* UpdateURL = "http://192.168.1.3:4080/8266OTA.php";
const IPAddress CollectdIP = {192,168,255,3};
const char* CollectdPort = "25826";
const char* FWVersion = "13062024";

// Update treshold in milliseconds, messages will only be sent on this interval
#define UPDATE_INTERVAL 30000  // 30s
//#define UPDATE_INTERVAL 300000 // 5 minutes

// * Baud rate for both hardware and software 
#define BAUD_RATE 115200

// The used serial pins, note that this can only be UART0, as other serial port doesn't support inversion
// By default the UART0 serial will be used. These settings displayed here just as a reference. 
// #define SERIAL_RX RX
// #define SERIAL_TX TX

// * Max telegram length
#define P1_MAXLINELENGTH 1050

// * The hostname of our little creature
#define HOSTNAME "p1meter"

// * Wifi timeout in milliseconds
#define WIFI_TIMEOUT 30000

long LAST_UPDATE_SENT = 0;

// * Set to store received telegram
char telegram[P1_MAXLINELENGTH];

// * Set to store the data values read
double CONSUMPTION = 0;
double CONSUMPTION_REACT = 0;

double RETURNDELIVERY = 0;
double RETURNDELIVERY_REACT = 0;

double ACTIVE_POWER = 0;
double ACTIVE_POWER_REACT = 0;

double ACTIVE_POWER_RETURNDELIVERY = 0;
double ACTIVE_POWER_RETURNDELIVERY_REACT = 0;

double L1_INSTANT_POWER_USAGE = 0;
double L1_INSTANT_POWER_DELIVERY = 0;
double L2_INSTANT_POWER_USAGE = 0;
double L2_INSTANT_POWER_DELIVERY = 0;
double L3_INSTANT_POWER_USAGE = 0;
double L3_INSTANT_POWER_DELIVERY = 0;

double L1_REACT_POWER_USAGE = 0;
double L1_REACT_POWER_DELIVERY = 0;
double L2_REACT_POWER_USAGE = 0;
double L2_REACT_POWER_DELIVERY = 0;
double L3_REACT_POWER_USAGE = 0;
double L3_REACT_POWER_DELIVERY = 0;

double L1_INSTANT_POWER_CURRENT = 0;
double L2_INSTANT_POWER_CURRENT = 0;
double L3_INSTANT_POWER_CURRENT = 0;
double L1_VOLTAGE = 0;
double L2_VOLTAGE = 0;
double L3_VOLTAGE = 0;

// * Set during CRC checking
unsigned int currentCRC = 0;
