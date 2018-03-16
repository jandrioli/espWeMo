#include <Arduino.h>
#include <ArduinoJson.h>
#include "FS.h" 
#ifdef ESP32
    #include <WiFi.h>
#else
    #include <ESP8266WiFi.h>
#endif
#ifdef ESP8266
extern "C" {
#include "user_interface.h"
}
#endif
#include "fauxmoESP.h"

#define HW_LEDG    4            //  output
#define HW_LEDR    5            //  output
#define HW_WATER  13            //  input_pullup
#define HW_RELAY1 14            //  output
#define HW_RELAY2 12            //  output
#define THIS_NODE_ID 3                  // master is 0, unoR3 debugger is 1, promicro_arrosoir is 2, etc
#define DEFAULT_ACTIVATION 600          // 10h from now we activate (in case radio is down and can't program)
#define DEFAULT_DURATION 10             // max 10s of activation time by default
#define DEFAULT_HEBDO 1                 // repeat program every 1 day 

#define useCredentialsFile
#ifdef useCredentialsFile
#include "credentials.h"
#else
mySSID = "    ";
myPASSWORD = "   ";
#endif
/**
 * exchange data via radio more efficiently with data structures.
 * we can exchange max 32 bytes of data per msg. 
 * schedules are reset every 24h (last for a day) so an INTEGER is
 * large enough to store the maximal value of a 24h-schedule.
 * temperature threshold is rarely used
 */
struct relayctl {
  uint32_t uptime = 0;                      // current running time of the machine (millis())  4 bytes  
  uint32_t sched1 = DEFAULT_ACTIVATION;     // schedule in minutes for the relay output nbr1   4 bytes
  uint32_t sched2 = DEFAULT_ACTIVATION;     // schedule in minutes for the relay output nbr2   4 bytes
  uint16_t maxdur1 = DEFAULT_DURATION;      // max duration nbr1 is ON                         2 bytes
  uint16_t maxdur2 = DEFAULT_DURATION;      // max duration nbr2 is ON                         2 bytes
   int8_t  temp_thres = 99;                 // temperature at which the syatem is operational  1 byte
   int8_t  temp_now   = 20;                 // current temperature read on the sensor          1 byte
  uint8_t  battery    =  0;                 // current temperature read on the sensor          1 byte
  bool     state1 = false;                  // state of relay output 1                         1 byte
  bool     state2 = false;                  // "" 2                                            1 byte
  bool     waterlow = false;                // indicates whether water is low                  1 byte
  uint8_t  nodeid = 64; /*SSDP*/            // nodeid is the identifier of the slave   (SSDP)  1 byte
} __attribute__ ((packed)) myData;

String                g_nwSSID = "", 
                      g_nwPASS = "",
                      g_tgCHAT = "60001082";
// ESP8266WebServer      server ( 80 );
int                   Bot_mtbs = 10000; //if i lower this interval, heap collides with stack
unsigned long         Bot_lasttime = 0;   //last time messages' scan has been done
bool                  activation_notified = false;
const static char     m_sHR[]  = "- - - - -";
const static char     m_sBtnSuccess[]  = "btn-success";
const static char     m_sBtnDanger[]  = "btn-danger";
const static char     m_sClassActive[]  = "class='active'";
const static char     m_sON[]  = "ON";
const static char     m_sOFF[]  = "OFF";
uint8_t               m_ssidScan = 0;
uint8_t               m_hebdo = DEFAULT_HEBDO;
unsigned long         t = 0;
bool                  b = false, 
                      bRealState1 = false, 
                      bRealState2 = false;

#define SERIAL_BAUDRATE                 115200
#define LED                             2

fauxmoESP fauxmo;



bool loadConfig() 
{
  Serial.println(F("Loading configuration..."));
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println(F("Failed to open config file"));
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024) {
    Serial.println(F("Config file size is too large"));
    return false;
  }

  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);

  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  configFile.readBytes(buf.get(), size);

  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(buf.get());

  if (!json.success()) {
    Serial.println(F("Failed to parse config file"));
    return false;
  }

  if (json.containsKey("ssid")) 
  {
    //const char* nwSSID = json["ssid"];
    g_nwSSID = String((const char*)json["ssid"]);
  }
  if (json.containsKey("pass")) 
  {
    //const char* nwPASS = json["pass"];
    g_nwPASS = String((const char*)json["pass"]);
  }
  if (json.containsKey("chat")) 
  {
    //const char* tgCHAT = json["chat"];
    g_tgCHAT = String((const char*)json["chat"]);
  }
  Serial.println("["+g_nwSSID+"]");  
  Serial.println("["+g_nwPASS+"]");
  Serial.println("["+g_tgCHAT+"]");
  if (json.containsKey("sched1") )
  {
    myData.sched1 = json["sched1"];
    myData.sched2 = json["sched2"];
    myData.maxdur1 = json["maxdur1"];
    myData.maxdur2 = json["maxdur2"];
  }

  if (json.containsKey("hebdo") )
  {
    m_hebdo = (uint8_t)json["hebdo"];
  }
  
  if (g_nwSSID.length() < 4 || g_nwPASS.length() < 6)
  {
    Serial.println(F("SSID or PSK were too short, defaulting to hard-coded nw."));
    g_nwSSID = mySSID;
    g_nwPASS = myPASSWORD;
  }
  return true;
}

bool saveConfig() 
{
  Serial.println(F("Saving configuration into spiffs..."));
  char cSSID[g_nwSSID.length()+1], cPASS[g_nwPASS.length()+1], cCHAT[g_tgCHAT.length()+1];
  g_nwSSID.toCharArray(cSSID, g_nwSSID.length()+1);    
  g_nwPASS.toCharArray(cPASS, g_nwPASS.length()+1);    
  g_tgCHAT.toCharArray(cCHAT, g_tgCHAT.length()+1);
  Serial.print(F("Saving new SSID:["));
  Serial.print(cSSID);
  Serial.println(']');
  Serial.print(F("Saving new PASS:["));
  Serial.print(cPASS);
  Serial.println(']');
  Serial.print(F("Saving new CHAT:["));
  Serial.print(cCHAT);
  Serial.println(']');
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["ssid"] = cSSID;
  json["pass"] = cPASS;
  json["chat"] = cCHAT;
  
  json["sched1"] = myData.sched1;
  json["sched2"] = myData.sched2;
  
  json["maxdur1"] = myData.maxdur1;
  json["maxdur2"] = myData.maxdur2;
  json["hebdo"] = m_hebdo;
  
  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println(F("Failed to open config file for writing"));
    return false;
  }
  json.printTo(configFile);
  return true;
}

void setup_spiffs()
{
  uint32_t realSize = ESP.getFlashChipRealSize();
  uint32_t ideSize = ESP.getFlashChipSize();
  FlashMode_t ideMode = ESP.getFlashChipMode();

  if(ideSize == realSize) 
  {
    if (!SPIFFS.begin()) {
      Serial.println(F("Failed to mount file system"));
      return;
    }
    if (!loadConfig()) {
      Serial.println(F("Failed to load config"));
    }
  } 
  else 
  {
    Serial.println(F("Flash Chip configuration wrong!\n"));
  }
  printf(("Flash real id:   %08X\n"), ESP.getFlashChipId());
  printf(("Flash real size: %u\n"), realSize);
  printf(("Flash ide  size: %u\n"), ideSize);
  printf(("Flash ide speed: %u\n"), ESP.getFlashChipSpeed());
  printf(("Flash ide mode:  %s\n"), (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));
  Serial.println(m_sHR);
} // setup_spiffs

// -----------------------------------------------------------------------------
// Wifi
// -----------------------------------------------------------------------------

void wifiSetup() 
{
  WiFi.mode(WIFI_OFF);
  yield();
  b = !b;
    
  delay(10);

  // Connect to WiFi network
  Serial.println(F("Connecting WiFi..."));
  delay(20); 
  WiFi.mode(WIFI_STA);
  yield();
  delay(20); 
  
  char cSSID[g_nwSSID.length()+1], cPASS[g_nwPASS.length()+1];
  g_nwSSID.toCharArray(cSSID, g_nwSSID.length()+1);    
  g_nwPASS.toCharArray(cPASS, g_nwPASS.length()+1);    
  WiFi.begin(cSSID, cPASS);
  yield();

  int timeout = 20;
  while (WiFi.status() != WL_CONNECTED) 
  {
    digitalWrite(HW_LEDR, HIGH);
    delay(100);
    digitalWrite(HW_LEDR, LOW);
    delay(900);
    
    if (timeout == 15) // a basic connect timeout sorta thingy
    {
      Serial.println();
      WiFi.printDiag(Serial);
      Serial.print(F("Failed to connect to WiFi nw. Status is now "));
      Serial.println(WiFi.status());
      Serial.println(F("Connecting first hardcoded wifi network"));
      WiFi.mode(WIFI_OFF);
      yield();
      delay(50); 
      WiFi.begin(mySSID, myPASSWORD);
    }
    if (timeout == 10) // a basic connect timeout sorta thingy
    {
      Serial.println();
      WiFi.printDiag(Serial);
      Serial.print(F("Failed to connect to WiFi nw. Status is now "));
      Serial.println(WiFi.status());
      Serial.println(F("Connecting secondary hardcoded wifi network"));
      WiFi.mode(WIFI_OFF);
      yield();
      delay(50); 
      WiFi.begin(mySSID2, myPASSWORD2);
    }
    if (timeout == 5) // a basic connect timeout sorta thingy
    {
      Serial.println();
      WiFi.printDiag(Serial);
      Serial.print(F("Failed to connect to WiFi nw. Status is now "));
      Serial.println(WiFi.status());
      Serial.println(F("Connecting thirdly hardcoded wifi network"));
      WiFi.mode(WIFI_OFF);
      yield();
      delay(50); 
      WiFi.begin(mySSID3, myPASSWORD3);
    }
    if (--timeout < 1) // a basic authentication-timeout sorta thingy
    {
      break;
    }
  }
  if (WiFi.status() != WL_CONNECTED) 
  {
    // this is also used by handleConfig(), dont delete this line!
    m_ssidScan = WiFi.scanNetworks();
      
      Serial.println(F("WiFi connection FAILED."));
      WiFi.printDiag(Serial);
    
      if (m_ssidScan == 0)
      {
        Serial.println(F("no networks found"));
      }
      else
      {
        Serial.print(m_ssidScan);
        Serial.println(F(" networks found"));
        for (int i = 0; i < m_ssidScan; ++i)
        {   
          // Print SSID and RSSI for each network found
          Serial.print(i + 1);
          Serial.print(F(": "));
          Serial.print(WiFi.SSID(i));
          Serial.print(F(" ("));
          Serial.print(WiFi.RSSI(i));
          Serial.print(F(")"));
          Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE)?" ":"*");
          delay(10);
        }
      }
    //starting software Access Point
    WiFi.mode(WIFI_OFF);
    delay(1);
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Regador", "RegadorAndrioli");
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
//    server.on ( "/wifi", handleConfig );
  }
  else 
  {
    Serial.println(F("WiFi connected"));    
    Serial.println(WiFi.localIP());
  }
  Serial.println(m_sHR);
  digitalWrite(HW_LEDR, false);
}

void printInfos()
{
  
    Serial.print(F("system_get_free_heap_size(): "));
    Serial.println(system_get_free_heap_size());

    Serial.print(F("system_get_os_print(): "));
    Serial.println(system_get_os_print());
    system_set_os_print(1);
    Serial.print(F("system_get_os_print(): "));
    Serial.println(system_get_os_print());

    system_print_meminfo();

    Serial.print(F("system_get_chip_id(): 0x"));
    Serial.println(system_get_chip_id(), HEX);

    Serial.print(F("system_get_sdk_version(): "));
    Serial.println(system_get_sdk_version());

    Serial.print(F("system_get_boot_version(): "));
    Serial.println(system_get_boot_version());

    Serial.print(F("system_get_userbin_addr(): 0x"));
    Serial.println(system_get_userbin_addr(), HEX);

    Serial.print(F("system_get_boot_mode(): "));
    Serial.println(system_get_boot_mode() == 0 ? F("SYS_BOOT_ENHANCE_MODE") : F("SYS_BOOT_NORMAL_MODE"));

    Serial.print(F("system_get_cpu_freq(): "));
    Serial.println(system_get_cpu_freq());
    
  Serial.print(F("wifi_get_opmode(): "));
  Serial.println(wifi_get_opmode());

  Serial.print(F("wifi_get_opmode_default(): "));
  Serial.println(wifi_get_opmode_default());
  
  Serial.print(F("wifi_get_broadcast_if(): "));
  Serial.println(wifi_get_broadcast_if());
  
  Serial.print(F("WiFi MAC Address: "));
  Serial.println(WiFi.macAddress());
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("WiFi connected"));    
    Serial.println(WiFi.localIP());
  }
  else {
    Serial.print(F("WiFi status: "));    
    Serial.println(WiFi.status());
  }
  
  Serial.println( "Compiled: " __DATE__ ", " __TIME__ ", " __VERSION__);

  Serial.println(m_sHR);  
} // printInfos

void setup() 
{
  WiFi.mode(WIFI_OFF);
  yield();
  pinMode(HW_LEDG, OUTPUT);
  pinMode(HW_LEDR, OUTPUT);
  pinMode(HW_WATER, INPUT_PULLUP);
  pinMode(HW_RELAY1, OUTPUT);
  pinMode(HW_RELAY2, OUTPUT);
  
    // Init serial port and clean garbage
    Serial.begin(SERIAL_BAUDRATE);
    Serial.println();
    Serial.println();

    delay(100);
    Serial.println();
    Serial.println(F("========================"));  
    Serial.println(F("  REGADOR WIFI ESP-12E  "));  
    Serial.println(F("========================"));  
    delay(100);
    Serial.println(F("Warning! Always query the controller node before attempting to program it!"));  
    delay(100);
    Serial.println(m_sHR); 
    
    printInfos();
    
    Serial.print(F("system_get_time(): "));
    Serial.println(system_get_time());
    
    //prepare and configure SPIFFS
    setup_spiffs();
  
    // Wifi
    wifiSetup();

    // LED
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);

    // You can enable or disable the library at any moment
    // Disabling it will prevent the devices from being discovered and switched
    fauxmo.enable(true);

    // Add virtual devices
    fauxmo.addDevice("switch one");
	  fauxmo.addDevice("switch two"); // You can add more devices
	//fauxmo.addDevice("switch three");

    // fauxmoESP 2.0.0 has changed the callback signature to add the device_id,
    // this way it's easier to match devices to action without having to compare strings.
    fauxmo.onSetState([](unsigned char device_id, const char * device_name, bool state) 
    {
        Serial.printf("[MAIN] Device #%d (%s) state: %s\n", device_id, device_name, state ? "ON" : "OFF");
        if (device_id = '1')
        myData.state1 =  !state;
        else
        myData.state2 =  !state;
    });

    // Callback to retrieve current state (for GetBinaryState queries)
    fauxmo.onGetState([](unsigned char device_id, const char * device_name) 
    {
      Serial.printf("[MAIN] Get #%d (%s) state: %s\n", device_id, device_name);
        if (device_id = '1')
        return myData.state1 ;
        else
        return myData.state2 ;
    });

}

void loop() {

    // Since fauxmoESP 2.0 the library uses the "compatibility" mode by
    // default, this means that it uses WiFiUdp class instead of AsyncUDP.
    // The later requires the Arduino Core for ESP8266 staging version
    // whilst the former works fine with current stable 2.3.0 version.
    // But, since it's not "async" anymore we have to manually poll for UDP
    // packets
    fauxmo.handle();

    static unsigned long last = millis();
    if (millis() - last > 5000) {
        last = millis();
        Serial.printf("[MAIN] Free heap: %d bytes\n", ESP.getFreeHeap());
    }

  while (Serial.available())
  {
    String s1 = Serial.readStringUntil('\n');
    Serial.println(F("CAREFUL, end of line is only NL and no CR!!!"));
    Serial.print(F("You typed:"));
    Serial.println(s1);
    if (s1.indexOf("setnewssid ")>=0)
    {
      s1 = s1.substring(s1.indexOf(" ")+1);
      g_nwSSID = s1.substring(0, s1.length());
      Serial.println(("new ssid is now [") + g_nwSSID + "]");
    }
    else if (s1.indexOf("setnewpass ")>=0)
    {
      s1 = s1.substring(s1.indexOf(" ")+1);
      g_nwPASS = s1.substring(0, s1.length());
      Serial.println(("new pass is now [") + g_nwPASS + "]");
    }
    else if (s1.indexOf("setnewpass ")>=0)
    {
      s1 = s1.substring(s1.indexOf(" ")+1);
      g_tgCHAT = s1.substring(0, s1.length());
      Serial.println(("new pass is now [") + g_tgCHAT + "]");
    }
    else if (s1.indexOf("save")>=0)
    {
      saveConfig();
    }
    else if (s1.indexOf("reboot please")>=0)
    {
      ESP.restart();
    }
    else if (s1.indexOf("debug")>=0)
    {
      printInfos();
    }      
    else if ((s1.indexOf("setnewpass")!=0) && (s1.indexOf("setnewssid")!=0) && (s1.indexOf("setnewchat")!=0))
    {
      Serial.println(F("** Serial interface expects:\n\r"\
        "** 0 - setnewssid: set a new SSID for the module\n\r"\
        "** 1 - setnewpass: set a new PSK key for the nw\n\r"\
        "** 1 - setnewchat: set a new chat destinataire\n\r"\
        "** 3 - save : save the configuration into a file on the flash"));
    }
  }
}
