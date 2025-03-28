#include <ArduinoJson.h>
#include <ETH.h>
#include <WebServer.h>
#include <Preferences.h>
#include <driver/timer.h>
#include "esp_system.h"
#include "rom/ets_sys.h"
#include "esp_mac.h"

//////////gpio define//////////
/* UART0 */
#define GPIO_TXD0   1
#define GPIO_RXD0   3

/* RMII */
#define GPIO_RMII_RXD0  25
#define GPIO_RMII_RXD1  26
#define GPIO_RMII_CRS   27
#define GPIO_RMII_TXD1  22
#define GPIO_RMII_TXD0  19
#define GPIO_RMII_TXEN  21
#define GPIO_RMII_CLK   17
#define GPIO_RMII_MDC   23
#define GPIO_RMII_MDIO  18

/* CAN */
#define GPIO_CAN_RX   4
#define GPIO_CAN_TX   5

//////////relay channel select//////////

#define DT_2CH  1
// #define DT_4CH  1
// #define DT_8CH  1    /* for hw version < v3.6.11 */
//#define DT_16CH 1    /* for hw version < v3.6.0 */
//#define DT_32CH 1    /* for hw version < v3.4.0 */
//#define DT_8CH_V2  1    /* for hw version >= v3.6.11 */
//#define DT_16CH_V2 1    /* for hw version >= v3.6.0 */
//#define DT_32CH_V2 1    /* for hw version >= v3.4.0 */

#if (1 == DT_2CH) || (1 == DT_4CH) || (1 == DT_8CH) || (1 == DT_8CH_V2)
#define GPIO_RMII_PWR 0
#elif (1 == DT_16CH) || (1 == DT_32CH) || (1 == DT_16CH_V2) || (1 == DT_32CH_V2)
#define GPIO_RMII_PWR 12
#else
#define GPIO_RMII_PWR 0
#endif

//////////wiegand input//////////
#ifdef DT_2CH
  #define GPIO_FACT_LD  32
  #define GPIO_WIEG_D0  36
  #define GPIO_WIEG_D1  39
  #define GPIO_RLY_CH1  16
  #define GPIO_RLY_CH2  2
#elif DT_4CH
  #define GPIO_FACT_LD  15
  #define GPIO_WIEG_D0  36
  #define GPIO_WIEG_D1  39
#elif DT_8CH
  #define GPIO_FACT_LD  2
  #define GPIO_WIEG_D0  14
  #define GPIO_WIEG_D1  15
#endif

// ethernet pin define
#define USE_ETHERNET                             // Add support for ethernet (+20k code)
#define USE_STATIC_IP
  // Dingtian Ethernet
  #define ETH_PHY_TYPE        ETH_PHY_JL1101
  #define ETH_PHY_ADDR        1
  #define ETH_PHY_MDC         GPIO_RMII_MDC
  #define ETH_PHY_MDIO        GPIO_RMII_MDIO
  #define ETH_PHY_POWER       GPIO_RMII_PWR
  #define ETH_CLK_MODE        ETH_CLOCK_GPIO17_OUT

// #define USE_WATCHDOG
// #define USE_WIEGAND
#define USE_WEBSERVICE

hw_timer_t *timer = NULL;  // Define the timer handle
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t *wdt_timer = NULL; // Define the watchdog timer handle
// Flag to track Ethernet connection status
bool eth_connected = false;
uint32_t chipId = 0;

#ifdef USE_WEBSERVICE
// create a server instance on port 80
WebServer server(80);
#endif
Preferences preferences;

#ifdef USE_WATCHDOG
void ARDUINO_ISR_ATTR resetModule() {
  ets_printf("reboot\n");
  esp_restart();
}

void init_watchdog(void) {
  // Initialize the watchdog timer
  wdt_timer = timerBegin(1000000);                     //timer 1Mhz resolution
  timerAttachInterrupt(wdt_timer, &resetModule);       //attach callback
  timerAlarm(wdt_timer, (wdtTimeout * 1000), false, 0);  //set time in us
}
#endif

#ifdef USE_WIEGAND
// Wiegand variables
volatile uint32_t wiegandCode = 0;
volatile uint8_t wiegandBitCount = 0;

// Interrupt service routine for Wiegand D0
void ARDUINO_ISR_ATTR wiegandD0ISR() {
  wiegandCode <<= 1;
  wiegandBitCount++;
}

// Interrupt service routine for Wiegand D1
void ARDUINO_ISR_ATTR wiegandD1ISR() {
  wiegandCode <<= 1;
  wiegandCode |= 1;
  wiegandBitCount++;
}
#endif

// Define the timer interrupt service routine
void ARDUINO_ISR_ATTR blinkLED() {
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  // if you are using a variable in the ISR, you should declare it as volatile
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
  digitalWrite(GPIO_FACT_LD, !digitalRead(GPIO_FACT_LD));
}

void init_timer(void) {
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Set timer frequency to 1Mhz
  timer = timerBegin(1000000);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &blinkLED);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  timerAlarm(timer, 1000000, true, 0);
}

// WARNING: onEvent is called from a separate FreeRTOS task (thread)!
void onEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      // The hostname must be set after the interface is started, but needs
      // to be set before DHCP, so set it from the event handler thread.
      ETH.setHostname("esp32-ethernet");
      break;

    case ARDUINO_EVENT_ETH_CONNECTED: 
      Serial.println("ETH Connected"); 
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.println("ETH Got IP");
      Serial.println(ETH);
      eth_connected = true;
      break;

    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.println("ETH Lost IP");
      eth_connected = false;
      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;

    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;

    default:
      break;
  }
}

void CheckChipID(void)
{
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores\n", ESP.getChipCores());
  Serial.print("Chip ID: ");
  Serial.println(chipId);
}

String getDefaultMacAddress() {

  String mac = "";
  unsigned char mac_base[6] = {0};

  if (esp_efuse_mac_get_default(mac_base) == ESP_OK) {
    char buffer[18];  // 6*2 characters for hex + 5 characters for colons + 1 character for null terminator
    sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X", mac_base[0], mac_base[1], mac_base[2], mac_base[3], mac_base[4], mac_base[5]);
    mac = buffer;
  }
  return mac;
}

String getInterfaceMacAddress(esp_mac_type_t interface) {

  String mac = "";
  unsigned char mac_base[6] = {0};

  if (esp_read_mac(mac_base, interface) == ESP_OK) {
    char buffer[18];  // 6*2 characters for hex + 5 characters for colons + 1 character for null terminator
    sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X", mac_base[0], mac_base[1], mac_base[2], mac_base[3], mac_base[4], mac_base[5]);
    mac = buffer;
  }
  return mac;
}

void GetMacAddress(void)
{  
  Serial.println("Interface\t\t\t\t\t\tMAC address (6 bytes, 4 universally administered, default)");

  Serial.print("Wi-Fi Station (using 'esp_efuse_mac_get_default')\t");
  Serial.println(getDefaultMacAddress());

  Serial.print("WiFi Station (using 'esp_read_mac')\t\t\t");
  Serial.println(getInterfaceMacAddress(ESP_MAC_WIFI_STA));

  Serial.print("WiFi Soft-AP (using 'esp_read_mac')\t\t\t");
  Serial.println(getInterfaceMacAddress(ESP_MAC_WIFI_SOFTAP));

  Serial.print("Bluetooth (using 'esp_read_mac')\t\t\t");
  Serial.println(getInterfaceMacAddress(ESP_MAC_BT));

  Serial.print("Ethernet (using 'esp_read_mac')\t\t\t\t");
  Serial.println(getInterfaceMacAddress(ESP_MAC_ETH));
}


void handleGet() {
  StaticJsonDocument<200> doc;
  doc["message"] = "Hello from ESP32";
  doc["status"] = "success";
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handlePost() {
  if (server.hasArg("plain")) {
      String rawMessage = server.arg("plain");
      Serial.println("Raw JSON received: " + rawMessage);
      
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, rawMessage);
      
      if (error) {
          server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
          return;
      }
      
      String receivedMessage = doc["message"] | "No message";
      Serial.println("Parsed message: " + receivedMessage);
      
      StaticJsonDocument<200> responseDoc;
      responseDoc["received"] = receivedMessage;
      responseDoc["status"] = "ok";
      
      String response;
      serializeJson(responseDoc, response);
      server.send(200, "application/json", response);
  } else {
      server.send(400, "application/json", "{\"error\":\"No JSON received\"}");
  }
}

void setup() {
  // Initialize GPIO pins
  pinMode(GPIO_FACT_LD, OUTPUT);
  pinMode(GPIO_RLY_CH1, OUTPUT);
  pinMode(GPIO_RLY_CH2, OUTPUT);

  // Initialize serial communication
  Serial.begin(115200);
  delay(1000); /* wait serial ok */

  CheckChipID();
  GetMacAddress();
  
#ifdef USE_WIEGAND
  // Initialize Wiegand pins
  pinMode(GPIO_WIEG_D0, INPUT);
  pinMode(GPIO_WIEG_D1, INPUT);
  attachInterrupt(GPIO_WIEG_D0, wiegandD0ISR, FALLING);
  attachInterrupt(GPIO_WIEG_D1, wiegandD1ISR, FALLING);
#endif

  init_timer();
#ifdef USE_WATCHDOG
  // Initialize watchdog
  init_watchdog();
#endif

#ifdef USE_ETHERNET
  // Initialize Ethernet
  if (!ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_POWER, ETH_CLK_MODE)) {
    Serial.println("Ethernet initialization failed, something went wrong!!!!");
    while (1) {
      // infinite loop if initialization fails
      digitalWrite(GPIO_FACT_LD, HIGH);
    }
  }
  delay(200); // Add a short delay to allow PHY to settle
#ifdef USE_STATIC_IP
  // Use the following to configure a static IP
  // ex. ETH.config(local_ip, gateway, subnet, primary_dns, secondary_dns);
  ETH.config(IPAddress(192,168,1,100), IPAddress(192,168,1,1), IPAddress(255,255,255,0), 
            IPAddress(192,168,1,1), IPAddress(192,168,1,1));
#else
  ETH.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE); // Use DHCP
#endif // end of USE_STATIC_IP
  // Set up event handler for Ethernet events
  Network.onEvent(onEvent);

#ifdef USE_WEBSERVICE
  server.on("/get", HTTP_GET, handleGet);
  server.on("/post", HTTP_POST, handlePost);
    
  server.begin();
  Serial.println("Web service is started."); 
#endif // end of USE_WEBSERVICE
#endif // end of USE_ETHERNET
}

void loop() {
  static bool ledToggle = true;
#ifdef USE_WIEGAND
  // Check for Wiegand code
  if (wiegandBitCount >= 26) {
    Serial.println("Wiegand code: " + String(wiegandCode));
    wiegandCode = 0;
    wiegandBitCount = 0;
  }
#endif

#ifdef USE_WATCHDOG
  // Feed the watchdog (reset the timer)
  timerWrite(wdt_timer, 0);
  delay(1); // or some other short delay
#endif

#ifdef USE_WEBSERVICE
  // Handle web server requests
  server.handleClient();
#endif
}