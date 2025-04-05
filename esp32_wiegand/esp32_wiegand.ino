/**
  ******************************************************************************
  * @file	: esp32_wiegand.ino
  * @company	: Microcube Enterprise
  * @author	: Ricky Siah
  * @email	: myinfo.microcube@gmail.com
  * @vers.	: V1.0.0
  * @date	: 28-March-2025
  * @brief	: ESP32 Wiegand Reader with Ethernet and Web Service
  *			  
  ******************************************************************************
  * @attention
  * Copyright (C) 2025 Microcube Enterprise
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions
  * are met:
  * 1. Redistributions of source code must retain the above copyright
  *    notice, this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright
  *    notice, this list of conditions and the following disclaimer in the
  *    documentation and/or other materials provided with the distribution.
  * 3. The name of the author may not be used to endorse or promote products
  *    derived from this software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
  * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
  * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  ******************************************************************************
**/

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
  #define GPIO_FACT_BT  34
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

#define WIEGAND_BITS 32

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

  #define LOCAL_IP    "192.168.1.100"
  #define GATEWAY_IP  "192.168.1.1"
  #define NETMASK_IP  "255.255.255.0"
  #define DNS_IP1     "8.8.8.8"
  #define DNS_IP2     "8.8.4.4"

// #define USE_WATCHDOG
#define USE_WIEGAND
#define USE_WEBSERVICE

hw_timer_t *timer = NULL;  // Define the timer handle
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t *wdt_timer = NULL; // Define the watchdog timer handle
// Flag to track Ethernet connection status
bool eth_connected = false;
uint32_t chipId = 0;
uint32_t wieCard[265];
uint8_t produce = 0;
uint8_t consume = 0;

#ifdef USE_ETHERNET
String localIP = LOCAL_IP;
String gateway = GATEWAY_IP;
String subnet = NETMASK_IP;
String dns1 = DNS_IP1;
String dns2 = DNS_IP2;
#endif

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
// Variables to store Wiegand data
unsigned long wiegandData = 0;
unsigned long bitCount = 0;
unsigned long lastPulseTime = 0;
const unsigned long TIMEOUT = 2500; // Timeout in microseconds

// Interrupt service routine for Wiegand D0
void ARDUINO_ISR_ATTR wiegandD0ISR() {
  lastPulseTime = micros();
  if (bitCount < WIEGAND_BITS) {
    wiegandData &= ~(1UL << (31 - bitCount)); // Clear the bit
    bitCount++;
  }
}

// Interrupt service routine for Wiegand D1
void ARDUINO_ISR_ATTR wiegandD1ISR() {
  lastPulseTime = micros();
  if (bitCount < WIEGAND_BITS) {
    wiegandData |= (1UL << (31 - bitCount)); // Set the bit
    bitCount++;
  }
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

#ifdef USE_ETHERNET
void saveNetworkSettings(const String& ip, const String& gw, const String& sn, const String& d1, const String& d2) {
  preferences.begin("network", false);
  preferences.putString("ip", ip);
  preferences.putString("gateway", gw);
  preferences.putString("subnet", sn);
  preferences.putString("dns1", d1);
  preferences.putString("dns2", d2);
  preferences.end();
}

bool isValidIP(const String& ip) {
  IPAddress temp;
  return temp.fromString(ip);
}

void loadNetworkSettings() {
  preferences.begin("network", true);
  String ip = preferences.getString("ip", "");
  String gw = preferences.getString("gateway", "");
  String sn = preferences.getString("subnet", "");
  String d1 = preferences.getString("dns1", "");
  String d2 = preferences.getString("dns2", "");
  preferences.end();

  if (isValidIP(ip) && isValidIP(gw) && isValidIP(sn) && isValidIP(d1) && isValidIP(d2)) {
      localIP = ip;
      gateway = gw;
      subnet = sn;
      dns1 = d1;
      dns2 = d2;
  } else {
    Serial.println("Use defaults and save them");
      localIP = LOCAL_IP;
      gateway = GATEWAY_IP;
      subnet = NETMASK_IP;
      dns1 = DNS_IP1;
      dns2 = DNS_IP2;
      saveNetworkSettings(localIP, gateway, subnet, dns1, dns2);
  }
}

void factoryResetNetworkSettings() {
  preferences.begin("network", false);
  preferences.clear();
  preferences.end();
  Serial.println("Factory reset applied: Network settings cleared.");
}

void checkResetButton() {
  pinMode(GPIO_FACT_BT, INPUT);
  if (digitalRead(GPIO_FACT_BT) == LOW) {
      Serial.println("Reset button pressed. Waiting 5 seconds...");
      unsigned long pressStart = millis();
      while (digitalRead(GPIO_FACT_BT) == LOW) {
          if (millis() - pressStart >= 5000) {
              factoryResetNetworkSettings();
              break;
          }
      }
  }
}

#ifdef USE_WEBSERVICE
void handleGetNetwork() {
  StaticJsonDocument<200> doc;
  doc["ip"] = localIP;
  doc["gateway"] = gateway;
  doc["subnet"] = subnet;
  doc["dns1"] = dns1;
  doc["dns2"] = dns2;

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleGetWiegand() {
  StaticJsonDocument<200> doc;
  if(consume == produce) {
    doc["cardid"] = "none";  // no more cards in the buffer
    doc["consume"] = "none";
    doc["produce"] = "none";
  } 
  else {
    doc["cardid"] = wieCard[consume];
    doc["consume"] = consume;
    doc["produce"] = produce;
    consume++;
  }

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

    // Optional: handle network settings update from JSON
    if (doc.containsKey("ip") && doc.containsKey("gateway") && doc.containsKey("subnet") && doc.containsKey("dns1") && doc.containsKey("dns2")) {
      saveNetworkSettings(doc["ip"].as<String>(), doc["gateway"].as<String>(), doc["subnet"].as<String>(), doc["dns1"].as<String>(), doc["dns2"].as<String>());
    }

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
#endif // end of USE_WEBSERVICE
#endif // end of USE_ETHERNET

void setup() {
  // Initialize GPIO pins
  pinMode(GPIO_FACT_LD, OUTPUT);
  pinMode(GPIO_RLY_CH1, OUTPUT);
  pinMode(GPIO_RLY_CH2, OUTPUT);
  pinMode(GPIO_FACT_BT, INPUT);

  // Initialize serial communication
  Serial.begin(115200);
  delay(1000); /* wait serial ok */

  CheckChipID();
  GetMacAddress();
  
#ifdef USE_WIEGAND
  // Initialize Wiegand pins
  pinMode(GPIO_WIEG_D0, INPUT);
  pinMode(GPIO_WIEG_D1, INPUT);

  attachInterrupt(digitalPinToInterrupt(GPIO_WIEG_D0), wiegandD0ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(GPIO_WIEG_D1), wiegandD1ISR, FALLING);
#endif

  init_timer();
#ifdef USE_WATCHDOG
  // Initialize watchdog
  init_watchdog();
#endif

#ifdef USE_ETHERNET
  Serial.println("Initializing Ethernet...");
  if (!ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_POWER, ETH_CLK_MODE)) {
    Serial.println("Ethernet initialization failed, something went wrong!!!!");
    while (1) {
      // infinite loop if initialization fails
      digitalWrite(GPIO_FACT_LD, HIGH);
    }
  }
  delay(200); // Add a short delay to allow PHY to settle
#ifdef USE_STATIC_IP
  checkResetButton();
  loadNetworkSettings();
  IPAddress ip, gw, sn, d1, d2;
  ip.fromString(localIP);
  gw.fromString(gateway);
  sn.fromString(subnet);
  d1.fromString(dns1);
  d2.fromString(dns2);
  // Use the following to configure a static IP
  // ex. ETH.config(local_ip, gateway, subnet, primary_dns, secondary_dns);
  ETH.config(ip, gw, sn, d1, d2);
  Serial.println("*** Static Network: ");
  Serial.print("LocalIP: ");
  Serial.println(ETH.localIP());
  Serial.print("Gateway: ");
  Serial.println(gw);
  Serial.print("Subnet: ");
  Serial.println(sn);
  Serial.print("DNS1: ");
  Serial.println(d1);
  Serial.print("DNS2: ");
  Serial.println(d2);
#else
  ETH.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE); // Use DHCP
  while (ETH.localIP() == INADDR_NONE) {
      delay(100);
  }
  Serial.print("*** DHCP network resolved: ");
  Serial.print("IP Address: ");
  Serial.println(ETH.localIP());
#endif   // end of USE_STATIC_IP
  
  // Set up event handler for Ethernet events
  Network.onEvent(onEvent);

#ifdef USE_WEBSERVICE
  server.on("/get/network", HTTP_GET, handleGetNetwork);
  server.on("/get/wiegand", HTTP_GET, handleGetWiegand);
  server.on("/post", HTTP_POST, handlePost);
    
  server.begin();
  Serial.println("Web service is started."); 
#endif // end of USE_WEBSERVICE
#endif // end of USE_ETHERNET
}

void loop() {

#ifdef USE_WIEGAND
if (bitCount == WIEGAND_BITS) {
  Serial.print("Wiegand Data: ");
  Serial.print(wiegandData, Dec);
  Serial.print(" (");
  Serial.print(wiegandData, BIN);
  Serial.print(")");

  wieCard[produce] = wiegandData;
  produce++;

  bitCount = 0;
  wiegandData = 0;
} else if (bitCount > 0 && (micros() - lastPulseTime > TIMEOUT)) {
  // Timeout, reset
  Serial.println("Wiegand Timeout, Resetting");
  bitCount = 0;
  wiegandData = 0;
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
  delayMicroseconds(100); // Small delay to prevent excessive CPU usage
}