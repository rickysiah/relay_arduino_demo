#include <ArduinoJson.h>
#include <ETH.h>
#include <WebServer.h>
#include <driver/timer.h>
#include "esp_system.h"
#include "rom/ets_sys.h"

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

//#define DT_2CH  1
// #define DT_4CH  1
#define DT_8CH  1    /* for hw version < v3.6.11 */
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
  // Dingtian Ethernet
  #define ETH_PHY_TYPE        ETH_PHY_JL1101
  #define ETH_PHY_ADDR        1
  #define ETH_PHY_MDC         GPIO_RMII_MDC
  #define ETH_PHY_MDIO        GPIO_RMII_MDIO
  #define ETH_PHY_POWER       GPIO_RMII_PWR
  #define ETH_CLK_MODE        ETH_CLOCK_GPIO17_OUT

// #define USE_WATCHDOG
// #define USE_WIEGAND

hw_timer_t *timer = NULL;  // Define the timer handle
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t *wdt_timer = NULL; // Define the watchdog timer handle
const int wdtTimeout = 10000; // time in 10ms to trigger the watchdog

// Web server
WebServer server(80);

// Flag to track Ethernet connection status
bool eth_connected = false;
uint32_t chipId = 0;

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

// Callback function for Ethernet events
void onEvent(arduino_event_id_t event) {
  switch (event) {
  case ARDUINO_EVENT_ETH_START:
    Serial.println("ETH Started");
    ETH.setHostname("esp32-ethernet-dt");
    break;
  case ARDUINO_EVENT_ETH_CONNECTED:
    Serial.println("ETH Connected");
    break;
  case ARDUINO_EVENT_ETH_GOT_IP:
    Serial.println("ETH Got IP");
    Serial.println(ETH.localIP());
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

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // CheckChipID();

#ifdef USE_WIEGAND
  // Initialize Wiegand pins
  pinMode(GPIO_WIEG_D0, INPUT);
  pinMode(GPIO_WIEG_D1, INPUT);
  attachInterrupt(GPIO_WIEG_D0, wiegandD0ISR, FALLING);
  attachInterrupt(GPIO_WIEG_D1, wiegandD1ISR, FALLING);
#endif

  // init_timer();
#ifdef USE_WATCHDOG
  // Initialize watchdog
  init_watchdog();
#endif

  // Initialize Ethernet
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO,
            ETH_PHY_POWER, ETH_CLK_MODE);
  ETH.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE); // Use DHCP

  // Set up event handler for Ethernet events
  Network.onEvent(onEvent);

  // Initialize web server
  server.on("/post", HTTP_POST, handlePost);
  server.on("/get", HTTP_GET, handleGet);
  server.begin();
}

void loop() {
  // Handle incoming HTTP requests
  server.handleClient();

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
}

void handlePost() {
  // Check if the request has a JSON body
  if (server.hasArg("plain") == false) {
    server.send(200, "text/plain", "Body not received");
    return;
  }

  // Get the JSON body
  String json = server.arg("plain");

  // Parse the JSON body
  DynamicJsonDocument doc(2048);
  DeserializationError error = deserializeJson(doc, json);

  if (error) {
    server.send(200, "text/plain", "Invalid JSON");
    return;
  }

  // Process the JSON data
  String message = doc["message"];
  server.send(200, "text/plain", "Received message: " + message);
}

void handleGet() {
  // Send a response back to the client
  String response = "Hello from ESP32!";
  server.send(200, "text/plain", response);
}