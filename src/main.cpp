#if defined(ESP32)
#pragma message "ESP32 stuff happening!"
#else
#pragma message "ESP8266 stuff happening!"
#endif

// ===== Logging Level =====
//#define LOG_LEVEL LOG_LEVEL_INFO   // oder INFO
#define LOG_LEVEL LOG_LEVEL_INFO
//#define LOG_LEVEL LOG_LEVEL_DEBUG
#include "logging.h"

// ======================================================
// 1. Includes
// ======================================================

#include <terseCRSF.h>  // https://github.com/zs6buj/terseCRSF   use v 0.0.6 or later
#include "msp.h"   // MSP Paket Handling
#include "msptypes.h"
#include "fake_vrx_fake_trainer.h"
#include <math.h>
#include "application.h"
#include "types.h"
#include <Adafruit_NeoPixel.h> // https://www.oceanlabz.in/esp32-s3-ws2812-rgb-led-with-arduino-ide-2/


// TODO Add Led Strip for visual feedback, e.g. on connection status, mode, etc.
// =======================================================
// LED Configuration
// =======================================================
#define LED_PIN 21         // Pin connected to the LED strip
#define NUM_LEDS 1         // Number of LEDs in the strip
#define LED_BRIGHTNESS 50 // Brightness (0-255)
#define LED_TYPE NEO_GRB + NEO_KHZ800
#define LED_GREEN 255, 0, 0
#define LED_RED 0, 255, 0
#define LED_YELLOW 150, 255, 0
#define LED_OFF 0, 0, 0
#define LED_BLINK_INTERVAL 500 // Blink interval in milliseconds
#define LED_CONNECTION_TIMEOUT 5000 // Time in milliseconds to consider connection lost
#define LED_UART_TIMEOUT 2000 // Time in milliseconds to consider UART data lost

static uint8_t cur_r = 0, cur_g = 0, cur_b = 0;

bool connectionGood = false;
bool dataReceived = false;
bool dataLost = false; // Flag to indicate if data is considered lost due to timeout


// LED Solid Red: No connection and no data received.
// LED Blinking Red: No connection but data received (e.g. from UART).
// LED Solid Green: Connection established and good data received.
// LED Blinking Yellow: Connection established but no data received (e.g. UART data missing)

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void initLed() {
    strip.begin();
    strip.setBrightness(LED_BRIGHTNESS);
    strip.show(); // Initialize all pixels to 'off'
}

void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
    if (r == cur_r && g == cur_g && b == cur_b) return; // ingen ändring, skippa show()
    cur_r = r; cur_g = g; cur_b = b;
    strip.setPixelColor(0, strip.Color(r, g, b));
    strip.show(); // anropas bara vid faktisk förändring
}

void data_timeoutCheck() {
    static unsigned long lastDataTime = 0;

    if (dataReceived) {
        lastDataTime = millis();
        dataLost = false; // Reset data lost flag when new data is received
    } else if (!dataLost && millis() - lastDataTime > LED_UART_TIMEOUT) {
        dataLost = true; // Set flag to indicate data is considered lost
    }

}


void updateLedStatus(bool connectionGood, bool dataReceived, bool blinkState) {
    if (!connectionGood && !dataReceived) {
        setLedColor(LED_RED); // Solid Red
    } else if (!connectionGood && dataReceived) {
        //etLedColor(LED_RED); // Blinking Red
        // Implement blinking logic in the main loop
        if(blinkState) {
            setLedColor(LED_RED); // Blinking Red
        } else {
            setLedColor(LED_OFF); // Turn off LED during "off" phase of blinking
        }
    } else if (connectionGood && dataReceived) {
        setLedColor(LED_GREEN); // Solid Green
    } else if (connectionGood && !dataReceived) {
        // setLedColor(LED_YELLOW); // Blinking Yellow
        // Implement blinking logic in the main loop
        if(blinkState) {
            setLedColor(LED_YELLOW); // Blinking Yellow
        } else {
            setLedColor(LED_OFF); // Turn off LED during "off" phase of blinking
        }
    }
}

void handleLedBlinking() {
    static bool blinkState = false;
    static unsigned long lastBlinkTime = 0;
    
    if (millis() - lastBlinkTime >= LED_BLINK_INTERVAL) {
        blinkState = !blinkState;
        lastBlinkTime = millis();
    }

    data_timeoutCheck(); // Check for data timeout and update dataLost flag

    updateLedStatus(connectionGood, !dataLost, blinkState);

}


// ======================================================
// 2. Platform Selection (ESP32 / ESP8266)
// ======================================================

#if defined(ESP32)
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#else
#include <ESP8266WiFi.h>
#include <espnow.h>
#endif

enum RadioMode
{
    MODE_RX_ONLY = 0,
    MODE_TX_ONLY = 1,
    MODE_BOTH    = 2
};

// ======================================================
// 3. Configuration (Defines, UID, WiFi, Logging)
// ======================================================

// Please use ExpressLRS Configurator Runtime Options to obtain your UID (unique MAC hashed 
// from binding_phrase) Insert the six numbers between the curly brackets below
//For ESP NOW / ELRS Backpack you need to enter the UID resulting from hashing your 
//secret binding phrase. This must be obtained by launching https://expresslrs.github.io/web-flasher/, 
//enter your binding phrase, then make a note of the UID. Enter the 6 numbers between the commas
// Config Elrs Binding
//uint8_t UID[6] = {0,0,0,0,0,0}; // this is my UID. You have to change it to your once, should look 
//uint8_t UID[6] = {106,19,19,206,193,30};
uint8_t UID[6] = {48,183,109,106,23,63};


// ======================================================
// RadioMode Configuration
// ======================================================
//
// MODE_RX_ONLY  : Only receive ESP-NOW data (telemetry).
//                 Sending (trainer/VTX channels) is disabled.
//
// MODE_TX_ONLY  : Only send ESP-NOW data (trainer/VTX channels).
//                 Receiving telemetry is disabled.
//
// MODE_BOTH     : Send and receive ESP-NOW data.
//                 Full bidirectional operation.
//
// You can change the default mode below.
// It can also be changed later via Serial commands.
// ======================================================
RadioMode radioMode = MODE_TX_ONLY;   // Default startup mode

// Time window (in milliseconds) after boot during which
// the RadioMode can be changed via Serial commands.
// After this time, the mode becomes fixed.
const unsigned long CONFIG_WINDOW_MS = 5000;  // 5 seconds

// ===== AP Wifi Config =====
const char* ssid = "Backpack_ELRS_Crsf";           // SSID des Access Points
const char* password = "12345678";                  // Passwort des Access Points

// ===== Config for Channel output =====
const uint8_t NUM_CHANNELS = 16;
#define CHANNEL_MIN 172
#define CHANNEL_MAX 1811
// ===== in the Example there is a wave for the output channels that you can see the channes are moving
#define WAVE_SPEED 0.05
float wavePhase = 0.0;
#define STEP_INTERVAL 100   // 100ms Offset for each channel when the wave starts to the channel bevor

// ======================================================
// 4. Global Objects
// ======================================================


unsigned long configWindowStart = 0;
uint16_t rampChannels[NUM_CHANNELS];
uint32_t lastStepTime = 0;
uint8_t activeChannel = 0;
bool rampUp = true;

volatile bool espnow_received = false;
volatile uint16_t espnow_len = 0;
uint8_t espnow_buffer[250];
volatile uint16_t crsf_len = 0;

// ======================================================
// Platform Specific
// ======================================================
#if defined(ESP32)
QueueHandle_t rxqueue;
#endif

// ======================================================
// Module Instances
// ======================================================
FakeVRXFakeTrainer vrxModule;
MSP recv_msp;
CRSF crsf;

// ======================================================
// Uart for CRSF
// ======================================================

int16_t uart_pitch = 0;
int16_t uart_roll = 0;    
int16_t uart_yaw = 0;

#define DTQSYS_UART_PORT 1          // Adjust to your ESP32-S3 UART port
#define DTQSYS_RX_PIN 7           // Adjust to your ESP32-S3 pin
#define DTQSYS_TX_PIN -1           // Not needed (RX only)
#define DTQSYS_BAUD 420000         // CRSF standard baud rate
#define DTQSYS_INVERT false        // CRSF does not use inverted signals

CRSF crsf_uart;
HardwareSerial dtqsysSerial(DTQSYS_UART_PORT);

inline uint16_t getCrsfChannel(const uint8_t *payload, int ch)
{
    int bitIndex = ch * 11;
    int byteIndex = bitIndex / 8;
    int bitOffset = bitIndex % 8;

    uint32_t value =
        ((uint32_t)payload[byteIndex]) |
        ((uint32_t)payload[byteIndex + 1] << 8) |
        ((uint32_t)payload[byteIndex + 2] << 16);

    return (uint16_t)((value >> bitOffset) & 0x7FF);
}

// ===============================
// RC Channels
// ===============================
uint16_t rc_channels[NUM_CHANNELS] = {0};

// ===============================
// Telemetry Values
// ===============================
int16_t hud_bat1_volts = 0;
int16_t hud_bat1_amps  = 0;
uint16_t hud_bat1_mAh  = 0;

bool motArmed = false;

// ===============================
// GPS Struct
// ===============================

Location hom = {0,0,0,0,0};
Location cur = {0,0,0,0,0};

// ===============================
bool finalHomeStored = false;

// ======================================================
// ESP-NOW Receive Callback
// ======================================================
#if defined(ESP32)
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
#endif

    if (len <= 8) return;

    crsf_len = len - 8;

    memcpy(crsf.crsf_buf, incomingData + 8, crsf_len);

    espnow_received = true;
}

// ======================================================
// ESP-NOW Send Callback
// ======================================================

#if defined(ESP32)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Beim ESP32 ist status 0 (ESP_NOW_SEND_SUCCESS)
    bool success = (status == ESP_NOW_SEND_SUCCESS);
#else
void OnDataSent(uint8_t *mac_addr, uint8_t status) {
    // Beim ESP8266 ist status 0 meist Erfolg
    bool success = (status == 0);
#endif

    if (success) {
        LOG_DEBUG_INLINE("ESP-NOW Send OK          ");
        connectionGood = true;
    } else {
        LOG_ERROR_INLINE("ESP-NOW Send FAIL (Status: %d) t=%lu ms      ", status, millis());
        connectionGood = false;
    }
}

// ======================================================
// 7. Setup()
// ======================================================

void initSerial()
{
    Serial.begin(115200);
    LOG_INFO("Start");
}

void initUartSerial()
{
    delay(1000);
    dtqsysSerial.begin(DTQSYS_BAUD, SERIAL_8N1, DTQSYS_RX_PIN, DTQSYS_TX_PIN);
    LOG_INFO("UART Serial for CRSF initialized"); 
    crsf_uart.initialise(dtqsysSerial);
    delay(1000);
}

void initWiFi()
{
    UID[0] &= ~0x01;   // unicast fix
    WiFi.mode(WIFI_STA);
    //WiFi.mode(WIFI_AP_STA); // Ermöglicht AP und Station gleichzeitig
    WiFi.disconnect();
    #if defined(ESP32)
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    #else
        wifi_set_channel(1);
    #endif
}

void initMac()
{
#if defined(ESP32)
    esp_wifi_set_mac(WIFI_IF_STA, UID);
#else
    wifi_set_macaddr(STATION_IF, UID);
#endif
}

void initESPNow()
{
    #if defined(ESP32)
        if (esp_now_init() != ESP_OK)
    #else
        if (esp_now_init() != 0)
    #endif
    {
        LOG_ERROR("ESP-NOW init failed");
        return;
    }

    esp_now_register_send_cb(OnDataSent);
    //esp_now_register_recv_cb(OnDataRecv); // Trying without receive callback, as we only want to send data in this example. Receiving is disabled to save resources and avoid potential issues with queue handling.

    #if defined(ESP32)
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, UID, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
            LOG_ERROR("Peer add failed");
    #else
        esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
        esp_now_add_peer(UID, ESP_NOW_ROLE_COMBO, 0, NULL, 0);
    #endif
}

void initRamp()
{
    for(int i = 0; i < NUM_CHANNELS; i++)
        rampChannels[i] = CHANNEL_MIN;
}

void initESP32Queue()
{
#if defined(ESP32)
    rxqueue = xQueueCreate(20, sizeof(mspPacket_t));

    if (rxqueue == NULL)
    {
        LOG_ERROR("Queue creation failed");
    }
#endif
}

void initInfo()
{
    LOG_INFO("==================================================");
    LOG_INFO("Radio Mode Configuration");
    LOG_INFO("Viewpoint: ESP Module Role (not the RC transmitter)");
    LOG_INFO("--------------------------------------------------");
    LOG_INFO("RX_ONLY : ESP receives ESP-NOW data and decodes it.");
    LOG_INFO("          Sending is disabled.");
    LOG_INFO("TX_ONLY : ESP sends ESP-NOW data.");
    LOG_INFO("          Receiving/decoding is disabled.");
    LOG_INFO("BOTH    : Full bidirectional operation.");
    LOG_INFO("--------------------------------------------------");
    LOG_INFO("Config window active for %lu ms", CONFIG_WINDOW_MS);
    LOG_INFO("Change mode via Serial during this window:");
    LOG_INFO("  1 = RX_ONLY");
    LOG_INFO("  2 = TX_ONLY");
    LOG_INFO("  3 = BOTH");
    LOG_INFO("==================================================");

    configWindowStart = millis();
}

void setup() {
    initSerial();
    initWiFi();
    initMac();
    initESP32Queue();
    initESPNow();
    vrxModule.init(UID);
    initUartSerial();
    //initRamp();
    //initInfo();
    initLed();
}

// ======================================================
// 8. Loop()
// ======================================================

void loop()
{


    // ======================================================
    // Handle LED Blinking
    // ======================================================
    handleLedBlinking();


    // ======================================================
    // Serial Mode Switch (only during config window)
    // ======================================================
    /*
    if (millis() - configWindowStart < CONFIG_WINDOW_MS)
    {
        if (Serial.available())
        {
            char c = Serial.read();

            // Nur echte Ziffern akzeptieren
            if (c >= '1' && c <= '3')
            {
                radioMode = (RadioMode)(c - '1');

                LOG_INFO("RadioMode changed to %d", radioMode + 1);
            }
        }
    }
    */

    // ======================================================
    // ESP-NOW Receive Handling (CRSF)
    // ======================================================
    /*
    if (radioMode != MODE_TX_ONLY)
    {
        if (espnow_received)
        {
            noInterrupts();
            espnow_received = false;
            interrupts();

            processCRSFFrame(crsf.crsf_buf, crsf_len);
        }
    }
    */

    // ======================================================
    // Regular Mode
    // ======================================================
    if (radioMode != MODE_RX_ONLY)
    {
        // Read CRSF data from UART
        if(dtqsysSerial.available()){
            dataReceived = true; // Set flag to indicate data reception
            if (crsf_uart.readCrsfFrame(crsf_uart.frame_lth))
            {
                    
                uint8_t frame_id = crsf_uart.decodeTelemetry(&*crsf_uart.crsf_buf, crsf_uart.frame_lth);
                
                if (frame_id == 0x16) // CHANNELS_ID
                {
                    const uint8_t *payload = &crsf_uart.crsf_buf[3];
                    
                    vrxModule.sendHeadtracking(payload);

                    //Serial.print("CRSF HT Channels in buffer: ");
                    
                    //uart_pitch = getCrsfChannel(payload,0); // kanal 3 = pitch
                    //uart_roll = getCrsfChannel(payload,1);  // kanal 4 = roll
                    //uart_yaw = getCrsfChannel(payload,2);  // kanal 5 = yaw
                                        
                    //Serial.printf("Pitch: %d, Roll: %d, Yaw: %d\n", uart_pitch, uart_roll, uart_yaw);
                    
                    //vrxModule.sendFakeHeadtracking(uart_pitch, uart_roll, uart_yaw);

                    //Serial.println();
                }
            }
        }else{
            dataReceived = false; // No data available
        }

        // Send headtracking data via ESP-NOW
        //vrxModule.sendFakeHeadtracking(uart_pitch, uart_roll, uart_yaw);   // ESP-NOW senden

        // Send trainer/VTX channels via ESP-NOW
        // vrxModule.updateChannelRamp();   // ESP-NOW senden
        
        //vrxModule.sendTrainerMode16ch(rampChannels);
    }
}