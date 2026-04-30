#include "fake_vrx_fake_trainer.h"
#include "logging.h"
#include <terseCRSF.h>

extern MSP recv_msp;

// DTQSYS Head Tracker UART Configuration
// Reference: https://headtracker.gitbook.io/head-tracker-v2.2/settings/gui-settings/uart
#define DTQSYS_UART_PORT 1
#define DTQSYS_RX_PIN 18           // Adjust to your ESP32-S3 pin
#define DTQSYS_TX_PIN -1           // Not needed (RX only)
#define DTQSYS_BAUD 420000         // CRSF standard baud rate
#define DTQSYS_INVERT false        // Standard CRSF (not inverted)
#define DTQSYS_UPDATE_RATE_HZ 140  // DTQSYS sends at 140Hz

CRSF crsf;                         // terseCRSF instance from local lib
HardwareSerial dtqsysSerial(DTQSYS_UART_PORT);

void FakeVRXFakeTrainer::init(const uint8_t* uid)
{
    memcpy(_uid, uid, 6);
    
    // Initialize UART for DTQSYS Head Tracker
    dtqsysSerial.begin(DTQSYS_BAUD, SERIAL_8N1, DTQSYS_RX_PIN, DTQSYS_TX_PIN, DTQSYS_INVERT);
    crsf.initialise(dtqsysSerial);
    
    LOG_INFO("DTQSYS Head Tracker CRSF initialized");
    LOG_INFO("UART%d @ %d baud, RX pin %d, %d Hz update rate", 
             DTQSYS_UART_PORT, DTQSYS_BAUD, DTQSYS_RX_PIN, DTQSYS_UPDATE_RATE_HZ);
}

void FakeVRXFakeTrainer::readAndSendHeadtracking()
{
    // terseCRSF reads and parses CRSF frames automatically
    // At 140Hz, frames arrive every ~7.14ms
    if (crsf.readCrsfFrame(crsf.frame_lth))
    {
        // Decode telemetry frames (ATTITUDE_ID = 0x1E for head tracking)
        uint8_t frame_id = crsf.decodeTelemetry(&crsf.crsf_buf[0], crsf.frame_lth);
        
        if (frame_id == ATTITUDE_ID)
        {
            // DTQSYS sends attitude data (pitch, roll, yaw) in CRSF frames
            // The library stores decoded values in:
            // - attiF_pitch: in degrees
            // - attiF_roll: in degrees  
            // - attiF_yaw: in degrees (0-359)
            
            // Convert from degrees to PWM range (1000-2000)
            // Assuming ±90 degrees maps to ±500 PWM units from center (1500)
            // So: 500 units / 90 degrees ≈ 5.556 units/degree
            
            uint16_t pan = (uint16_t)((crsf.attiF_yaw * 5.556f) + 1500);    // yaw -> pan
            uint16_t roll = (uint16_t)((crsf.attiF_roll * 5.556f) + 1500);  // roll -> roll
            uint16_t tilt = (uint16_t)((crsf.attiF_pitch * 5.556f) + 1500); // pitch -> tilt
            
            // Constrain to safe PWM range
            pan = constrain(pan, 1000, 2000);
            roll = constrain(roll, 1000, 2000);
            tilt = constrain(tilt, 1000, 2000);
            
            sendHeadtrackingViaMSP(pan, roll, tilt);
        }
    }
}

void FakeVRXFakeTrainer::sendHeadtrackingViaMSP(uint16_t pan, uint16_t roll, uint16_t tilt)
{
    mspPacket_t packet;
    packet.reset();
    packet.makeCommand();
    packet.function = MSP_ELRS_BACKPACK_SET_PTR;

    packet.addByte(pan & 0xFF);
    packet.addByte(pan >> 8);

    packet.addByte(roll & 0xFF);
    packet.addByte(roll >> 8);

    packet.addByte(tilt & 0xFF);
    packet.addByte(tilt >> 8);

    uint8_t buf[64];
    uint8_t size = recv_msp.convertToByteArray(&packet, buf);

    int result = esp_now_send(_uid, buf, size);

    if (result != 0)
        LOG_WARN("esp_now_send failed (%d)", result);
}

void FakeVRXFakeTrainer::sendFakeHeadtracking(uint16_t pan, uint16_t roll, uint16_t tilt)
{
    mspPacket_t packet;
    packet.reset();
    packet.makeCommand();
    packet.function = MSP_ELRS_BACKPACK_SET_PTR;

    packet.addByte(pan & 0xFF);
    packet.addByte(pan >> 8);

    packet.addByte(roll & 0xFF);
    packet.addByte(roll >> 8);

    packet.addByte(tilt & 0xFF);
    packet.addByte(tilt >> 8);

    uint8_t buf[64];
    uint8_t size = recv_msp.convertToByteArray(&packet, buf);

    int result = esp_now_send(_uid, buf, size);

    if (result != 0)
        LOG_WARN("esp_now_send failed (%d)", result);
}

void FakeVRXFakeTrainer::sendTrainerMode16ch(uint16_t *channels)
{
    mspPacket_t packet;
    packet.reset();
    packet.makeCommand();
    packet.function = MSP_ELRS_BACKPACK_SET_PTR;

    for(int i = 0; i < NUM_CHANNELS; i++)
    {
        packet.addByte(channels[i] & 0xFF);
        packet.addByte(channels[i] >> 8);
    }

    uint8_t buf[64];
    uint8_t size = recv_msp.convertToByteArray(&packet, buf);

    int result = esp_now_send(_uid, buf, size);

    if (result != 0)
        LOG_WARN("esp_now_send failed (%d)", result);
}

void FakeVRXFakeTrainer::updateChannelRamp()
{
    uint32_t now = millis();

    if (now - lastStepTime < STEP_INTERVAL)
        return;

    lastStepTime = now;

    for(int i = 0; i < NUM_CHANNELS; i++)
    {
        float phase = wavePhase + i * 0.4f;
        float s = sin(phase);
        float normalized = (s + 1.0f) * 0.5f;

        rampChannels[i] = CHANNEL_MIN +
            normalized * (CHANNEL_MAX - CHANNEL_MIN);
    }

    wavePhase += WAVE_SPEED;

    sendTrainerMode16ch(rampChannels);
}
