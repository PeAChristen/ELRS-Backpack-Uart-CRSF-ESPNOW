#include "fake_vrx_fake_trainer.h"
#include "logging.h"

extern MSP recv_msp;

void FakeVRXFakeTrainer::init(const uint8_t* uid)
{
    memcpy(_uid, uid, 6);
}

inline void addLE16(mspPacket_t &pkt, uint16_t v)
{
    pkt.addByte(v & 0xFF);
    pkt.addByte(v >> 8);
}


void FakeVRXFakeTrainer::sendFakeHeadtracking(uint16_t pan, uint16_t roll, uint16_t tilt)
{
    mspPacket_t packet;
    packet.reset();
    packet.makeCommand();
    packet.function = MSP_ELRS_BACKPACK_SET_PTR;

    // Adding channels in little-endian format (LSB first, MSB second)
    addLE16(packet, pan);
    addLE16(packet, roll);
    addLE16(packet, tilt);

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

    packet.addByte(channels[0] & 0xFF); 
    packet.addByte(channels[0] >> 8);   

    packet.addByte(channels[1] & 0xFF);
    packet.addByte(channels[1] >> 8);

    packet.addByte(channels[2] & 0xFF);
    packet.addByte(channels[2] >> 8);

    for(int i = 3; i < NUM_CHANNELS; i++)
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