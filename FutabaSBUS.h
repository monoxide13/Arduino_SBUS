#ifndef __FUTABA_S_SBUS_H__
#define __FUTABA_S_SBUS_H__

#include <Arduino.h>

#define SBUS_BAUD_RATE 100000

#define CHANNELS        18
#define BUFFER_LENGTH   25
#define MAX_READ_ATTEMPTS       32

struct channels {
        int16_t channel1;
        int16_t channel2;
        int16_t channel3;
        int16_t channel4;
        int16_t channel5;
        int16_t channel6;
        int16_t channel7;
        int16_t channel8;
        int16_t channel9;
        int16_t channel10;
        int16_t channel11;
        int16_t channel12;
        int16_t channel13;
        int16_t channel14;
        int16_t channel15;
        int16_t channel16;
        int16_t channel17;
        int16_t channel18;
};

typedef union {
        int16_t data[CHANNELS];
        struct channels channels;
} ChannelData;

class FutabaSBUS {
public:
        FutabaSBUS();
        FutabaSBUS(HardwareSerial& serialPort, bool passThrough = false, uint32_t baud = SBUS_BAUD_RATE, bool fastDecode = true);
#ifdef SoftwareSerial_h
        FutabaSBUS(SoftwareSerial& serialPort, bool passThrough = false, uint32_t baud = SBUS_BAUD_RATE, bool fastDecode = true);
#endif
        ~FutabaSBUS() {}
        
        void begin(HardwareSerial& serialPort, bool passThrough = false, uint32_t baud = SBUS_BAUD_RATE, bool fastDecode = true);
#ifdef SoftwareSerial_h
        void begin(SoftwareSerial& serialPort, bool passThrough = false, uint32_t baud = SBUS_BAUD_RATE, bool fastDecode = true);
#endif

        void attachPassThroughHandler(ChannelData (*callback)(ChannelData data)) {
                passthrough_handler = callback;
        }
        void attachDataReceived(void (*callback)(ChannelData data)) {
                data_received = callback;
        }
        void attachFrameError(void (*callback)()) {
                frame_error_callback = callback;
        }
        void attachFailSafe(void (*callback)()) {
                failsafe_callback = callback;
        }
        void attachRawData(void (*callback)(uint8_t data[BUFFER_LENGTH])) {
                raw_data_callback = callback;
        }
        void detachPassThroughHandler() {
                passthrough_handler = NULL;
        }
        void detachDataReceived() {
                data_received = NULL;
        }
        void detachFrameError() {
                frame_error_callback = NULL;
        }
        void detachFailSafe() {
                failsafe_callback = NULL;
        }
        void detachRawData() {
                raw_data_callback = NULL;
        }
        void receive();
        void send();
        void passThrough(bool passThrough) {
                pass_through = passThrough; 
        }
        void updateChannels(ChannelData channels, bool frameError = false, bool failSafe = false);
        ChannelData getChannels();
        
private:
        bool decode_sbus_data();

        Stream *serial;
        ChannelData channels;
        uint32_t baud_rate;
        uint8_t buffer[BUFFER_LENGTH];
        bool pass_through;
        int offset;
        bool failsafe;
        bool frame_error;
        bool fast_decode;
        void (*data_received)(ChannelData data);
        void (*raw_data_callback)(uint8_t data[BUFFER_LENGTH]);
        void (*frame_error_callback)();
        void (*failsafe_callback)();
        ChannelData (*passthrough_handler)(ChannelData data);
};

#endif
