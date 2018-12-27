#include "SBUS.h"


FutabaSBUS::FutabaSBUS() : serial(NULL), baud_rate(SBUS_BAUD_RATE), pass_through(false), offset(0), failsafe(false), frame_error(false), fast_decode(true), data_received(NULL), raw_data_callback(NULL), frame_error_callback(NULL), failsafe_callback(NULL), passthrough_handler(NULL) {
}

FutabaSBUS::FutabaSBUS(HardwareSerial& serialPort, bool passThrough, uint32_t baud, bool fastDecode) : serial(&serialPort), baud_rate(baud), pass_through(passThrough), offset(0), failsafe(false), frame_error(false), fast_decode(fastDecode), data_received(NULL), raw_data_callback(NULL), frame_error_callback(NULL), failsafe_callback(NULL), passthrough_handler(NULL) {
        serialPort.begin(baud_rate, SERIAL_8E2);
}

#ifdef SoftwareSerial_h
FutabaSBUS::FutabaSBUS(SoftwareSerial& serialPort, bool passThrough, uint32_t baud, bool fastDecode) : serial(&serialPort), baud_rate(baud), pass_through(passThrough), offset(0), failsafe(false), frame_error(false), fast_decode(fastDecode), data_received(NULL), raw_data_callback(NULL), frame_error_callback(NULL), failsafe_callback(NULL), passthrough_handler(NULL) {
        serialPort.begin(baud_rate, SERIAL_8E2);
}
#endif

void FutabaSBUS::begin(HardwareSerial& serialPort, bool passThrough, uint32_t baud, bool fastDecode) {
        pass_through = passThrough;
        baud_rate = baud;
        fast_decode = fastDecode;
        
        serialPort.begin(baud_rate, SERIAL_8E2);
        serial = &serialPort;
}

#ifdef SoftwareSerial_h
void FutabaSBUS::begin(SoftwareSerial& serialPort, bool passThrough, uint32_t baud, bool fastDecode) {
        pass_through = passThrough;
        baud_rate = baud;
        fast_decode = fastDecode;
        
        serialPort.begin(baud_rate, SERIAL_8E2);
        serial = &serialPort;
}
#endif

ChannelData FutabaSBUS::getChannels() {
        return channels;
}

void FutabaSBUS::receive() {
        int data;
        uint8_t counter = 0;
        
        if (!serial) return;

        while (serial->available() > 0 && offset < PACKET_LENGTH && counter < MAX_READ_ATTEMPTS) {
				// If offset is more than packet length, skip reading and go right to decoding.
                counter++;
                data = serial->read();
                
                // We need a start byte. Ignore everything else
                if (offset == 0 && data != 0x0f)
                        continue;

                buffer[offset++] = (data & 0xff);
        }
        
        // We have a full S-BUS packet, decode it
        if (offset == PACKET_LENGTH) {
                if (raw_data_callback)
                        raw_data_callback(buffer);

                // If decode was successfull call the installed callbacks
                if (decode_sbus_data()) {
                        if (frame_error && frame_error_callback)
                                frame_error_callback();
                        
                        if (failsafe && failsafe_callback)
                                failsafe_callback();
                        
                        if (!failsafe && !frame_error && data_received) 
                                data_received(channels);
                }

                if (pass_through) {
                        if (passthrough_handler) 
                                updateChannels(passthrough_handler(channels), false, false);
                        
                        send();
                }
                
                // Reset data structures
                for (uint8_t i = 0; i < PACKET_LENGTH; i++) {
                        buffer[i] = 0;
                }
                offset = 0;
        }
}

void FutabaSBUS::send() {
        if (!serial) return;
        for (uint8_t i = 0; i < PACKET_LENGTH; i++)
                serial->write(buffer[i]);
}

bool FutabaSBUS::decode_sbus_data() {
        uint8_t byte_in_sbus = 1, bit_in_sbus = 0, ch = 0, bit_in_channel = 0;
        
        if (buffer[0] != 0x0f || buffer[PACKET_LENGTH - 1] != 0x00)
                return false;
		
        for (uint8_t i = 0; i < CHANNELS; i++) channels.data[i] = 0;
        
	/* Decode the 16 channels */
        if (fast_decode) {
                channels.data[0]  = ((buffer[1] | buffer[2] << 8) & 0x07FF);
                channels.data[1]  = ((buffer[2] >> 3 | buffer[3] << 5) & 0x07FF);
                channels.data[2]  = ((buffer[3] >> 6 | buffer[4] << 2 | buffer[5] << 10) & 0x07FF);
                channels.data[3]  = ((buffer[5] >> 1 | buffer[6] << 7) & 0x07FF);
                channels.data[4]  = ((buffer[6] >> 4 | buffer[7] << 4) & 0x07FF);
                channels.data[5]  = ((buffer[7] >> 7 | buffer[8] << 1 | buffer[9] << 9) & 0x07FF);
                channels.data[6]  = ((buffer[9] >> 2 | buffer[10] << 6) & 0x07FF);
                channels.data[7]  = ((buffer[10] >> 5 | buffer[11] << 3) & 0x07FF);
                channels.data[8]  = ((buffer[12] | buffer[13] << 8) & 0x07FF);
                channels.data[9]  = ((buffer[13] >> 3 | buffer[14] << 5) & 0x07FF);
                channels.data[10] = ((buffer[14] >> 6 | buffer[15] << 2 | buffer[16] << 10) & 0x07FF);
                channels.data[11] = ((buffer[16] >> 1 | buffer[17] << 7) & 0x07FF);
                channels.data[12] = ((buffer[17] >> 4 | buffer[18] << 4) & 0x07FF);
                channels.data[13] = ((buffer[18] >> 7 | buffer[19] << 1 | buffer[20] << 9) & 0x07FF);
                channels.data[14] = ((buffer[20] >> 2 | buffer[21] << 6) & 0x07FF);
                channels.data[15] = ((buffer[21] >> 5 | buffer[22] << 3) & 0x07FF);
        } else {
                for (uint8_t i = 0; i < 176; i++) {
                        if (buffer[byte_in_sbus] & (1 << bit_in_sbus)) {
                                channels.data[ch] |= (1 << bit_in_channel);
                        }
          
                        bit_in_sbus++;
                        bit_in_channel++;

                        if (bit_in_sbus == 8) {
                                bit_in_sbus =0;
                                byte_in_sbus++;
                        }
          
                        if (bit_in_channel == 11) {
                                bit_in_channel =0;
                                ch++;
                        }
                }
        }
        
         /* Two digi channels */       
        channels.channels.channel17 = (buffer[23] & (1 << 0)) != 0; 
        channels.channels.channel18 = (buffer[23] & (1 << 1)) != 0; 
                
        /* Signal lost (frame error) and failsafe status */
        frame_error = (buffer[23] & (1 << 2)) != 0;
        failsafe = (buffer[23] & (1 << 3)) != 0;
        
        return true;
}


void FutabaSBUS::updateChannels(ChannelData channels, bool frameError, bool failSafe) {
        uint8_t ch = 0, bit_in_servo = 0, byte_in_sbus = 1, bit_in_sbus = 0;

	for (uint8_t i = 0; i < PACKET_LENGTH; i++) buffer[i] = 0;

        /* Start and end byte */
        buffer[0] = 0x0f;
        buffer[24] = 0x00;

        /* Channel data */
        for (uint8_t i = 0; i < 176; i++) {
                if (channels.data[ch] & (1 << bit_in_servo)) {
                        buffer[byte_in_sbus] |= (1 << bit_in_sbus);
                }
                bit_in_sbus++;
                bit_in_servo++;

                if (bit_in_sbus == 8) {
                        bit_in_sbus =0;
                        byte_in_sbus++;
                }
                
                if (bit_in_servo == 11) {
                        bit_in_servo =0;
                        ch++;
                }
        }
        
        /* Digi channels */
        if (channels.channels.channel17 != 0)
                buffer[23] |= (1 << 0);

        if (channels.channels.channel18 != 0)
                buffer[23] |= (1 << 1);
        
        /* Signal lost and failsafe */
        if (frameError)
                buffer[23] |= (1 << 2);
        
        if (failSafe)
                buffer[23] |= (1 << 3);

}

//  1111 1111 | 1112 2222 | 2222 2233 | 3333 3333 | 3444 4444 | 4444 5555 | 5555 5556 | 6666 6666 | 6677 7777 | 7777 7888 | 8888 8888 |
//      1           2           3           4           5           6           7           8           9           a           b 
