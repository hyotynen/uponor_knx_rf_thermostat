// Stack Size needs to be increased to avoid carshing: https://github.com/esphome/issues/issues/855
//
// EDIT:    .platformio/packages/framework-arduinoespressif32/cores/esp32/main.cpp
// CHANGE:  xTaskCreateUniversal(loopTask, "loopTask", ARDUINO_LOOP_STACK_SIZE, NULL, 1, &loopTaskHandle, CONFIG_ARDUINO_RUNNING_CORE);
// TO:	    xTaskCreateUniversal(loopTask, "loopTask", 32768, NULL, 1, &loopTaskHandle, CONFIG_ARDUINO_RUNNING_CORE);

#include "esphome.h"
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <Crc16.h>

static const char *TAG = "KNXRFGATEWAY";
byte buffer[400] = {0xFF};

struct KNXDATA {
    byte data[200] = {0};
    int length;
    bool crcError;
    uint8_t unidirectional;
    uint16_t serialNoHighWord;
    uint16_t serialNoLowWord;
    char sensor_id[13] = {0};
    uint16_t source_address;
    uint16_t target_address;
    uint8_t is_group_address;
    uint8_t max_counter;
    uint8_t frame_no;
    uint8_t add_ext_type;
    uint8_t tpci;
    uint8_t seq_number;
    uint8_t apci;
    uint16_t sensor_data;
    double temperature;
};

class KNXRFGateway : public Component, public Sensor {
private:
    int knx_offset = 32;
    std::map<std::string, std::string> ids;

public:
    std::string status = "";

    const char* id1;
    const char* id2;
    const char* id3;
    const char* id4;
    const char* id5;
    const char* id6;
    const char* id7;
    const char* id8;

    // Receive sensor id:s from configuration
    KNXRFGateway(const char* id1, const char* id2, const char* id3, const char* id4, const char* id5, const char* id6, const char* id7, const char* id8):id1(id1),id2(id2),id3(id3),id4(id4),id5(id5),id6(id6),id7(id7),id8(id8) {}

    Sensor *temperature1 = new Sensor();
    Sensor *temperature2 = new Sensor();
    Sensor *temperature3 = new Sensor();
    Sensor *temperature4 = new Sensor();
    Sensor *temperature5 = new Sensor();
    Sensor *temperature6 = new Sensor();
    Sensor *temperature7 = new Sensor();
    Sensor *temperature8 = new Sensor();

    struct KNXDATA parse(struct KNXDATA knxdata){
        knxdata.unidirectional = get_knx_data(knxdata,3) & 0x1;
        knxdata.serialNoHighWord    = get_knx_data(knxdata,4);
        knxdata.serialNoHighWord    = (knxdata.serialNoHighWord << 8) + get_knx_data(knxdata,5);
        knxdata.serialNoLowWord     = get_knx_data(knxdata,6);
        knxdata.serialNoLowWord     = (knxdata.serialNoLowWord << 8) + get_knx_data(knxdata,7);
        knxdata.serialNoLowWord     = (knxdata.serialNoLowWord << 8) + get_knx_data(knxdata,8);
        knxdata.serialNoLowWord     = (knxdata.serialNoLowWord << 8) + get_knx_data(knxdata,9);
        sprintf(knxdata.sensor_id, "%04X%08X", knxdata.serialNoHighWord, knxdata.serialNoLowWord);

        knxdata.source_address = (get_knx_data(knxdata,13) << 8) + get_knx_data(knxdata,14);
        knxdata.target_address = (get_knx_data(knxdata,15) << 8) + get_knx_data(knxdata,16);
        knxdata.is_group_address = (get_knx_data(knxdata,17) & 0x80) >> 7;
        knxdata.max_counter = (get_knx_data(knxdata,17) & 0x70) >> 4;
        knxdata.frame_no = (get_knx_data(knxdata,17) & 0xE) >> 1;
        knxdata.add_ext_type = get_knx_data(knxdata,17) & 0x1;
        knxdata.tpci = (get_knx_data(knxdata,18) & 0xC0) >> 6;
        knxdata.seq_number = get_knx_data(knxdata,18) & 0x3;
        knxdata.apci = get_knx_data(knxdata,19);
        knxdata.sensor_data = (get_knx_data(knxdata,20)<<8)+ get_knx_data(knxdata,21);
        knxdata.temperature  = transformTemperature(knxdata.sensor_data) / 100.0;

        return knxdata;
    }

    byte get_knx_data(struct KNXDATA knxdata, int index) {
        return knxdata.data[knx_offset + index];
    }

    uint16_t transformTemperature(uint16_t data) {
        if (data & 0x800) data = (data & 0x7FF) * 2;
        return data;
    }

    int mandecode(unsigned int a1) {
        int ret = 0;
        for (int i = 0; i < 8; i++)
        {
            int b2;
            b2 = (a1 >> ((7 - i) * 2)) & 0b11;
            long bt;
            bt = (a1 >> ((7 - i) * 2));
            switch (b2)
            {
            case 0b01:
                ret = (ret << 1) | 1;
                break;
            case 0b10:
                ret = (ret << 1) | 0;
                break;
            }
        }

        return ret;
    }

    void setup() override {
        ESP_LOGD(TAG, "Starting setup");
        ELECHOUSE_cc1101.Init();                // must be set to initialize the cc1101!
        ELECHOUSE_cc1101.setCCMode(1);          // set config for internal transmission mode.
        ELECHOUSE_cc1101.setModulation(0);      // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
        ELECHOUSE_cc1101.setMHZ(868.3);        // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
        ELECHOUSE_cc1101.setDeviation(47.607422);   // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.
        ELECHOUSE_cc1101.setChannel(0);         // Set the Channelnumber from 0 to 255. Default is cahnnel 0.
        ELECHOUSE_cc1101.setChsp(199.951172);       // The channel spacing is multiplied by the channel number CHAN and added to the base frequency in kHz. Value from 25.39 to 405.45. Default is 199.95 kHz.
        ELECHOUSE_cc1101.setRxBW(270.833333);       // Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.
        ELECHOUSE_cc1101.setDRate(32.7301);       // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
        ELECHOUSE_cc1101.setPA(5);             // Set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!
        ELECHOUSE_cc1101.setSyncMode(5);       // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
        ELECHOUSE_cc1101.setSyncWord(0x76, 0x96); // Set sync word. Must be the same for the transmitter and receiver. (Syncword high, Syncword low)
        ELECHOUSE_cc1101.setAdrChk(0);          // Controls address check configuration of received packages. 0 = No address check. 1 = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast.
        ELECHOUSE_cc1101.setAddr(0);            // Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
        ELECHOUSE_cc1101.setWhiteData(0);       // Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on.
        ELECHOUSE_cc1101.setPktFormat(0);       // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.
        ELECHOUSE_cc1101.setLengthConfig(0);    // 0 = Fixed packet length mode. 1 = Variable packet length mode. 2 = Infinite packet length mode. 3 = Reserved
        ELECHOUSE_cc1101.setPacketLength(61);    // Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed.
        ELECHOUSE_cc1101.setCrc(0);             // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.
        ELECHOUSE_cc1101.setCRC_AF(0);          // Enable automatic flush of RX FIFO when CRC is not OK. This requires that only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size.
        ELECHOUSE_cc1101.setDcFilterOff(0);     // Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud The recommended IF frequency changes when the DC blocking is disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity).
        ELECHOUSE_cc1101.setManchester(0);      // Enables Manchester encoding/decoding. 0 = Disable. 1 = Enable.
        ELECHOUSE_cc1101.setFEC(0);             // Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. 0 = Disable. 1 = Enable.
        ELECHOUSE_cc1101.setPQT(0);             // Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted.
        ELECHOUSE_cc1101.setAppendStatus(1);    // When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK.
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FIFOTHR,  0x40);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FSCTRL1,  0x08);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FSCTRL0,  0x00);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_MDMCFG1,  0x22);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_DEVIATN,  0x47);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_MCSM1,  0x30);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_BSCFG,  0x6D);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FOCCFG,  0x2E);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2,  0x43);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL1,  0x40);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL0,  0x91);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREND1,  0xB6);

        ELECHOUSE_cc1101.SpiWriteReg(CC1101_WORCTRL,  0xFB);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FSCAL3,  0xEF);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FSCAL2,  0x2E);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FSCAL1,  0x19);
        ESP_LOGD(TAG, "Setup done.");
    }

    void loop() override {
        if (ELECHOUSE_cc1101.CheckRxFifo(100)) {

            //CRC Check. If "setCrc(false)" crc returns always OK!
            if (ELECHOUSE_cc1101.CheckCRC()) {
                struct KNXDATA knxdata;

                //Get received Data and calculate length
                int len = ELECHOUSE_cc1101.ReceiveData(buffer);
                buffer[len] = '\0';
                knxdata.length = len;
                byte pckidx = 0;
                for (int i = 0 ; i < len; i++) {
                    if (i % 2 == 1) {
                        knxdata.data[pckidx] = mandecode(buffer[i - 1] * 256 + buffer[i]);
                        pckidx++;
                    }
                }
                uint8_t crcError = true, crcFailIdx = 0;
                uint32_t crcValue, serialNoLowWord = 0;
                uint8_t crcFailed, startIdx, blockIdx;
                uint16_t packetLength = 0, serialNoHighWord = 0;
                Crc16 crc;

                //KNX frame
                if (len > 68 &&  knxdata.data[knx_offset + 1] == 68 &&  knxdata.data[knx_offset+2] == 255) {

                    int packetLength = (14+((knxdata.data[knx_offset]-9)%16)+(((knxdata.data[knx_offset]-9)/16)*18))*2;

                    packetLength  = (packetLength >> 1) + knx_offset;
                    if (packetLength > 2) {
                        crcError = false;
                        startIdx = knx_offset;
                        blockIdx = 12;
                        crcFailIdx = 0;
                        crcFailed = 0;

                    while ((startIdx<packetLength)) {
                            crcValue = knxdata.data[min(packetLength-2,startIdx+blockIdx-2)];
                            crcValue = (crcValue<<8)+knxdata.data[min(packetLength-1,startIdx+blockIdx-1)];
                            if (((crc.fastCrc(knxdata.data,startIdx,min(packetLength-startIdx-2,blockIdx-2),false,false,0x3D65,0,0,0x8000,0)^0xFFFF)&0xFFFF)!=crcValue) {
                                crcError = true;
                                crcFailed = crcFailed | (1<<crcFailIdx);
                            }
                            startIdx +=blockIdx;
                            blockIdx = 18;
                            ++crcFailIdx;
                        }
                    }
                    knxdata.crcError = crcError;

                    knxdata = parse(knxdata);

                    // Only publish when temperature is received and is above threshold to filter outliers
                    if (knxdata.target_address == 1 && knxdata.temperature > 16) {
                        if (!strcmp(knxdata.sensor_id,id1)) {
                            temperature1->publish_state(knxdata.temperature);
                        }
                        if (!strcmp(knxdata.sensor_id,id2)) {
                            temperature2->publish_state(knxdata.temperature);
			                  }
                        if (!strcmp(knxdata.sensor_id,id3)) {
                            temperature3->publish_state(knxdata.temperature);
			                  }
                        if (!strcmp(knxdata.sensor_id,id4)) {
                            temperature4->publish_state(knxdata.temperature);
			                  }
                        if (!strcmp(knxdata.sensor_id,id5)) {
                            temperature5->publish_state(knxdata.temperature);
			                  }
                        if (!strcmp(knxdata.sensor_id,id6)) {
                            temperature6->publish_state(knxdata.temperature);
			                  }
                        if (!strcmp(knxdata.sensor_id,id7)) {
                            temperature7->publish_state(knxdata.temperature);
			                  }
                        if (!strcmp(knxdata.sensor_id,id8)) {
                            temperature8->publish_state(knxdata.temperature);
			                  }
                    }
                }
            }
        }
    }
};
