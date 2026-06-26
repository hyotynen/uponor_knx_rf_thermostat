#pragma once
#include "esphome/components/sensor/sensor.h"
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <Crc16.h>
#include <vector>
#include <string>

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

struct KNXSensorEntry {
    std::string knx_id;
    esphome::sensor::Sensor *sensor{nullptr};
};

class KNXRFGateway : public esphome::Component {
private:
    const char *TAG = "knxrfgateway";
    const int knx_offset = 32;
    byte buffer[400] = {0xFF};
    std::vector<KNXSensorEntry> sensors_;

public:
    float get_setup_priority() const override { return esphome::setup_priority::LATE; }

    void add_sensor(const std::string &knx_id, esphome::sensor::Sensor *s) {
        sensors_.push_back({knx_id, s});
    }

    void setup() override {
        ESP_LOGD(TAG, "Starting setup, %d sensors configured", sensors_.size());

        ELECHOUSE_cc1101.Init();
        ELECHOUSE_cc1101.setCCMode(1);
        ELECHOUSE_cc1101.setModulation(0);
        ELECHOUSE_cc1101.setMHZ(868.3);
        ELECHOUSE_cc1101.setDeviation(47.607422);
        ELECHOUSE_cc1101.setChannel(0);
        ELECHOUSE_cc1101.setChsp(199.951172);
        ELECHOUSE_cc1101.setRxBW(270.833333);
        ELECHOUSE_cc1101.setDRate(32.7301);
        ELECHOUSE_cc1101.setPA(5);
        ELECHOUSE_cc1101.setSyncMode(5);
        ELECHOUSE_cc1101.setSyncWord(0x76, 0x96);
        ELECHOUSE_cc1101.setAdrChk(0);
        ELECHOUSE_cc1101.setAddr(0);
        ELECHOUSE_cc1101.setWhiteData(0);
        ELECHOUSE_cc1101.setPktFormat(0);
        ELECHOUSE_cc1101.setLengthConfig(0);
        ELECHOUSE_cc1101.setPacketLength(61);
        ELECHOUSE_cc1101.setCrc(0);
        ELECHOUSE_cc1101.setCRC_AF(0);
        ELECHOUSE_cc1101.setDcFilterOff(0);
        ELECHOUSE_cc1101.setManchester(0);
        ELECHOUSE_cc1101.setFEC(0);
        ELECHOUSE_cc1101.setPQT(0);
        ELECHOUSE_cc1101.setAppendStatus(1);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FIFOTHR,  0x40);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FSCTRL1,  0x08);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FSCTRL0,  0x00);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_MDMCFG1,  0x22);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_DEVIATN,  0x47);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_MCSM1,    0x30);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_BSCFG,    0x6D);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FOCCFG,   0x2E);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, 0x43);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL1, 0x40);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL0, 0x91);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREND1,   0xB6);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_WORCTRL,  0xFB);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FSCAL3,   0xEF);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FSCAL2,   0x2E);
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FSCAL1,   0x19);
        ESP_LOGD(TAG, "Setup done.");
    }

    void loop() override {
        if (!ELECHOUSE_cc1101.CheckRxFifo(100)) return;
        if (!ELECHOUSE_cc1101.CheckCRC()) return;

        struct KNXDATA knxdata;
        int len = ELECHOUSE_cc1101.ReceiveData(buffer);
        buffer[len] = '\0';
        knxdata.length = len;

        byte pckidx = 0;
        for (int i = 0; i < len; i++) {
            if (i % 2 == 1) {
                knxdata.data[pckidx] = mandecode(buffer[i - 1] * 256 + buffer[i]);
                pckidx++;
            }
        }

        if (len <= 68 || knxdata.data[knx_offset + 1] != 68 || knxdata.data[knx_offset + 2] != 255)
            return;

        int packetLength = (14 + ((knxdata.data[knx_offset] - 9) % 16) +
                           (((knxdata.data[knx_offset] - 9) / 16) * 18)) * 2;
        packetLength = (packetLength >> 1) + knx_offset;
        if (packetLength <= 2) return;

        bool crcError = false;
        uint8_t crcFailed = 0, crcFailIdx = 0;
        uint8_t startIdx = knx_offset, blockIdx = 12;
        Crc16 crc;

        while (startIdx < packetLength) {
            uint32_t crcValue = knxdata.data[min(packetLength - 2, startIdx + blockIdx - 2)];
            crcValue = (crcValue << 8) + knxdata.data[min(packetLength - 1, startIdx + blockIdx - 1)];
            if (((crc.fastCrc(knxdata.data, startIdx, min(packetLength - startIdx - 2, blockIdx - 2),
                              false, false, 0x3D65, 0, 0, 0x8000, 0) ^ 0xFFFF) & 0xFFFF) != crcValue) {
                crcError = true;
                crcFailed |= (1 << crcFailIdx);
            }
            startIdx += blockIdx;
            blockIdx = 18;
            ++crcFailIdx;
        }

        knxdata.crcError = crcError;
        knxdata = parse(knxdata);

        if (knxdata.target_address != 1 || knxdata.temperature <= 16) return;

        for (auto &entry : sensors_) {
            if (entry.sensor && entry.knx_id == knxdata.sensor_id) {
                entry.sensor->publish_state(knxdata.temperature);
                break;
            }
        }
    }

private:
    struct KNXDATA parse(struct KNXDATA knxdata) {
        knxdata.unidirectional   = get_knx_data(knxdata, 3) & 0x1;
        knxdata.serialNoHighWord = get_knx_data(knxdata, 4);
        knxdata.serialNoHighWord = (knxdata.serialNoHighWord << 8) + get_knx_data(knxdata, 5);
        knxdata.serialNoLowWord  = get_knx_data(knxdata, 6);
        knxdata.serialNoLowWord  = (knxdata.serialNoLowWord << 8) + get_knx_data(knxdata, 7);
        knxdata.serialNoLowWord  = (knxdata.serialNoLowWord << 8) + get_knx_data(knxdata, 8);
        knxdata.serialNoLowWord  = (knxdata.serialNoLowWord << 8) + get_knx_data(knxdata, 9);
        sprintf(knxdata.sensor_id, "%04X%08X", knxdata.serialNoHighWord, knxdata.serialNoLowWord);
        knxdata.source_address   = (get_knx_data(knxdata, 13) << 8) + get_knx_data(knxdata, 14);
        knxdata.target_address   = (get_knx_data(knxdata, 15) << 8) + get_knx_data(knxdata, 16);
        knxdata.is_group_address = (get_knx_data(knxdata, 17) & 0x80) >> 7;
        knxdata.max_counter      = (get_knx_data(knxdata, 17) & 0x70) >> 4;
        knxdata.frame_no         = (get_knx_data(knxdata, 17) & 0xE) >> 1;
        knxdata.add_ext_type     = get_knx_data(knxdata, 17) & 0x1;
        knxdata.tpci             = (get_knx_data(knxdata, 18) & 0xC0) >> 6;
        knxdata.seq_number       = get_knx_data(knxdata, 18) & 0x3;
        knxdata.apci             = get_knx_data(knxdata, 19);
        knxdata.sensor_data      = (get_knx_data(knxdata, 20) << 8) + get_knx_data(knxdata, 21);
        knxdata.temperature      = transformTemperature(knxdata.sensor_data) / 100.0;
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
        for (int i = 0; i < 8; i++) {
            int b2 = (a1 >> ((7 - i) * 2)) & 0b11;
            switch (b2) {
                case 0b01: ret = (ret << 1) | 1; break;
                case 0b10: ret = (ret << 1) | 0; break;
            }
        }
        return ret;
    }
};
