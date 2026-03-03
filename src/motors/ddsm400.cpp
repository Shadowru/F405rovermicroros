#pragma once
#include <Arduino.h>

// Протокол DDSM400 (UART mode)
// Команда скорости: [ID, 0x64, speed_h, speed_l, 0, 0, 0, CRC]
// Запрос данных:    [ID, 0x74, 0, 0, 0, 0, 0, CRC]
// Ответ:           [ID, 0x74, speed_h, speed_l, current_h, current_l, encoder_h, encoder_l, ...]

class DDSM400 {
public:
    DDSM400(HardwareSerial &serial) : _serial(serial) {}
    
    void begin(uint32_t baud = 115200) {
        _serial.begin(baud);
    }
    
    // Установить скорость в RPM (-480 ... +480)
    void setSpeed(uint8_t id, float rpm) {
        int16_t rpm_int = constrain((int16_t)rpm, -480, 480);
        
        uint8_t packet[8];
        packet[0] = id;
        packet[1] = 0x64;  // Speed command
        packet[2] = (rpm_int >> 8) & 0xFF;
        packet[3] = rpm_int & 0xFF;
        packet[4] = 0x00;
        packet[5] = 0x00;
        packet[6] = 0x00;
        packet[7] = crc8(packet, 7);
        
        _serial.write(packet, 8);
    }
    
    // Запросить и прочитать данные мотора
    bool requestFeedback(uint8_t id) {
        uint8_t packet[8] = {id, 0x74, 0, 0, 0, 0, 0, 0};
        packet[7] = crc8(packet, 7);
        _serial.write(packet, 8);
        
        // Ждём ответ (с таймаутом)
        uint32_t start = micros();
        while (_serial.available() < 8) {
            if (micros() - start > 2000) return false;  // 2ms timeout
        }
        
        uint8_t resp[8];
        _serial.readBytes(resp, 8);
        
        if (resp[0] != id || resp[1] != 0x74) return false;
        if (resp[7] != crc8(resp, 7)) return false;
        
        // Парсим
        _feedback[id].rpm = (int16_t)((resp[2] << 8) | resp[3]);
        _feedback[id].current_ma = (int16_t)((resp[4] << 8) | resp[5]);
        _feedback[id].encoder = (uint16_t)((resp[6] << 8) | resp[7]);
        
        return true;
    }
    
    float getRPM(uint8_t id) {
        requestFeedback(id);
        return (float)_feedback[id].rpm;
    }
    
    float getCurrent(uint8_t id) {
        return (float)_feedback[id].current_ma / 1000.0f;
    }

private:
    HardwareSerial &_serial;
    
    struct MotorFeedback {
        int16_t rpm = 0;
        int16_t current_ma = 0;
        uint16_t encoder = 0;
    };
    
    MotorFeedback _feedback[4];  // До 4 моторов
    
    uint8_t crc8(uint8_t *data, uint8_t len) {
        uint8_t crc = 0;
        for (uint8_t i = 0; i < len; i++) {
            crc += data[i];
        }
        return crc & 0xFF;
    }
};