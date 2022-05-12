//
// Created by sunfu on 2022/4/24.
//

#ifndef UWB_POSE_TRACKING_PUBLISHER_H
#define UWB_POSE_TRACKING_PUBLISHER_H

#include "stm32f1xx.h"
#include "nlink/tagframe0.h"
#include "utils.h"

template<uint16_t DATA_PREFIX_SIZE>
class DataTransmissionPublisher {
public:
    explicit DataTransmissionPublisher(UARTBuffer &uart_buf) :
            uart_buffer(uart_buf),
            data_prefix{} {
        // Frame Header
        data_prefix[0] = 0x54;
        // Function Mark
        data_prefix[1] = 0xF1;
        // reserved * 4
        data_prefix[2] = 0xFF;
        data_prefix[3] = 0xFF;
        data_prefix[4] = 0xFF;
        data_prefix[5] = 0xFF;
        // remote_role
        data_prefix[6] = 0x00;
        // remote_id
        data_prefix[7] = 0x00;
        // data_length
        data_prefix[8] = 0x00;
        data_prefix[9] = 0x00;
    }

    ~DataTransmissionPublisher() = default;

    bool publish(uint8_t *data, uint16_t length) {
        uint16_t length_tot = length + DATA_PREFIX_SIZE + 1;
        uint8_t data_tx[length_tot];
        memset(data_tx, 0, length_tot);

        memcpy(data_tx, data_prefix, DATA_PREFIX_SIZE - 2);
        memcpy(data_tx + DATA_PREFIX_SIZE - 2, &length, 2);
        memcpy(data_tx + DATA_PREFIX_SIZE, data, length);
        NLink_UpdateCheckSum(data_tx, length_tot);
        uart_buffer.write(data_tx, length_tot);
        return true;
    }

    UARTBuffer &uart_buffer;
    uint8_t data_prefix[DATA_PREFIX_SIZE];
private:
};

#endif //UWB_POSE_TRACKING_PUBLISHER_H
