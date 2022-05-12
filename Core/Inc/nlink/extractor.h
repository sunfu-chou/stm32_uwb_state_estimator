//
// Created by sunfu on 2022/4/18.
//

#ifndef UWB_POSE_TRACKING_EXTRACTOR_H
#define UWB_POSE_TRACKING_EXTRACTOR_H

#include "stm32f1xx.h"
#include "nlink/tagframe0.h"
#include "utils.h"

template<uint16_t RX_BUFFER_SIZE = 512,
        uint16_t TX_BUFFER_SIZE = 28,
        uint16_t IDX_HEADER_SIZE = 4>
class ProtocolExtractor {
public:
    explicit ProtocolExtractor(UARTBuffer &uart_buf, nlt_tagframe0_t &f = g_nlt_tagframe0) :
            uart_buffer(uart_buf),
            frame(f),
            rx_data_buf{},
            tx_data_buf{},
            tx_buffer_size(TX_BUFFER_SIZE),
            idx_new(0),
            idx_header{},
            idx_idx_header(),
            success_cnt(0),
            failure_cnt(0),
            is_success(false) {
        for (auto &i: idx_header) {
            i = -1;
        }
    }


    ~ProtocolExtractor() = default;

    static size_t normalizeNewIndex(const size_t idx) { return idx & (RX_BUFFER_SIZE - 1); }

    static size_t normalizeHeaderIndex(const size_t idx) { return idx & (IDX_HEADER_SIZE - 1); }

    bool addData(const int data) {
        if (data == -1) {
            is_success = false;
            return false;
        }

        rx_data_buf[idx_new] = static_cast<uint8_t>(data);
        idx_new = normalizeNewIndex(idx_new + 1);

        if (rx_data_buf[normalizeNewIndex(idx_new - 3)] == 0x55 &&
            rx_data_buf[normalizeNewIndex(idx_new - 2)] == 0x01) {
            idx_header[idx_idx_header] = normalizeNewIndex(idx_new - 3);
            idx_idx_header = normalizeHeaderIndex(idx_idx_header + 1);
        }

        if (normalizeNewIndex(idx_new - idx_header[normalizeHeaderIndex(idx_idx_header - 1)]) == 128 &&
            idx_header[normalizeHeaderIndex(idx_idx_header - 1)] >= 0) {
            uint8_t data_temp[128];
            size_t idx_tail = 128 <= RX_BUFFER_SIZE - idx_header[normalizeHeaderIndex(idx_idx_header - 1)] ?
                              128 :
                              RX_BUFFER_SIZE - idx_header[normalizeHeaderIndex(idx_idx_header - 1)];

            memcpy(&data_temp, &rx_data_buf[idx_header[normalizeHeaderIndex(idx_idx_header - 1)]], idx_tail);

            if (idx_tail != 128) {
                memcpy(&data_temp + idx_tail, &rx_data_buf, 128 - idx_tail);
            }

            if (frame.UnpackData(data_temp, 128)) {
                ++success_cnt;
                toSerial();
                is_success = true;
            } else {
                ++failure_cnt;
                is_success = false;
            }
        }
        return true;
    }

    bool addData() {
        int data = uart_buffer.read();
        return addData(data);
    }

    uint8_t *toSerial() {
        uint16_t idx = 0;
        uint16_t pos_x = Float32toUInt16(frame.result.pos_3d[0]);
        uint16_t pos_y = Float32toUInt16(frame.result.pos_3d[1]);
        memcpy(tx_data_buf + idx, &pos_x, 2);
        idx += 2;
        memcpy(tx_data_buf + idx, &pos_y, 2);
        idx += 2;
        memcpy(tx_data_buf + idx, &frame.result.imu_gyro_3d, 12);
        idx += 12;
        memcpy(tx_data_buf + idx, &frame.result.imu_acc_3d, 12);
        return tx_data_buf;
    }

    UARTBuffer &uart_buffer;
    nlt_tagframe0_t &frame;
    uint8_t rx_data_buf[RX_BUFFER_SIZE];
    uint8_t tx_data_buf[TX_BUFFER_SIZE];
    uint16_t tx_buffer_size;
    size_t idx_new;
    size_t idx_header[IDX_HEADER_SIZE];
    size_t idx_idx_header;
    uint32_t success_cnt;
    uint32_t failure_cnt;
    bool is_success;
private:
};

#endif //UWB_POSE_TRACKING_EXTRACTOR_H
