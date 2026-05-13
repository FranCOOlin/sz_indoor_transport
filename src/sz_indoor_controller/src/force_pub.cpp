#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <deque>
#include <eigen3/Eigen/Dense>

#include "sz_indoor_controller/custom/system_params.h"


// CRC16 Modbus 
uint16_t crc16_modbus(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

uint32_t swap_endian(uint32_t x) {
    return ((x >> 24) & 0x000000FF) | ((x >> 8) & 0x0000FF00) |
           ((x << 8) & 0x00FF0000) | ((x << 24) & 0xFF000000);
}

float bytes_to_float(uint8_t *bytes) {
    uint32_t temp = 0;
    memcpy(&temp, bytes, sizeof(temp));
    float value;
    memcpy(&value, &temp, sizeof(value));
    return value;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "adc_reader_node");
    ros::NodeHandle nh("~");

    std::string uav_id;
    std::string file_path;
    ros::param::get("~uav_id", uav_id);
    ros::param::get("~file_path", file_path);
    common::SystemParams params;

    if (!params.loadFromFile(file_path))
    {
        ROS_ERROR("Failed to load parameters from file.");
        return -1;
    }

    ros::Publisher pub = nh.advertise<geometry_msgs::Vector3Stamped>("/" + uav_id + "/cable_force", 10);

    int fd;
    if ((fd = serialOpen("/dev/ttyAS3", 115200)) < 0) {
        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
        return 1;
    }

    wiringPiSetup();
    pinMode(19, OUTPUT);
    digitalWrite(19, HIGH);
    std::deque<uint8_t> data_queue;

    digitalWrite(19, LOW);
    delay(100);
    digitalWrite(19, HIGH);

    // 参数矩阵 M（示例值）
    Eigen::Matrix3d M;
    M = params.force_M;

    while (ros::ok()) {
        if (serialDataAvail(fd)) {
            int c = serialGetchar(fd);
            if (c >= 0) {
                data_queue.push_back((uint8_t)c);
            }
        }

        while (data_queue.size() >= 14) {
            uint8_t buf[14];
            for (int i = 0; i < 14; ++i) {
                buf[i] = data_queue[i];
            }

            uint16_t crc = crc16_modbus(buf, 12);
            uint16_t crc_recv = buf[12] | (buf[13] << 8);

            if (crc == crc_recv) {
                float v1 = bytes_to_float(&buf[0]);
                float v2 = bytes_to_float(&buf[4]);
                float v3 = bytes_to_float(&buf[8]);

                Eigen::Vector3d raw(v1, v2, v3);
                Eigen::Vector3d result = M * raw;

                geometry_msgs::Vector3Stamped msg;
                msg.header.stamp = ros::Time::now();
                msg.vector.x = result.x();
                msg.vector.y = result.y();
                msg.vector.z = result.z();
                pub.publish(msg);

                for (int i = 0; i < 14; ++i) {
                    data_queue.pop_front();
                }
            } else {
                data_queue.pop_front();
                ROS_WARN("CRC check failed, discarding first byte");
            }

            if (data_queue.size() > 512) {
                data_queue.clear();
                ROS_WARN("Queue cleared due to overflow");
            }
        }

        ros::spinOnce();
    }

    return 0;
}
