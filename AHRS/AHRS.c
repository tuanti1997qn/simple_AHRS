#include <string.h>
#include <stdio.h>
#include <syslog.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include "madgwickFilter.h"
#include "AHRS.h"
#include "ssd1306.h"
#include "pthread.h"

#define MPU6050_path ("/dev/imu_reader")
#define I2C_DEVICE_FILE   ("/dev/i2c-2")

#define ACC_FS_SENSITIVITY_0					16384
#define ACC_FS_SENSITIVITY_1		            8192
#define ACC_FS_SENSITIVITY_2		            4096
#define ACC_FS_SENSITIVITY_3		            2048

#define GYR_FS_SENSITIVITY_0					 131
#define GYR_FS_SENSITIVITY_1					 65.5
#define GYR_FS_SENSITIVITY_2					 32.8
#define GYR_FS_SENSITIVITY_3				 	 16.4

struct imu_raw_data {
    short acc_x;
    short acc_y;
    short acc_z;
    short gyro_x;
    short gyro_y;
    short gyro_z;
};

struct gyro_data {
    double gyro_x;
    double gyro_y;
    double gyro_z;
};

struct euler_data {
    float roll;
    float pitch;
    float yaw;
};

struct euler_data euler_angles = {
    .roll = 0,
    .pitch = 0,
    .yaw = 0
};

#define degree_to_radian(x) ((x) * 0.01745329251f)

void imu_thread(void *arg)
{
    struct imu_raw_data raw_data;
    struct gyro_data offset_gyro = {
        .gyro_x = 0,
        .gyro_y = 0,
        .gyro_z = 0
    };
    int ret;
    int fd = open(MPU6050_path, O_RDWR);
    if (fd < 0) {
        syslog(LOG_ERR, "Failed to open I2C device file.\n");
        return;
    }

    int loop = 10;
    while (loop--) {
        ret = read(fd, &raw_data, sizeof(raw_data));
        if (ret < 0) {
            syslog(LOG_ERR, "Failed to read from I2C device\n");
            close(fd);
            return;
        }

        offset_gyro.gyro_x += degree_to_radian((double)raw_data.gyro_x / GYR_FS_SENSITIVITY_0);
        offset_gyro.gyro_y += degree_to_radian((double)raw_data.gyro_y / GYR_FS_SENSITIVITY_0);
        offset_gyro.gyro_z += degree_to_radian((double)raw_data.gyro_z / GYR_FS_SENSITIVITY_0);
        usleep(10000); // sleep for 10ms
    }

    offset_gyro.gyro_x /= 10;
    offset_gyro.gyro_y /= 10;
    offset_gyro.gyro_z /= 10;
    while (1) {
        ret = read(fd, &raw_data, sizeof(raw_data));
        if (ret < 0) {
            syslog(LOG_ERR, "Failed to read from I2C device\n");
            close(fd);
            return;
        }
        // printf("acc_x: %d, acc_y: %d, acc_z: %d, gyro_x: %d, gyro_y: %d, gyro_z: %d\n",
        //        raw_data.acc_x, raw_data.acc_y, raw_data.acc_z,
        //        raw_data.gyro_x, raw_data.gyro_y, raw_data.gyro_z);

        // Convert raw values to physical units
        double accx = (double)raw_data.acc_x / ACC_FS_SENSITIVITY_0;
        double accy = (double)raw_data.acc_y / ACC_FS_SENSITIVITY_0;
        double accz = (double)raw_data.acc_z / ACC_FS_SENSITIVITY_0;
        double gyrox = degree_to_radian((double)raw_data.gyro_x / GYR_FS_SENSITIVITY_0) - offset_gyro.gyro_x;
        double gyroy = degree_to_radian((double)raw_data.gyro_y / GYR_FS_SENSITIVITY_0) - offset_gyro.gyro_y;
        double gyroz = degree_to_radian((double)raw_data.gyro_z / GYR_FS_SENSITIVITY_0) - offset_gyro.gyro_z;
        // printf("acc_x: %f, acc_y: %f, acc_z: %f, gyro_x: %f, gyro_y: %f, gyro_z: %f\n",
        //        accx, accy, accz,
        //        gyrox, gyroy, gyroz);
        // Call the Madgwick filter
        imu_filter(accx, accy, accz, gyrox, gyroy, gyroz);
        eulerAngles(q_est, &euler_angles.roll, &euler_angles.pitch, &euler_angles.yaw);
        // printf("Roll: %f Pitch: %f Yaw: %f\n", 
        //        euler_angles.roll, euler_angles.pitch, euler_angles.yaw);
        usleep(10000); // sleep for 10ms
    }

}

void oled_display_thread(void *arg)
{
    struct display_info disp;
    disp.address = SSD1306_I2C_ADDR;
    ssd1306_open(&disp, "/dev/i2c-2");
    ssd1306_init(&disp);
    ssd1306_send_buffer(&disp);

    static double vector_length = 30;

    while (1) {
        // Update the display with the latest euler angles
        memset(disp.buffer, 0, sizeof(disp.buffer));
        double roll_rad = degree_to_radian(euler_angles.roll);
        double yaw_rad = degree_to_radian(euler_angles.yaw);
        int x = (int)vector_length*cos(roll_rad)*cos(yaw_rad)+96;
        int y = (int)vector_length*cos(roll_rad)*sin(yaw_rad)+32;
        ssd1306_writeLine(&disp, 96, 32, x, y);
        char str[20];
        snprintf(str, sizeof(str), "Roll:%d", (int)euler_angles.roll);
        sd1306_write_string(&disp, str, 0, 0);
        snprintf(str, sizeof(str), "Pitch:%d", (int)euler_angles.pitch);
        sd1306_write_string(&disp, str, 0, 23);
        snprintf(str, sizeof(str), "Yaw:%d", (int)euler_angles.yaw);
        sd1306_write_string(&disp, str, 0, 46);

        ssd1306_send_buffer(&disp);
        usleep(100000); // sleep for 100ms
    }
}



int main(int argc, char *argv[])
{
    int ret;

    openlog("AHRS", LOG_PID | LOG_PERROR, LOG_USER);
    
    pthread_t imu_thread_id;
    ret = pthread_create(&imu_thread_id, NULL, (void *)imu_thread, NULL);
    if (ret != 0) {
        syslog(LOG_ERR, "Failed to create IMU thread\n");
        return -1;
    }

    pthread_t oled_thread_id;
    ret = pthread_create(&oled_thread_id, NULL, (void *)oled_display_thread, NULL);

    while(1);
    return 0;
}