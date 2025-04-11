#ifndef __IMU_READER_H_
#define __IMU_READER_H_

#define PDEBUG(fmt, args...) printk(KERN_DEBUG "IMU reader " fmt, ## args)

struct imu_reader_dev {
    struct cdev cdev; // Character device structure
    struct mutex imu_mutex;

};

#endif