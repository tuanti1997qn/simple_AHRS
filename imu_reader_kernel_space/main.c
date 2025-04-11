/**
 * @file main.c
 * @brief simple kernel module for mpu6050
 *
 * Based on the implementation of the "scull" device driver, found in
 * Linux Device Drivers example code.
 *
 * @author Tuan Pham
 * @date 2025-04-02
 * @copyright Copyright (c) 2023
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/fs.h> // file_operations
#include "imu_reader.h"
#include <linux/i2c.h>
#include "MPU6050.h"
#include <linux/delay.h>

MODULE_AUTHOR("tuanti1997qn"); /** fill in your name **/
MODULE_LICENSE("GPL");

#define I2C_BUS_NUM     (         2 )
#define I2C_ADDR        (      0x68 )
#define I2C_DEV_NAME    ( "mympu6050" )

int imu_reader_major = 0;
int imu_reader_minor = 0;
struct imu_reader_dev imu_reader_device;

static struct i2c_adapter *i2c_adapter;
static struct i2c_client *i2c_client;
static struct i2c_board_info i2c_board_info = {
    I2C_BOARD_INFO(I2C_DEV_NAME, I2C_ADDR),
};

struct imu_raw_data {
    short acc_x;
    short acc_y;
    short acc_z;
    short gyro_x;
    short gyro_y;
    short gyro_z;
};


// static int I2C_Write(unsigned char *buf, unsigned int len)
// {
//     int ret = i2c_master_send(i2c_client, buf, len);
    
//     return ret;
// }
// static int I2C_Read(unsigned char *out_buf, unsigned int len)
// {
//     int ret = i2c_master_recv(i2c_client, out_buf, len);
    
//     return ret;
// }

static int I2C_write(unsigned char reg_addr, unsigned char data)
{
    int ret;
    unsigned char buf[2];
    buf[0] = reg_addr;
    buf[1] = data;
    ret = i2c_master_send(i2c_client, buf, 2);
    if (ret < 0) {
        PDEBUG("Failed to write to I2C device\n");
        return ret;
    }
    return 0;
}

static int I2C_read(unsigned char reg_addr, unsigned char *data, unsigned int len)
{
    int ret;
    ret = i2c_master_send(i2c_client, &reg_addr, 1);
    if (ret < 0) {
        PDEBUG("Failed to write to I2C device\n");
        return ret;
    }
    ret = i2c_master_recv(i2c_client, data, len);
    if (ret < 0) {
        PDEBUG("Failed to read from I2C device\n");
        return ret;
    }
    return 0;
}

static int imu_reader_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    PDEBUG("Probing I2C device: %s\n", client->name);
    // test whoami mpu6050
    unsigned char buf;

    I2C_read(MPU6050_RA_WHO_AM_I, &buf, 1);
    if (buf != 0x68) {
        PDEBUG("Failed to read WHO_AM_I register\n");
        return -ENODEV;
    }
    PDEBUG("WHO_AM_I: 0x%02x\n", buf);
    
    // init mpu6050
    // soft reset
    I2C_write(MPU6050_RA_PWR_MGMT_1, 0x80);
    msleep(1);

    // disable sleep mode
    I2C_write(MPU6050_RA_PWR_MGMT_1, 0x00);
    msleep(1);

    // Adjust full scale values for gyro and acc
    I2C_write(MPU6050_RA_ACCEL_CONFIG, 0x00);
    msleep(1);
    I2C_write(MPU6050_RA_GYRO_CONFIG, 0x00);
 
    return 0;
}
static void imu_reader_remove(struct i2c_client *client)
{
    PDEBUG("Removing I2C device: %s\n", client->name);
}
static const struct i2c_device_id imu_reader_id[] = {
    { I2C_DEV_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, imu_reader_id);
static struct i2c_driver imu_reader_driver = {
    .driver = {
        .name = I2C_DEV_NAME,
        .owner = THIS_MODULE,
    },
    .id_table = imu_reader_id,
    .probe = imu_reader_probe,
    .remove = imu_reader_remove,
};

// init template fops functions
int imu_reader_open(struct inode *inode, struct file *filp)
{
    PDEBUG("Opening IMU reader device\n");
    struct imu_reader_dev *dev = container_of(inode->i_cdev, struct imu_reader_dev, cdev);
    filp->private_data = dev;
    return 0;
}

int imu_reader_release(struct inode *inode, struct file *filp)
{
    PDEBUG("Releasing IMU reader device\n");
    filp->private_data = NULL;
    return 0;
}
ssize_t imu_reader_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    // if (*f_pos >0) {
    //     PDEBUG("End of file reached\n");
    //     return 0;
    // }

    PDEBUG("Reading from IMU reader device\n");
    // read accel and gyro from mpu6050
    unsigned char data[6];
    // mutex lock
    struct imu_reader_dev *dev = filp->private_data;
    if (mutex_lock_interruptible(&dev->imu_mutex)) {
        PDEBUG("Failed to lock mutex\n");
        return -ERESTARTSYS;
    }
    I2C_read(MPU6050_RA_ACCEL_XOUT_H, data, 6);
    // mutex unlock
    mutex_unlock(&dev->imu_mutex);
    // convert to 16 bit values
    short acc_x = (data[0] << 8) | data[1];
    short acc_y = (data[2] << 8) | data[3];
    short acc_z = (data[4] << 8) | data[5];
    short gyro_x, gyro_y, gyro_z;

    // mutex lock
    if (mutex_lock_interruptible(&dev->imu_mutex)) {
        PDEBUG("Failed to lock mutex\n");
        return -ERESTARTSYS;
    }
    I2C_read(MPU6050_RA_GYRO_XOUT_H, data, 6);
    // mutex unlock
    mutex_unlock(&dev->imu_mutex);
    // convert to 16 bit values
    gyro_x = (data[0] << 8) | data[1];
    gyro_y = (data[2] << 8) | data[3];
    gyro_z = (data[4] << 8) | data[5];
    // // copy to user space format string x,y,z + EOL
    // char buf_kernel[100];
    // snprintf(buf_kernel, sizeof(buf_kernel), "acc_x: %d, acc_y: %d, acc_z: %d, gyro_x: %d, gyro_y: %d, gyro_z: %d\n", acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
    // copy to user space
    // if (copy_to_user(buf, buf_kernel, strlen(buf_kernel))) {
    //     PDEBUG("Failed to copy to user space\n");
    //     return -EFAULT;
    // }
    // // update file position
    // *f_pos += strlen(buf_kernel);
    // // return number of bytes read
    // return strlen(buf_kernel);

    // copy to user space raw data
    struct imu_raw_data raw_data = {
        .acc_x = acc_x,
        .acc_y = acc_y,
        .acc_z = acc_z,
        .gyro_x = gyro_x,
        .gyro_y = gyro_y,
        .gyro_z = gyro_z,
    };

    if (copy_to_user(buf, &raw_data, sizeof(raw_data))) {
        PDEBUG("Failed to copy to user space\n");
        return -EFAULT;
    }

    // update file position
    *f_pos += sizeof(raw_data);
    // return number of bytes read
    return sizeof(raw_data);
}
ssize_t imu_reader_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    PDEBUG("Writing to IMU reader device\n");
    return 0;
}
long imu_reader_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    PDEBUG("IOCTL on IMU reader device\n");
    return 0;
}

// init fops
struct file_operations imu_reader_fops = {
    .owner = THIS_MODULE,
    .open = imu_reader_open,
    .release = imu_reader_release,
    .read = imu_reader_read,
    .write = imu_reader_write,
    .unlocked_ioctl = imu_reader_ioctl,
};

int __init imu_reader_init_module(void)
{
    PDEBUG("Initializing IMU reader module\n");
    dev_t dev = 0;
    int result;
    result = alloc_chrdev_region(&dev, 0, 1, "imu_reader");
    if (result < 0) {
        PDEBUG("Failed to allocate char device region\n");
        return result;
    }
    imu_reader_major = MAJOR(dev);
    imu_reader_minor = MINOR(dev);
    PDEBUG("Allocated char device region: major=%d, minor=%d\n", imu_reader_major, imu_reader_minor);

    memset(&imu_reader_device,0,sizeof(struct imu_reader_dev));

    int err;
    int devno = MKDEV(imu_reader_major, imu_reader_minor);

    cdev_init(&imu_reader_device.cdev, &imu_reader_fops);
    imu_reader_device.cdev.owner = THIS_MODULE;
    mutex_init(&imu_reader_device.imu_mutex);

    err = cdev_add(&imu_reader_device.cdev, devno, 1);
    if (err) {
        PDEBUG("Failed to add cdev\n");
        unregister_chrdev_region(dev, 1);
        return err;
    }
    PDEBUG("Added cdev: %d\n", err);

    // init i2c bus
    i2c_adapter = i2c_get_adapter(I2C_BUS_NUM);
    if (!i2c_adapter) {
        PDEBUG("Failed to get I2C adapter\n");
        cdev_del(&imu_reader_device.cdev);
        unregister_chrdev_region(dev, 1);
        return -ENODEV;
    }
    PDEBUG("Got I2C adapter: %p\n", i2c_adapter);
    i2c_client = i2c_new_client_device(i2c_adapter, &i2c_board_info);
    if (!i2c_client) {
        PDEBUG("Failed to create I2C client\n");
        cdev_del(&imu_reader_device.cdev);
        unregister_chrdev_region(dev, 1);
        return -ENODEV;
    }
    i2c_add_driver(&imu_reader_driver);
    i2c_put_adapter(i2c_adapter);
    PDEBUG("Created I2C client: %p\n", i2c_client);
    return 0;
}

void __exit imu_reader_cleanup_module(void)
{
    PDEBUG("Cleaning up IMU reader module\n");
    i2c_del_driver(&imu_reader_driver);
    i2c_unregister_device(i2c_client);
    cdev_del(&imu_reader_device.cdev);
    unregister_chrdev_region(MKDEV(imu_reader_major, imu_reader_minor), 1);
}



module_init(imu_reader_init_module);
module_exit(imu_reader_cleanup_module);
