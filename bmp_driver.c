#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/property.h>

/*****************************
 *  Bosch BMP2-Sensortec API Headers
 *  */
#include "bmp2_defs.h"
#include "bmp2.h"

#define DEV_MEM_SIZE                    512
#define I2C_TX_BUFF_SIZE                128
#define I2C_BUS_NUMBER                  0
#define BMP280_SLAVE_DEVICE_NAME        "BMP280"
#define BMP280_SLAVE_DEVICE_ADDR        BMP2_I2C_ADDR_PRIM
#define BMP_SENSOR_NAME                 "bmp280_dev"
#define E_FAILED                        -1
#define E_SUCCESS                       0
#define BMP2_64BIT_COMPENSATION

BMP2_INTF_RET_TYPE bmp280_i2c_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
BMP2_INTF_RET_TYPE bmp280_i2c_reg_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
static int bmp280_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int bmp280_remove(struct i2c_client *client);
void bmp2_error_codes_print_result(const char api_name[], int8_t rslt);
static int8_t get_data(uint32_t period, struct bmp2_config *conf, struct bmp2_dev *dev, struct bmp2_data *comp_data);


// dev_t bmp_dev;

static struct kobject *bmp280_kobj;

struct bmp2_dev bmp280_dev;
struct bmp2_config bmp280_conf;


/* pseudo device's memory */
char device_buffer[DEV_MEM_SIZE];


/*************************************************/

static struct i2c_adapter *bmp280_i2c_adapter = NULL;
static struct i2c_client *bmp280_i2c_client = NULL;

static struct of_device_id bmp280_of_i2c_match[] = {
    {
        .compatible = "bosch,bmp280",
    },
    {/*Guard section*/},
};

MODULE_DEVICE_TABLE(of, bmp280_of_i2c_match);

static const struct i2c_device_id bmp280_i2c_dev_id[] = {
    {BMP280_SLAVE_DEVICE_NAME, I2C_BUS_NUMBER},
    { },/*Empty to Nerminate the list*/
};

static struct i2c_driver bmp280_i2c_driver = {
    .driver = {
        .name = BMP280_SLAVE_DEVICE_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(bmp280_of_i2c_match),
    },
    .probe = bmp280_probe,
    .remove = bmp280_remove,
    .id_table = bmp280_i2c_dev_id
};
module_i2c_driver(bmp280_i2c_driver);

/************************* SYSFS Callbacks **************************************/ 

/**
 * @brief Read CB for bmp280_device for reading temperature
 * @param[in] kobj: kobject
 * @param[in] attr: ttributes information
 * @param[out] buffer: Write to use. 
 * 
 * @returns Number of bytes written to buffer
 */
static ssize_t bmp280_temperature_show(struct kobject *kobj, struct kobj_attribute *attr, char *buffer) {
	int result = BMP2_OK;
    struct bmp2_data bmp2_data_buff;
    uint32_t meas_time = 0;
    pr_info("Device file read!!!\n");

    /* Set normal power mode */
    result = bmp2_set_power_mode(BMP2_POWERMODE_NORMAL, &bmp280_conf, &bmp280_dev);
    bmp2_error_codes_print_result("bmp2_set_power_mode", result);

    /* Calculate measurement time in microseconds */
    result = bmp2_compute_meas_time(&meas_time, &bmp280_conf, &bmp280_dev);
    bmp2_error_codes_print_result("bmp2_compute_meas_time", result);

    /* Read pressure and temperature data */
    result = get_data(meas_time, &bmp280_conf, &bmp280_dev, &bmp2_data_buff);
    bmp2_error_codes_print_result("get_data", result);

    return sprintf(buffer, "%ld deg C\n", (long int)bmp2_data_buff.temperature);
}

/**
 * @brief Read CB for bmp280_device for reading Pressure
 * @param[in] kobj: kobject
 * @param[in] attr: ttributes information
 * @param[out] buffer: Write to use. 
 * 
 * @returns Number of bytes written to buffer
 */
static ssize_t bmp280_pressure_show(struct kobject *kobj, struct kobj_attribute *attr, char *buffer) {
	int result = BMP2_OK;
    struct bmp2_data bmp2_data_buff;
    uint32_t meas_time = 0;
    pr_info("Device file read!!!\n");

    /* Set normal power mode */
    result = bmp2_set_power_mode(BMP2_POWERMODE_NORMAL, &bmp280_conf, &bmp280_dev);
    bmp2_error_codes_print_result("bmp2_set_power_mode", result);

    /* Calculate measurement time in microseconds */
    result = bmp2_compute_meas_time(&meas_time, &bmp280_conf, &bmp280_dev);
    bmp2_error_codes_print_result("bmp2_compute_meas_time", result);

    /* Read pressure and temperature data */
    result = get_data(meas_time, &bmp280_conf, &bmp280_dev, &bmp2_data_buff);
    bmp2_error_codes_print_result("get_data", result);

    return sprintf(buffer, "%lu Pa\n", (long unsigned int)bmp2_data_buff.pressure);
}

/**
 * @brief Creating pressue attribute
 * 
 */
static struct kobj_attribute bmp280_pressure_attr = __ATTR_RO(bmp280_pressure);

/**
 * @brief Creating temperature attribute
 * 
 */
static struct kobj_attribute bmp280_temperature_attr = __ATTR_RO(bmp280_temperature);

/**
 * @brief Creating group of attributes
 * 
 */
static struct attribute *attrs[] = {
    &bmp280_temperature_attr.attr,
    &bmp280_pressure_attr.attr,
    NULL,/*NULL to terminate the list*/
};

static struct attribute_group attr_group = {
    .attrs = attrs,
};

/**
 * @brief Kernel Delay in us
 * @param[in] period : Delay time in us
 * @param[in] intf_ptr : Interface Selection
 *                          By Default: I2C
 *  @return voide
 */
void bmp280_delay_us(uint32_t period, void *intf_ptr)
{
    udelay(period);
}

/**
 * @brief bmp280_remove : Called during removing the device
 * @param[in] client : Client Information
 * 
 * @return Status of execution.
 */
static int bmp280_remove(struct i2c_client *client)
{
    // put sensor in sleep mode	
    bmp2_set_power_mode(BMP2_POWERMODE_SLEEP, &bmp280_conf, &bmp280_dev);
    kobject_put(bmp280_kobj);
    pr_info("bmp_driver - bmp280_remove - BMP280 Removed!!!\n");
    return 0;
}

/**
 * @brief bmp280_probe : Called during entry of probe
 * @param[in] client : Client Information
 * @param[in] i2c_device_id : i2c_device_id table information
 * @return Status of execution.
 */
static int bmp280_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    uint32_t meas_time;
    uint8_t result = BMP2_OK;
    struct bmp2_data bmp2_temp;
    uint8_t dev_addr; //= BMP280_SLAVE_DEVICE_ADDR;
    uint8_t device_id = 0;
    // uint8_t reg_addr = BMP2_REG_CHIP_ID;

    // struct device *dev = &client->dev;
    bmp280_i2c_adapter = client->adapter;
    bmp280_i2c_client = client;
    dev_addr = client->addr;

    /*Read chip id info*/
    device_id = i2c_smbus_read_byte_data(bmp280_i2c_client, BMP2_REG_CHIP_ID);
    

    printk("ID: 0x%x\n", device_id);

    pr_info("bmp_driver - bmp280_probe - BMP280 Initialze!!!\n");
    pr_info("bmp280: Device ID : %x", dev_addr);
    bmp280_dev.read = bmp280_i2c_reg_read;
    bmp280_dev.write = bmp280_i2c_reg_write;
    bmp280_dev.intf = BMP2_I2C_INTF;
    bmp280_dev.intf_ptr = &dev_addr;
    bmp280_dev.delay_us = bmp280_delay_us;

    bmp280_kobj = kobject_create_and_add(BMP_SENSOR_NAME, kernel_kobj);
    if(bmp280_kobj == NULL)
    {
        pr_err("bmp280 - Error creating /sys/kernel/%s", BMP_SENSOR_NAME);
        return E_FAILED;
    }
    if(sysfs_create_group(bmp280_kobj, &attr_group) == true)
    {
        kobject_put(bmp280_kobj);
        return E_FAILED;
    }

    /**** Driver Specific Code *****/
    result = bmp2_init(&bmp280_dev);
    if(result < 0)
    {
        pr_info("bmp_driver - bmp2_init failed : %d", result);
        return E_FAILED;
    }

    result = bmp2_get_config(&bmp280_conf, &bmp280_dev);
    if(result < 0)
    {
        pr_info("bmp_driver - bmp2_get_config failed : %d", result);
        return E_FAILED;
    }

    /* Configuring the over-sampling mode, filter coefficient and output data rate */
    /* Overwrite the desired settings */
    bmp280_conf.filter = BMP2_FILTER_OFF;

    /* Over-sampling mode is set as high resolution i.e., os_pres = 8x and os_temp = 1x */
    bmp280_conf.os_mode = BMP2_OS_MODE_HIGH_RESOLUTION;

    /* Setting the output data rate */
    bmp280_conf.odr = BMP2_ODR_250_MS;

    result = bmp2_set_config(&bmp280_conf, &bmp280_dev);
    bmp2_error_codes_print_result("bmp2_set_config", result);

    /* Set normal power mode */
    result = bmp2_set_power_mode(BMP2_POWERMODE_NORMAL, &bmp280_conf, &bmp280_dev);
    bmp2_error_codes_print_result("bmp2_set_power_mode", result);

    /* Calculate measurement time in microseconds */
    result = bmp2_compute_meas_time(&meas_time, &bmp280_conf, &bmp280_dev);
    bmp2_error_codes_print_result("bmp2_compute_meas_time", result);

    /* Read pressure and temperature data */
    result = get_data(meas_time, &bmp280_conf, &bmp280_dev, &bmp2_temp);
    bmp2_error_codes_print_result("get_data", result);

    return result;
}

/**
 * @brief Get Data from the sensor
 * @param[in] period : Measurement Delay
 * @param[in] conf : bmp280 configuration 
 * @param[in] dev : bmp280 device information
 * @param[out] comp_data : compensated Temperature and Pressure Data
 * 
 * @return Status of execution.
 */
static int8_t get_data(uint32_t period, struct bmp2_config *conf, struct bmp2_dev *dev, struct bmp2_data *comp_data)
{
    int8_t rslt = BMP2_E_NULL_PTR;
    struct bmp2_status status;
    struct bmp2_data bmp2_temp_data;

    // printk("Measurement delay : %lu us\n", (long unsigned int)period);

    rslt = bmp2_get_status(&status, dev);
    bmp2_error_codes_print_result("bmp2_get_status", rslt);

    if (status.measuring == BMP2_MEAS_DONE)
    {
        /* Delay between measurements */
        dev->delay_us(period, dev->intf_ptr);

        /* Read compensated data */
        rslt = bmp2_get_sensor_data(&bmp2_temp_data, dev);
        bmp2_error_codes_print_result("bmp2_get_sensor_data", rslt);

        #ifdef BMP2_64BIT_COMPENSATION
        bmp2_temp_data.pressure = bmp2_temp_data.pressure / 256;
        #endif

        pr_info("Temperature: %ld deg C	Pressure: %lu Pa\n", (long int)bmp2_temp_data.temperature,
            (long unsigned int)bmp2_temp_data.pressure);

        //Copy to ptr
        comp_data->pressure = bmp2_temp_data.pressure;
        comp_data->temperature = bmp2_temp_data.temperature;
    }

    return rslt;
}

/**
 * @brief Error Info printer
 * @param[in] api_name : Which API USED
 * @param[in] rslt : Returns from API after execution
 * 
 * @return Status of execution.
 */
void bmp2_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMP2_OK)
    {
        printk("%s\t", api_name);

        switch (rslt)
        {
            case BMP2_E_NULL_PTR:
                printk("Error [%d] : Null pointer error.", rslt);
                printk(
                    "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
                break;
            case BMP2_E_COM_FAIL:
                printk("Error [%d] : Communication failure error.", rslt);
                printk(
                    "It occurs due to read/write operation failure and also due to power failure during communication\r\n");
                break;
            case BMP2_E_INVALID_LEN:
                printk("Error [%d] : Invalid length error.", rslt);
                printk("Occurs when length of data to be written is zero\n");
                break;
            case BMP2_E_DEV_NOT_FOUND:
                printk("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                       rslt);
                break;
            case BMP2_E_UNCOMP_TEMP_RANGE:
                printk("Error [%d] : Uncompensated temperature data not in valid range error.", rslt);
                break;
            case BMP2_E_UNCOMP_PRESS_RANGE:
                printk("Error [%d] : Uncompensated pressure data not in valid range error.", rslt);
                break;
            case BMP2_E_UNCOMP_TEMP_AND_PRESS_RANGE:
                printk(
                    "Error [%d] : Uncompensated pressure data and uncompensated temperature data are not in valid range error.",
                    rslt);
                break;
            default:
                printk("Error [%d] : Unknown error code\r\n", rslt);
                break;
        }
    }
}

/**
 * @brief Reading Sensors's registers through I2C Bus Communication
 * @param[in] reg_addr : Reg. address
 * @param[out] reg_data : Pointer to data buffer
 * @param[in] length : Np. Byest to read
 * @param[in] intf_ptr : Interface Selection
 *                          By Default: I2C
 * @return Status of execution.
 */
BMP2_INTF_RET_TYPE bmp280_i2c_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    int ret = i2c_master_send(bmp280_i2c_client, &reg_addr, 1);
    ret |= i2c_master_recv(bmp280_i2c_client, reg_data, length);
    
     if(ret >= 0)
    	return 0;
    else
    	return ret;
}

/**
 * @brief Writing Sensors's registers through I2C Bus Communication
 * @param[in] reg_addr : Reg. address
 * @param[out] reg_data : Pointer to data buffer
 * @param[in] length : Np. Byest to send
 * @param[in] intf_ptr : Interface Selection
 *                          By Default: I2C
 * 
 * @return Status of execution.
 */
BMP2_INTF_RET_TYPE bmp280_i2c_reg_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t tx_buff[I2C_TX_BUFF_SIZE] = {0};
    int ret = 0;
    //Reg. address
    tx_buff[0] = reg_addr;
    memcpy(&tx_buff[1], &reg_data,length);

    ret = i2c_master_send(bmp280_i2c_client, tx_buff, length + 1);
    
    if(ret >= 0)
    	return 0;
    else
    	return ret;
}


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jagannath Sahoo <jagannath.sahoo@live.com>");
MODULE_DESCRIPTION("BMP280 Linux Driver for Pressure and temperature read");
MODULE_VERSION("1.0");
