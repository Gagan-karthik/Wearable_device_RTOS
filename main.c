#include <zephyr.h>
#include <drivers/i2c.h>
#include <drivers/flash.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <sys/printk.h>
#include <string.h>
#include "i2c_communication.h"
#include "imu_driver.h"
#include "ppg_driver.h"
#include "ble_communication.h"



// Define task priorities and stack sizes
#define STACK_SIZE 1024
#define PRIORITY_BLE_TASK 3
#define PRIORITY_FLASH_TASK 2
#define PRIORITY_SENSOR_TASK 1

// Flash storage configuration
#define FLASH_BASE_ADDRESS DT_FLASH_AREA_STORAGE_OFFSET
#define FLASH_AREA_SIZE DT_FLASH_AREA_STORAGE_SIZE


// Shared data structures and synchronization
struct sensor_session {
    uint32_t timestamp;
    int16_t acc_x, acc_y, acc_z;
    int16_t gyr_x, gyr_y, gyr_z;
    uint32_t red, ir;
};

K_MSGQ_DEFINE(sensor_data_msgq, sizeof(struct sensor_session), 10, 4); // Message queue
K_MUTEX_DEFINE(flash_mutex);

bool is_session_active = false;
static const struct device *flash_dev;

// Thread references for flash and sensor tasks
static struct k_thread flash_data_thread;
static struct k_thread sensor_data_thread;
static struct k_thread ble_thread;
k_tid_t flash_thread_id = NULL;
k_tid_t sensor_thread_id = NULL;
static k_tid_t ble_thread_id = NULL;

// Stack for sensor and flash tasks
K_THREAD_STACK_DEFINE(sensor_stack_area, STACK_SIZE);
K_THREAD_STACK_DEFINE(flash_stack_area, STACK_SIZE);
K_THREAD_STACK_DEFINE(ble_stack_area, STACK_SIZE);

// Function declarations

static const struct device *flash_init(void);
static void sensor_task(void);
static void flash_task(void);
void ble_init(void)
static ssize_t start_session_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static ssize_t stop_session_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

// Define BLE services
BT_GATT_SERVICE_DEFINE(sensor_Data_service,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(UUID_BASE)),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_16(UUID_START_SESSION), BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, start_session_write, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_16(UUID_STOP_SESSION), BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, stop_session_write, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_16(UUID_DOWNLOAD_DATA), BT_GATT_CHRC_READ, BT_GATT_PERM_READ, session_data_read, NULL, NULL)
);



// Initialize Flash device
static const struct device *flash_init(void) {
    const struct device *flash_dev = device_get_binding(DT_LABEL(DT_NODELABEL(flash0))); 
    if (!flash_dev) {
        printk("No Flash device found\n");
        return NULL;
    }
    return flash_dev;
}

// Sensor task to acquire data periodically
static void sensor_task(void) {
    uint8_t imu_buffer[12];
    uint8_t ppg_buffer[6];

    while (is_session_active) {
        struct sensor_session session;
        session.timestamp = k_uptime_get_32();

        if (read_imu_data(imu_buffer) != 0) {
            printk("IMU read failed. Stopping session.\n");
            is_session_active = false;
            break;
        }

        if (read_ppg_data(ppg_buffer) != 0) {
            printk("PPG read failed. Stopping session.\n");
            is_session_active = false;
            break;
        }

        session.acc_x = (imu_buffer[1] << 8) | imu_buffer[0];
        session.acc_y = (imu_buffer[3] << 8) | imu_buffer[2];
        session.acc_z = (imu_buffer[5] << 8) | imu_buffer[4];
        session.gyr_x = (imu_buffer[7] << 8) | imu_buffer[6];
        session.gyr_y = (imu_buffer[9] << 8) | imu_buffer[8];
        session.gyr_z = (imu_buffer[11] << 8) | imu_buffer[10];
        session.red = (ppg_buffer[0] << 16) | (ppg_buffer[1] << 8) | ppg_buffer[2];
        session.ir = (ppg_buffer[3] << 16) | (ppg_buffer[4] << 8) | ppg_buffer[5];

        k_msgq_put(&sensor_data_msgq, &session, K_NO_WAIT); // K_NO_WAIT: message queue immediately returns when full.
        k_sleep(K_MSEC(40)); // 25 Hz sampling rate as Output data rate of imu sensor is 25Hz and sp02 is 50Hz
    }

    printk("Sensor task exited\n");
}

// Flash task to store data
static void flash_task(void) {
    struct sensor_session session;
    size_t offset = FLASH_BASE_ADDRESS;

    while (1) {
        if (k_msgq_get(&sensor_data_msgq, &session, K_FOREVER) == 0) {
            k_mutex_lock(&flash_mutex, K_FOREVER);
            if (offset + sizeof(session) <= FLASH_AREA_SIZE) {
                if (flash_write(flash_dev, offset, &session, sizeof(session)) != 0) {
                    printk("Failed to write to flash\n");
                }
                offset += sizeof(session);
            } else {
                printk("Flash memory full. Stopping session.\n");
                is_session_active = false;
            }
            k_mutex_unlock(&flash_mutex);
        }
    }
}

// BLE callbacks for session control
static ssize_t start_session_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    printk("Session started\n");

 // Start only if not already active
    if (!is_session_active) {
        is_session_active = true;

        // Start the sensor and flash tasks
        sensor_thread_id = k_thread_create(&sensor_data_thread, sensor_stack_area, K_THREAD_STACK_SIZEOF(sensor_stack_area),sensor_task, NULL, NULL, NULL, PRIORITY_SENSOR_TASK, 0, K_NO_WAIT);
        printk("Sensor thread started\n");

        flash_thread_id = k_thread_create(&flash_data_thread, flash_stack_area, K_THREAD_STACK_SIZEOF(flash_stack_area),flash_task, NULL, NULL, NULL, PRIORITY_FLASH_TASK, 0, K_NO_WAIT);
        printk("Flash thread started\n");
    }

    return len;
}

static ssize_t stop_session_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    printk("Session stopped\n");

    if (is_session_active) { 
        is_session_active = false;

        // Stop the sensor and flash threads
        if (sensor_thread_id != NULL) {
            k_thread_abort(sensor_thread_id);
            sensor_thread_id = NULL;
            printk("Sensor thread stopped\n");
        }

        if (flash_thread_id != NULL) {
            k_thread_abort(flash_thread_id);
            flash_thread_id = NULL;
            printk("Flash thread stopped\n");
        }
    }

    return len;
}

// Initialize BLE and start services
void ble_init(void) {
    int err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

}

// Main function
void main(void) {
    i2c_dev = i2c_init();
    flash_dev = flash_init();
    ble_thread_id = k_thread_create(&ble_thread, ble_stack_area, K_THREAD_STACK_SIZEOF(ble_stack_area),ble_init, NULL, NULL, NULL, PRIORITY_BLE_TASK, 0, K_NO_WAIT);
}
