#include "ble_communication.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <sys/printk.h>


extern bool is_session_active;
extern k_tid_t sensor_thread_id;
extern k_tid_t flash_thread_id;

static void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        printk("Failed to connect (err %u)\n", err);
        return;
    }
    printk("Connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    printk("Disconnected (reason %u)\n", reason);

    if (is_session_active) {
        is_session_active = false;

        // Stop the sensor thread
        if (sensor_thread_id != NULL) {
            k_thread_abort(sensor_thread_id);
            sensor_thread_id = NULL;
            printk("Sensor thread stopped due to disconnection\n");
        }

        // Stop the flash thread
        if (flash_thread_id != NULL) {
            k_thread_abort(flash_thread_id);
            flash_thread_id = NULL;
            printk("Flash thread stopped due to disconnection\n");
        }
    }
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

