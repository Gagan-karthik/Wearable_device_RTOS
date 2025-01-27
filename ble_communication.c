#include "ble_communication.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <sys/printk.h>

// BLE service definition

static void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        printk("Failed to connect (err %u)\n", err);
        return;
    }
    printk("Connected\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    printk("Disconnected (reason %u)\n", reason);
    stop_sensor_and_flash_tasks();
    printk("Session stopped due to disconnection\n");
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

