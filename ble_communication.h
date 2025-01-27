// ble_communication.h
#ifndef BLE_COMMUNICATION_H
#define BLE_COMMUNICATION_H

#include <zephyr.h>

// BLE characteristics UUIDs
#define UUID_BASE { 0x12, 0x34, 0x56, 0x78, 0x90, 0xAB, 0xCD, 0xEF, 0xFE, 0xDC, 0xBA, 0x09, 0x87, 0x65, 0x43, 0x21 }
#define UUID_START_SESSION 0x1234
#define UUID_STOP_SESSION  0x1235
#define UUID_METADATA      0x1236
#define UUID_DOWNLOAD_DATA 0x1237


static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);

#endif // BLE_COMMUNICATION_H
