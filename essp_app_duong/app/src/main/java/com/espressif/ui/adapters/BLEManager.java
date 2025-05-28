package com.espressif.ui.adapters;

import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCharacteristic;

public class BLEManager {
    private static BLEManager instance;
    private BluetoothGatt bluetoothGatt;
    private BluetoothGattCharacteristic writeCharacteristic;

    private BLEManager() {}

    public static BLEManager getInstance() {
        if (instance == null) {
            instance = new BLEManager();
        }
        return instance;
    }

    public void setBluetoothGatt(BluetoothGatt gatt) {
        this.bluetoothGatt = gatt;
    }

    public BluetoothGatt getBluetoothGatt() {
        return bluetoothGatt;
    }

    public void setWriteCharacteristic(BluetoothGattCharacteristic characteristic) {
        this.writeCharacteristic = characteristic;
    }

    public BluetoothGattCharacteristic getWriteCharacteristic() {
        return writeCharacteristic;
    }
}
