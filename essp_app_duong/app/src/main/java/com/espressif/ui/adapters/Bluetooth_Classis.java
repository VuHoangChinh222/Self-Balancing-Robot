package com.espressif.ui.adapters;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.UUID;

public class Bluetooth_Classis {
    private static final String TAG = "Bluetooth_Classis";
    private static final UUID HC05_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB"); // UUID cho HC-05
    private BluetoothSocket bluetoothSocket;
    private BluetoothAdapter bluetoothAdapter;
    private InputStream inputStream;
    private OutputStream outputStream;

    public Bluetooth_Classis() {
        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if (bluetoothAdapter == null) {
            Log.e(TAG, "Bluetooth is not supported on this device");
        }
    }

    // Kết nối với thiết bị HC-05
    public boolean connect(BluetoothDevice device) {
        try {
            bluetoothSocket = device.createRfcommSocketToServiceRecord(HC05_UUID);
            bluetoothAdapter.cancelDiscovery(); // Hủy quét để tránh xung đột
            bluetoothSocket.connect(); // Kết nối với thiết bị
            inputStream = bluetoothSocket.getInputStream();
            outputStream = bluetoothSocket.getOutputStream();
            Log.d(TAG, "Connected to HC-05: " + device.getName());
            return true;
        } catch (IOException e) {
            Log.e(TAG, "Failed to connect to HC-05", e);
            disconnect();
            return false;
        }
    }

    // Gửi dữ liệu đến HC-05
    public boolean sendData(String data) {
        try {
            if (outputStream != null) {
                outputStream.write(data.getBytes());
                Log.d(TAG, "Sent data: " + data);
                return true;
            } else {
                Log.e(TAG, "OutputStream is null. Cannot send data.");
                return false;
            }
        } catch (IOException e) {
            Log.e(TAG, "Failed to send data", e);
            return false;
        }
    }

    // Nhận dữ liệu từ HC-05
    public String receiveData() {
        try {
            if (inputStream != null) {
                byte[] buffer = new byte[1024];
                int bytes = inputStream.read(buffer);
                String receivedData = new String(buffer, 0, bytes);
                Log.d(TAG, "Received data: " + receivedData);
                return receivedData;
            } else {
                Log.e(TAG, "InputStream is null. Cannot receive data.");
                return null;
            }
        } catch (IOException e) {
            Log.e(TAG, "Failed to receive data", e);
            return null;
        }
    }

    // Ngắt kết nối với HC-05
    public void disconnect() {
        try {
            if (bluetoothSocket != null) {
                bluetoothSocket.close();
                Log.d(TAG, "Disconnected from HC-05");
            }
        } catch (IOException e) {
            Log.e(TAG, "Failed to disconnect from HC-05", e);
        }
    }
}