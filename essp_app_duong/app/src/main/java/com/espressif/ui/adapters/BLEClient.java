package com.espressif.ui.adapters;

import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCharacteristic;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

public class BLEClient {
    private volatile boolean canSend = true;

    private BluetoothGatt bluetoothGatt;
        private BluetoothGattCharacteristic writeCharacteristic;
        private final AtomicReference<short[]> dataRef = new AtomicReference<>(new short[3]);
    private final AtomicInteger counter = new AtomicInteger(0);
    private volatile boolean sending = false;
        private final byte[] byteBuffer = new byte[6];
        private volatile boolean isSending = false;
        private Thread sendingThread;
        private LogCallback logCallback;

        public interface LogCallback {
            void log(String message);
        }

        public BLEClient(LogCallback logCallback) {
            this.logCallback = logCallback;
        }

        public void setGattAndCharacteristic(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic) {
            this.bluetoothGatt = gatt;
            this.writeCharacteristic = characteristic;
        }

    public void startContinuousSending() {
        if (isSending) return;
        if (bluetoothGatt == null || writeCharacteristic == null) {
            log("BluetoothGatt or Characteristic not set!");
            return;
        }
        isSending = true;

        sendingThread = new Thread(() -> {
            try {
                while (isSending) {

                    short[] currentData = dataRef.get();
                    // Chuyển short[] thành byte[] (6 bytes)
                    ByteBuffer.wrap(byteBuffer)
                            .order(ByteOrder.LITTLE_ENDIAN)
                            .putShort(currentData[0])  // j1Y
                            .putShort(currentData[1])  // j2X
                            .putShort(currentData[2]) // speed
                            ;

                    // thêm một cách "chạm nhẹ" để Android hiểu đây là gói khác:
                    writeCharacteristic.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT); // ép write có response
                    writeCharacteristic.setValue(byteBuffer);
                    boolean success = bluetoothGatt.writeCharacteristic(writeCharacteristic);
                    if (!success) {
                        log("Failed to send BLE data");
                    }

                    // Chờ 50ms trước lần gửi tiếp theo (20 lần/giây)
                    Thread.sleep(50);
                }
            } catch (InterruptedException e) {
                log("Sending thread interrupted");
            }
        });
        sendingThread.start();
    }


    public void updateData(short val1, short val2, short val3) {
        dataRef.set(new short[]{val1, val2, val3});
        }

        public void stopContinuousSending() {
            isSending = false;
            if (sendingThread != null) {
                sendingThread.interrupt();
            }
        }

        private void log(String message) {
            if (logCallback != null) {
                logCallback.log(message);
            }
        }


        public void close() {
            stopContinuousSending();
            // Bạn không cần đóng GATT ở đây, thường đóng ở Activity/Service khi disconnect
        }
    }


