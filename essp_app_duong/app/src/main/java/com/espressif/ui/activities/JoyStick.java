package com.espressif.ui.activities;

import androidx.appcompat.app.AppCompatActivity;

import android.app.PendingIntent;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothManager;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.ActivityInfo;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import com.espressif.ui.adapters.BLEClient;
import com.espressif.ui.adapters.BLEManager;
import com.espressif.ui.adapters.Bluetooth_Classis;
import com.espressif.ui.adapters.KalmanFilter;
import com.espressif.ui.adapters.UDPClient;
import com.espressif.wifi_provisioning.R;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class JoyStick extends AppCompatActivity {
    private BluetoothGatt bluetoothGatt;
    private BluetoothGattCharacteristic writeCharacteristic;
    private JoystickView joystickLeft, joystickRight;
    private TextView tvLeft, tvRight;
    private KalmanFilter kalmanX = new KalmanFilter(0.01f, 5f, 1f, 0f);
    private KalmanFilter kalmanY = new KalmanFilter(0.01f, 5f, 1f, 0f);

    private UDPClient udpClient;
    private UsbDeviceConnection usbConnection;
    private SeekBar barSpeed;
    private UsbEndpoint usbEndpoint;
    private volatile short j1Y = 0, j2X = 0, speed = 50;
    private short lastY = 0, lastX = 0, lastSpeed = 50;
    private final Handler uiHandler = new Handler(Looper.getMainLooper());
    private final ExecutorService executor = Executors.newSingleThreadExecutor();
    private static final String TAG = "USB_HID";
    private static final String ACTION_USB_PERMISSION = "com.espressif.USB_PERMISSION";
    private UsbManager usbManager;
    private UsbDevice usbDevice;
    private Button connect_ble;
    private BLEClient bleClient;
    private Bluetooth_Classis bluetoothClassis;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_joy_stick);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);

       // Nhận tên thiết bị từ Intent
         String deviceName = getIntent().getStringExtra("DEVICE_NAME");
        connect_ble = findViewById(R.id.btn_conect_ble);
        connect_ble.setOnClickListener(v -> {
            // Tạo Intent để chuyển sang JoystickActivity
            Intent intent = new Intent(JoyStick.this, Connect_BLE_Activity.class);
            startActivity(intent);
        });
        if (deviceName != null) {
            connect_ble.setText("Connected: " + deviceName);

        }
        bluetoothClassis = new Bluetooth_Classis();
        // Lấy GATT và Characteristic từ BLEManager
        bluetoothGatt = BLEManager.getInstance().getBluetoothGatt();
        writeCharacteristic = BLEManager.getInstance().getWriteCharacteristic();
        // Khởi tạo BLEClient
    bleClient = new BLEClient(message -> Log.d(TAG, message));
    // Cấu hình GATT và Characteristic (giả sử bạn đã có chúng từ kết nối BLE trước đó)
    bleClient.setGattAndCharacteristic(bluetoothGatt, writeCharacteristic);

    // Bắt đầu gửi dữ liệu liên tục
    bleClient.startContinuousSending();

        initializeComponents();
        setupUDPClient();
        setupControls();
    }
    private void sendJoystickDataToHC05() {
        if (bluetoothClassis != null) {
            String data = String.format("J1Y:%d,J2X:%d,Speed:%d", j1Y, j2X, speed);
            boolean success = bluetoothClassis.sendData(data);
            if (success) {
                Log.d(TAG, "Joystick data sent to HC-05: " + data);
            } else {
                Log.e(TAG, "Failed to send joystick data to HC-05");
            }
        } else {
            Log.e(TAG, "Bluetooth_Classis is not initialized");
        }
    }

    private void initializeComponents() {
        tvLeft = findViewById(R.id.tv_joystick_left);
        tvRight = findViewById(R.id.tv_joystick_right);
        joystickLeft = findViewById(R.id.joystick_left);
        joystickRight = findViewById(R.id.joystick_right);
        barSpeed = findViewById(R.id.speed);
        IntentFilter filter = new IntentFilter();
        usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED); // Khi USB cắm vào
        filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED); // Khi USB rút ra
        filter.addAction(ACTION_USB_PERMISSION); // Khi yêu cầu cấp phép USB


    }

    public void sendData(byte[] data) {
        if (bluetoothGatt == null || writeCharacteristic == null) {
            Log.e(TAG, "BLE GATT or Characteristic not ready!");
//            log("BLE GATT or Characteristic not ready!");
            return;
        }
        writeCharacteristic.setValue(data);
        boolean success = bluetoothGatt.writeCharacteristic(writeCharacteristic);
        if (!success) {
            Log.e(TAG, "Failed to write joystick data via BLE");

//            log("Failed to write joystick data via BLE");
        }
    }
    // Hàm chuyển đổi byte array thành chuỗi hex để debug
    private String bytesToHex(byte[] bytes) {
        StringBuilder sb = new StringBuilder();
        for (byte b : bytes) {
            sb.append(String.format("%02X ", b));
        }
        return sb.toString().trim();
    }



    private void requestUsbPermission() {
        PendingIntent permissionIntent = PendingIntent.getBroadcast(this, 0,
                new Intent(ACTION_USB_PERMISSION), PendingIntent.FLAG_MUTABLE );
        usbManager.requestPermission(usbDevice, permissionIntent);
    }
    private void setupUDPClient() {
        udpClient = new UDPClient(message -> System.out.println(message));
        udpClient.init();
        udpClient.startContinuousSending();
    }

    private void setupControls() {
        barSpeed.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                speed = (short) progress;
                checkAndSendData();
                sendJoystickDataToHC05(); // Gửi dữ liệu đến HC-05 khi
//                sendJoystickData();
            }
            @Override public void onStartTrackingTouch(SeekBar seekBar) {}
            @Override public void onStopTrackingTouch(SeekBar seekBar) {}
        });

        joystickLeft.setOnJoystickMoveListener((x, y) -> {
            j1Y = clampValue((short) -y, (short) -1000, (short) 1000);
             checkAndSendData();
             sendJoystickDataToHC05(); // Gửi dữ liệu đến HC-05 khi
             updateUiDelayed();
//            sendJoystickData();

        });

        joystickRight.setOnJoystickMoveListener((x, y) -> {
            j2X = clampValue((short) -x, (short) -1000, (short) 1000);
             checkAndSendData();
             sendJoystickDataToHC05(); // Gửi dữ liệu đến HC-05 khi
             updateUiDelayed();
//            sendJoystickData();

        });
    }
    private void logData() {
        Log.d(TAG, String.format("Sending data: j1Y=%d, j2X=%d, speed=%d", j1Y, j2X, speed));
    }
    private void checkAndSendData() {
        executor.execute(() -> {
            if (dataChanged()) { // Kiểm tra nếu có bất kỳ giá trị nào thay đổi
                logData(); // Ghi log dữ liệu trước khi gửi
                bleClient.updateData(j1Y, j2X, speed); // Cập nhật dữ liệu vào BLEClient
                udpClient.updateData(j1Y, j2X, speed); // Cập nhật dữ liệu vào UDPClient (nếu cần)
                updateLastValues(); // Cập nhật giá trị cuối cùng
            }
        });
    }

    private boolean dataChanged() {
        return j1Y != lastY || j2X != lastX || speed != lastSpeed;
    }

    private void updateLastValues() {
        lastY = j1Y;
        lastX = j2X;
        lastSpeed = speed;
    }

    private short clampValue(short value, short min, short max) {
        return value < min ? min : (value > max ? max : value);
    }

    private void updateUiDelayed() {
        uiHandler.post(() -> {
            tvLeft.setText(String.format("Y: %d", j1Y));
            tvRight.setText(String.format("X: %d | S: %d", j2X, speed));
        });
    }

    @Override
    protected void onPause() {
        super.onPause();
        udpClient.stopContinuousSending();
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (udpClient != null) {
            udpClient.startContinuousSending();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        executor.shutdown();
        udpClient.close();
    }
}
