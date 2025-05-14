package com.espressif.ui.activities;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.widget.TextView;
import android.widget.Toast;

import androidx.activity.EdgeToEdge;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;

import com.espressif.ui.adapters.UDPClient;
import com.espressif.wifi_provisioning.R;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class ControlActivity extends AppCompatActivity {
    private static final String TAG = "USB_HID";
    private static final String ACTION_USB_PERMISSION = "com.espressif.USB_PERMISSION";

    private UsbManager usbManager;
    private UsbDevice usbDevice;
    private UsbDeviceConnection usbConnection;
    private UsbEndpoint usbEndpoint;
    private TextView txtJoystickData;
    private TextView txtJoystick;
    private UDPClient udpClient;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        EdgeToEdge.enable(this);
        setContentView(R.layout.activity_control);
        txtJoystick=findViewById(R.id.txtView_joystick);
        usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
        // Tạo IntentFilter và đăng ký Receiver
        IntentFilter filter = new IntentFilter();
        filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED); // Khi USB cắm vào
        filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED); // Khi USB rút ra
        filter.addAction(ACTION_USB_PERMISSION); // Khi yêu cầu cấp phép USB
        registerReceiver(usbReceiver, filter);

    }
    private void checkUsbDevice() {
        for (UsbDevice device : usbManager.getDeviceList().values()) {
            if (device.getVendorId() == 1155 && device.getProductId() == 22352) { // VID & PID STM32
                usbDevice = device;
                requestUsbPermission();
                break;
            }
        }
    }

    private void requestUsbPermission() {
        PendingIntent permissionIntent = PendingIntent.getBroadcast(this, 0,
                new Intent(ACTION_USB_PERMISSION), PendingIntent.FLAG_MUTABLE );
        usbManager.requestPermission(usbDevice, permissionIntent);
    }

    private final BroadcastReceiver usbReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();

            if (ACTION_USB_PERMISSION.equals(action)) {
                synchronized (this) {
                    UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                    if (device != null && intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                        setupUsbHid(device);
                    } else {
                        Toast.makeText(context, "USB Permission Denied", Toast.LENGTH_SHORT).show();
                    }
                }
            } else if (UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(action)) {
                checkUsbDevice();
            } else if (UsbManager.ACTION_USB_DEVICE_DETACHED.equals(action)) {
                if (usbDevice != null) {
                    Toast.makeText(context, "USB Device Disconnected", Toast.LENGTH_SHORT).show();
                    usbDevice = null;
                }
            }
        }
    };

    private void setupUsbHid(UsbDevice device) {

        UsbInterface usbInterface = device.getInterface(0);
        usbEndpoint = usbInterface.getEndpoint(0); // Chọn Endpoint đầu tiên

        usbConnection = usbManager.openDevice(device);
        if (usbConnection.claimInterface(usbInterface, true)) {
            Toast.makeText(this, "USB HID Connected", Toast.LENGTH_SHORT).show();
            readJoystickData();
        } else {
            Toast.makeText(this, "Failed to Claim USB Interface", Toast.LENGTH_SHORT).show();
            usbConnection.close();
        }
    }

    private void readJoystickData() {
        new Thread(() -> {
            byte[] buffer = new byte[6]; // 3 giá trị short (6 byte)
            while (usbDevice != null) {
                int bytesRead = usbConnection.bulkTransfer(usbEndpoint, buffer, buffer.length, 100);
                if (bytesRead > 0) {
                    short rawX = ByteBuffer.wrap(buffer, 0, 2).order(ByteOrder.LITTLE_ENDIAN).getShort();
                    short rawY = ByteBuffer.wrap(buffer, 2, 2).order(ByteOrder.LITTLE_ENDIAN).getShort();
                    short rawZ = ByteBuffer.wrap(buffer, 4, 2).order(ByteOrder.LITTLE_ENDIAN).getShort();
//                    int x = (rawX - 1616) * 1000 / 1616;
//                    int y = (rawY - 1616) * 1000 / 1616;
//                    int z = (rawZ - 1616) * 1000 / 1616;

                    onUsbDataReceived(rawX,rawY,rawZ);
//                    setupUDPClient();
//                    udpClient.updateData(rawX,rawY,rawZ);
//                    onUsbDataReceived(rawX,rawY,rawZ);
                }
            }
        }).start();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        unregisterReceiver(usbReceiver);
        if (usbConnection != null) {
            usbConnection.close();
        }
    }
    public void onUsbDataReceived(final int Xx, final int Xy,final int Yx) {
        runOnUiThread(() -> txtJoystick.setText("Xx: " + Xx + "Xy: " + Xy +"speedYx: " + Yx ));
    }
    private void setupUDPClient() {
        udpClient = new UDPClient(message -> System.out.println(message));
        udpClient.init();
        udpClient.startContinuousSending();
    }

}