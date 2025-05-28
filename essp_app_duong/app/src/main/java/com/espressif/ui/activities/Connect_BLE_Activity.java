package com.espressif.ui.activities;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.Button;
import android.widget.ListView;
import android.widget.ProgressBar;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.espressif.ui.adapters.BLEManager;
import com.espressif.ui.adapters.BleDeviceListAdapter;
import com.espressif.ui.models.BleDevice;
import com.espressif.wifi_provisioning.R;

import java.util.ArrayList;
import java.util.Set;

public class Connect_BLE_Activity extends AppCompatActivity {

    private static final String TAG = "Connect_BLE_Activity";
    private static final int REQUEST_BLUETOOTH_PERMISSIONS = 1;

    private BluetoothAdapter bluetoothAdapter;
    private ArrayList<BleDevice> deviceList;
    private BleDeviceListAdapter adapter;
    private ListView listView;
    private ProgressBar progressBar;
    private Button btnScan;
    private BluetoothDevice bluetoothDevice;

    private BluetoothDevice connectedDevice;
    private Handler handler = new Handler();
    private BluetoothGatt bluetoothGatt;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_connect_ble);

        // Khởi tạo giao diện
        listView = findViewById(R.id.ble_devices_list);
        progressBar = findViewById(R.id.progress_bar);
        btnScan = findViewById(R.id.btn_scan);

        deviceList = new ArrayList<>();
        adapter = new BleDeviceListAdapter(this, R.layout.item_ble_scan, deviceList);
        listView.setAdapter(adapter);

        // Lấy BluetoothAdapter
        BluetoothManager bluetoothManager = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        bluetoothAdapter = bluetoothManager.getAdapter();

        // Kiểm tra quyền Bluetooth
        if (!hasBluetoothPermissions()) {
            requestBluetoothPermissions();
        }

        // Nút quét thiết bị
        btnScan.setOnClickListener(v -> {
            if (hasBluetoothPermissions()) {
                startScan();
            } else {
                Toast.makeText(this, "Bluetooth permissions are required", Toast.LENGTH_SHORT).show();
                requestBluetoothPermissions();
            }
        });

        // Xử lý khi chọn thiết bị từ danh sách
        listView.setOnItemClickListener((AdapterView<?> parent, View view, int position, long id) -> {
            BleDevice bleDevice = deviceList.get(position);
            connectToDevice(bleDevice.getBluetoothDevice());
        });
    }

    private boolean hasBluetoothPermissions() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            return ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) == PackageManager.PERMISSION_GRANTED &&
                    ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED;
        } else {
            return ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED;
        }
    }

    private void requestBluetoothPermissions() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT},
                    REQUEST_BLUETOOTH_PERMISSIONS);
        } else {
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.ACCESS_FINE_LOCATION},
                    REQUEST_BLUETOOTH_PERMISSIONS);
        }
    }

    private void startScan() {
        if (bluetoothAdapter == null || !bluetoothAdapter.isEnabled()) {
            Toast.makeText(this, "Bluetooth is not enabled", Toast.LENGTH_SHORT).show();
            return;
        }

        progressBar.setVisibility(View.VISIBLE);
        deviceList.clear();
        adapter.notifyDataSetChanged();

        // Lấy danh sách thiết bị đã ghép nối
        Set<BluetoothDevice> pairedDevices = bluetoothAdapter.getBondedDevices();
        for (BluetoothDevice device : pairedDevices) {
            BleDevice bleDevice = new BleDevice();
            bleDevice.setName(device.getName());
            bleDevice.setBluetoothDevice(device);
            deviceList.add(bleDevice);
        }
        adapter.notifyDataSetChanged();

        // Quét thiết bị mới
        bluetoothAdapter.startDiscovery();
        handler.postDelayed(() -> {
            bluetoothAdapter.cancelDiscovery();
            progressBar.setVisibility(View.GONE);
        }, 10000); // Dừng quét sau 10 giây
    }

    private void connectToDevice(BluetoothDevice device) {
        if (!hasBluetoothPermissions()) {
            Toast.makeText(this, "Bluetooth permissions are required", Toast.LENGTH_SHORT).show();
            requestBluetoothPermissions();
            return;
        }
    
        progressBar.setVisibility(View.VISIBLE);
        connectedDevice = device;
    
        new Thread(() -> {
            try {
                bluetoothGatt = device.connectGatt(this, false, new BluetoothGattCallback() {
                    @Override
                    public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
                        super.onConnectionStateChange(gatt, status, newState);
                        if (newState == BluetoothGatt.STATE_CONNECTED) {
                            Log.d(TAG, "Connected to GATT server.");
                            gatt.discoverServices(); // Khám phá các dịch vụ BLE
                        } else if (newState == BluetoothGatt.STATE_DISCONNECTED) {
                            Log.d(TAG, "Disconnected from GATT server.");
                        }
                    }
    
                    @Override
                    public void onServicesDiscovered(BluetoothGatt gatt, int status) {
                        super.onServicesDiscovered(gatt, status);
                        if (status == BluetoothGatt.GATT_SUCCESS) {
                            for (BluetoothGattService service : gatt.getServices()) {
                                for (BluetoothGattCharacteristic characteristic : service.getCharacteristics()) {
                                    if ((characteristic.getProperties() & BluetoothGattCharacteristic.PROPERTY_WRITE) > 0) {
                                        BLEManager.getInstance().setBluetoothGatt(gatt);
                                        BLEManager.getInstance().setWriteCharacteristic(characteristic);
                                        Log.d(TAG, "Write characteristic found.");
                                        break;
                                    }
                                }
                            }
                        }
                    }
                });
    
                runOnUiThread(() -> {
                    progressBar.setVisibility(View.GONE);
                    Toast.makeText(this, "Connected to " + device.getName(), Toast.LENGTH_SHORT).show();
    
                    // Chuyển sang giao diện joystick
                    Intent intent = new Intent(Connect_BLE_Activity.this, JoyStick.class);
                    intent.putExtra("DEVICE_NAME", device.getName());
                    startActivity(intent);
                    finish();
                });
            } catch (Exception e) {
                Log.e(TAG, "Error connecting to device", e);
                runOnUiThread(() -> {
                    progressBar.setVisibility(View.GONE);
                    Toast.makeText(this, "Failed to connect to device", Toast.LENGTH_SHORT).show();
                });
            }
        }).start();
    }

}