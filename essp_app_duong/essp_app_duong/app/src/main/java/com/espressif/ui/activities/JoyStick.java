//package com.espressif.ui.activities;
//
//import androidx.appcompat.app.AppCompatActivity;
//
//import android.content.Intent;
//import android.content.pm.ActivityInfo;
//import android.os.Bundle;
//import android.os.Handler;
//import android.os.Looper;
//import android.widget.Button;
//import android.widget.EditText;
//import android.widget.ImageView;
//import android.widget.SeekBar;
//import android.widget.TextView;
//import android.widget.Toast;
//import com.espressif.ui.adapters.UDPClient;
//import com.espressif.wifi_provisioning.R;
//
//import java.io.InputStream;
//import java.net.HttpURLConnection;
//import java.net.URL;
//import java.util.concurrent.ExecutorService;
//import java.util.concurrent.Executors;
//
//import android.graphics.Bitmap;
//import android.graphics.BitmapFactory;
//
//public class JoyStick extends AppCompatActivity {
//    private JoystickView  joystickRight;
//    private TextView tvRight;
//    private UDPClient udpClient;
//    private SeekBar barSpeed;
//    private volatile short j2Y = 0, j2X = 0, speed = 50;
//    private short lastY = 0, lastX = 0, lastSpeed = 50;
//    private final Handler uiHandler = new Handler(Looper.getMainLooper());
//    private final ExecutorService executor = Executors.newSingleThreadExecutor();
//    private Button btnBackToMain;
//    private Button btnSaveUDPConfig;
//    public EditText editIP, editPort;
//    private ImageView cameraView;
//    private Handler handler = new Handler();
//    private static String CAMERA_URL ="http://192.168.100.1/";
//    @Override
//    protected void onCreate(Bundle savedInstanceState) {
//        super.onCreate(savedInstanceState);
//        setContentView(R.layout.activity_joy_stick);
//        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
//        initializeComponents();
//        setupUDPClient();
//        setupControls();
//        btnBackToMain = findViewById(R.id.backtoMain);
//        btnBackToMain.setOnClickListener(view -> {
//            Intent intent =new Intent(JoyStick.this,EspMainActivity.class);
//            startActivity(intent);
//        });
//        btnSaveUDPConfig =findViewById(R.id.save);
//        editIP=findViewById(R.id.editTextText);
//        editPort=findViewById(R.id.editPort);
//        btnSaveUDPConfig.setOnClickListener(view -> {
//            String ip = editIP.getText().toString();
//            String portString= editPort.getText().toString();
//            int portInt=Integer.parseInt(portString);
//            udpClient.updateTarget(ip,portInt);
//            String txt= "Update IP:"+ip+", port:"+portString+ " successful";
//            Toast.makeText(JoyStick.this,
//                            txt,
//                            Toast.LENGTH_SHORT)
//                    .show();
//            CAMERA_URL = "http://" + ip + "/";
//        });
//        cameraView.findViewById(R.id.imageView);
//        handler.post(fetchRunnable);
//    }
//
//    private Runnable fetchRunnable = new Runnable() {
//        @Override
//        public void run() {
//            new Thread(() -> {
//                try {
//                    // Mở kết nối HTTP
//                    URL url = new URL(CAMERA_URL);
//                    HttpURLConnection conn = (HttpURLConnection) url.openConnection();
//                    conn.setDoInput(true);
//                    conn.connect();
//
//                    // Đọc stream và giải mã thành Bitmap
//                    InputStream is = conn.getInputStream();
//                    final Bitmap bitmap = BitmapFactory.decodeStream(is);
//                    is.close();
//                    conn.disconnect();
//
//                    // Cập nhật lên UI thread
//                    runOnUiThread(() -> cameraView.setImageBitmap(bitmap));
//                } catch (Exception e) {
//                    e.printStackTrace();
//                }
//
//                // Lặp lại sau 200ms
//                handler.postDelayed(this, 100);
//            }).start();
//        }
//    };
//
//    private void initializeComponents() {
//        tvRight = findViewById(R.id.tv_joystick_right);
//        joystickRight = findViewById(R.id.joystick_right);
//        barSpeed = findViewById(R.id.speed);
//    }
//
//    private void setupUDPClient() {
//        udpClient = new UDPClient(message -> System.out.println(message));
//        udpClient.init();
//        udpClient.startContinuousSending();
//    }
//
//    private void setupControls() {
//        barSpeed.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
//            @Override
//            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
//                speed = (short) progress;
//                checkAndSendData();
//            }
//            @Override public void onStartTrackingTouch(SeekBar seekBar) {}
//            @Override public void onStopTrackingTouch(SeekBar seekBar) {}
//        });
//
//
//        joystickRight.setOnJoystickMoveListener((x, y) -> {
//            j2Y=  clampValue((short) -y, (short) -100, (short) 100);
//            j2X = clampValue((short) -x, (short) -100, (short) 100);
//            checkAndSendData();
//            updateUiDelayed();
//        });
//    }
//
//    private void checkAndSendData() {
//        executor.execute(() -> {
//            if (dataChanged()) {
//                udpClient.updateData(j2Y, j2X, speed);
//                updateLastValues();
//            }
//        });
//    }
//
//    private boolean dataChanged() {
//        return j2Y != lastY || j2X != lastX || speed != lastSpeed;
//    }
//
//    private void updateLastValues() {
//        lastY = j2Y;
//        lastX = j2X;
//        lastSpeed = speed;
//    }
//
//    private short clampValue(short value, short min, short max) {
//        return value < min ? min : (value > max ? max : value);
//    }
//
//    private void updateUiDelayed() {
//        uiHandler.post(() -> {
//            tvRight.setText(String.format("X: %d | Y: %d | S: %d", j2X,j2Y, speed));
//        });
//    }
//
//    @Override
//    protected void onPause() {
//        super.onPause();
//        udpClient.stopContinuousSending();
//    }
//
//    @Override
//    protected void onResume() {
//        super.onResume();
//        if (udpClient != null) {
//            udpClient.startContinuousSending();
//        }
//    }
//
//    @Override
//    protected void onDestroy() {
//        super.onDestroy();
//        executor.shutdown();
//        udpClient.close();
//        handler.removeCallbacks(fetchRunnable);
//    }
//}
package com.espressif.ui.activities;

import androidx.appcompat.app.AppCompatActivity;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;
import com.espressif.ui.adapters.UDPClient;
import com.espressif.wifi_provisioning.R;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

public class JoyStick extends AppCompatActivity {
    private JoystickView joystickRight;
    private TextView tvRight;
    private UDPClient udpClient;
    private SeekBar barSpeed;
    private volatile short j2Y = 0, j2X = 0, speed = 50;
    private short lastY = 0, lastX = 0, lastSpeed = 50;
    private final Handler uiHandler = new Handler(Looper.getMainLooper());
    private final ExecutorService executor = Executors.newSingleThreadExecutor();
    private Button btnBackToMain;
    private Button btnSaveUDPConfig;
    private EditText editIP, editPort;
    private ImageView cameraView;
    private Handler handler = new Handler();
    private static String CAMERA_URL = "http://192.168.100.1/";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_joy_stick);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);

        initializeComponents();
        setupUDPClient();
        setupControls();

        btnBackToMain = findViewById(R.id.backtoMain);
        btnBackToMain.setOnClickListener(view -> {
            startActivity(new Intent(JoyStick.this, EspMainActivity.class));
        });

        btnSaveUDPConfig = findViewById(R.id.save);
        editIP = findViewById(R.id.editTextText);
        editPort = findViewById(R.id.editPort);
        btnSaveUDPConfig.setOnClickListener(view -> {
            String ip = editIP.getText().toString().trim();
            String portString = editPort.getText().toString().trim();
            if (ip.isEmpty() || portString.isEmpty()) {
                Toast.makeText(this, "Vui lòng nhập IP và Port", Toast.LENGTH_SHORT).show();
                return;
            }
            int portInt;
            try {
                portInt = Integer.parseInt(portString);
            } catch (NumberFormatException e) {
                Toast.makeText(this, "Port không hợp lệ", Toast.LENGTH_SHORT).show();
                return;
            }
            udpClient.updateTarget(ip, portInt);
            CAMERA_URL = ip.startsWith("http") ? ip : "http://" + ip;
            if (!CAMERA_URL.endsWith("/")) CAMERA_URL += "/";
            Toast.makeText(JoyStick.this,
                    "Update IP: " + CAMERA_URL + ", port: " + portInt,
                    Toast.LENGTH_SHORT).show();
        });

        // Correctly find ImageView
        cameraView = findViewById(R.id.imageView);

        // Start image fetch loop
        handler.post(fetchRunnable);
    }

    private Runnable fetchRunnable = new Runnable() {
        @Override
        public void run() {
            executor.execute(() -> {
                try {
                    URL url = new URL(CAMERA_URL);
                    HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                    conn.setDoInput(true);
                    conn.connect();
                    InputStream is = conn.getInputStream();
                    final Bitmap bitmap = BitmapFactory.decodeStream(is);
                    is.close();
                    conn.disconnect();
                    runOnUiThread(() -> cameraView.setImageBitmap(bitmap));
                } catch (Exception e) {
                    Log.e("CameraFetch", "Error loading image", e);
                    e.printStackTrace();
                }
                handler.postDelayed(this, 200);
            });
        }
    };

    private void initializeComponents() {
        tvRight = findViewById(R.id.tv_joystick_right);
        joystickRight = findViewById(R.id.joystick_right);
        barSpeed = findViewById(R.id.speed);
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
            }
            @Override public void onStartTrackingTouch(SeekBar seekBar) {}
            @Override public void onStopTrackingTouch(SeekBar seekBar) {}
        });

        joystickRight.setOnJoystickMoveListener((x, y) -> {
            j2Y = clampValue((short) -y, (short) -100, (short) 100);
            j2X = clampValue((short) -x, (short) -100, (short) 100);
            checkAndSendData();
            updateUiDelayed();
        });
    }

    private void checkAndSendData() {
        executor.execute(() -> {
            if (dataChanged()) {
                udpClient.updateData(j2Y, j2X, speed);
                updateLastValues();
            }
        });
    }

    private boolean dataChanged() {
        return j2Y != lastY || j2X != lastX || speed != lastSpeed;
    }

    private void updateLastValues() {
        lastY = j2Y;
        lastX = j2X;
        lastSpeed = speed;
    }

    private short clampValue(short value, short min, short max) {
        return value < min ? min : (value > max ? max : value);
    }

    private void updateUiDelayed() {
        uiHandler.post(() -> tvRight.setText(String.format("X: %d | Y: %d | S: %d", j2X, j2Y, speed)));
    }

    @Override
    protected void onPause() {
        super.onPause();
        udpClient.stopContinuousSending();
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (udpClient != null) udpClient.startContinuousSending();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        executor.shutdown();
        udpClient.close();
        handler.removeCallbacks(fetchRunnable);
    }
}
