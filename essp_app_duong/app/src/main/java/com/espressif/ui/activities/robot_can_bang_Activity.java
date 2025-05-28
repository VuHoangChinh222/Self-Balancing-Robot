package com.espressif.ui.activities;

import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import androidx.activity.EdgeToEdge;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;

import com.espressif.ui.adapters.UDPClient;
import com.espressif.wifi_provisioning.R;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
//import com.espressif.ui.activities.JoystickView; // Ví dụ, sửa đúng theo vị trí của JoystickView

public class robot_can_bang_Activity extends AppCompatActivity {
    private JoystickView joystickRight;
    private TextView tvRight;
    private ImageView cameraView;
    private SeekBar barSpeed;
    private Button btnBackToMain, btnSaveUDPConfig;
    private EditText editIP, editPort;
    private boolean imageLoaded = false;

    private UDPClient udpClient;
    private final ExecutorService executor = Executors.newSingleThreadExecutor();
    private final ExecutorService cameraExecutor = Executors.newSingleThreadExecutor();
    private final ExecutorService udpExecutor = Executors.newSingleThreadExecutor();
    private final Handler uiHandler = new Handler(Looper.getMainLooper());
    private final Handler cameraHandler = new Handler();

    private volatile short j2Y = 0, j2X = 0, speed = 50;
    private short lastY = 0, lastX = 0, lastSpeed = 50;
    private String cameraUrl = "http://192.168.137.171:8080/capture"; // Cập nhật URL mặc định

private  URL url;
private int portInt;
private String portString;



    private volatile boolean isStreaming = false;
    private String ip = "192.168.137.8"; // Thay IP ESP32-CAM ở đây
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        EdgeToEdge.enable(this);
        setContentView(R.layout.activity_robot_can_bang);

        initializeComponents();
        Toast.makeText(this, "Đang kết nối đến: http://" + ip + ":8080/capture", Toast.LENGTH_LONG).show();
        setupJoystickControls();
        setupSpeedControl();
        setupUDPClient();
    }
    private void initializeComponents() {
        joystickRight = findViewById(R.id.joystick_right);
        tvRight = findViewById(R.id.tv_joystick_right);
        cameraView = findViewById(R.id.imageView);
        barSpeed = findViewById(R.id.speed);
        btnBackToMain = findViewById(R.id.backtoMain);
        btnSaveUDPConfig = findViewById(R.id.save);
        editIP = findViewById(R.id.editTextText);
        editPort = findViewById(R.id.editPort);
        btnBackToMain.setOnClickListener(view ->
                startActivity(new Intent(robot_can_bang_Activity.this, EspMainActivity.class))
        );

        btnSaveUDPConfig.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ip=editIP.getText().toString().trim();
                 portString = editPort.getText().toString().trim();

                if (ip.isEmpty() || portString.isEmpty()) {
                    Toast.makeText(robot_can_bang_Activity.this, "Vui lòng nhập đầy đủ IP và Port", Toast.LENGTH_SHORT).show();
                    return;
                }

                int port;
                try {
                    port = Integer.parseInt(portString);
                } catch (NumberFormatException e) {
                    Toast.makeText(robot_can_bang_Activity.this, "Port không hợp lệ!", Toast.LENGTH_SHORT).show();
                    return;
                }
                udpClient.stopContinuousSending();
                udpClient.updateTarget(ip, port);
                udpClient.startContinuousSending();
                Toast.makeText(robot_can_bang_Activity.this, "Cập nhật IP: " + ip + ", Port: " + port, Toast.LENGTH_SHORT).show();
                startMjpegStream();
                // TODO: Lưu ip và port vào biến toàn cục, SharedPreferences, hoặc gửi đi đâu đó
            }
        });

    }
    @Override
    protected void onResume() {
        super.onResume();
//        startMjpegStream();
//        if (udpClient != null) udpClient.startContinuousSending();
    }

    @Override
    protected void onPause() {
        super.onPause();
//        stopMjpegStream();
//        udpClient.stopContinuousSending();
    }

    private void startMjpegStream() {
        isStreaming = true;
        cameraExecutor.execute(() -> {
            HttpURLConnection connection = null;
            MjpegInputStream mjpegInputStream = null;
            try {
                URL url = new URL("http://" + ip + ":8080/capture");
                connection = (HttpURLConnection) url.openConnection();
                connection.setReadTimeout(5000);
                connection.setConnectTimeout(5000);
                connection.connect();
                Thread.sleep(500); // 2 FPS
                InputStream inputStream = connection.getInputStream();
                mjpegInputStream = new MjpegInputStream(inputStream);

                while (isStreaming) {
                    final Bitmap bitmap = mjpegInputStream.readMjpegFrame();
                    if (bitmap != null) {
                        uiHandler.post(() -> cameraView.setImageBitmap(bitmap));
                    }
                }

            } catch (Exception e) {
                Log.e("CameraStream", "Error streaming MJPEG", e);
                uiHandler.post(() ->
                        Toast.makeText(robot_can_bang_Activity.this,
                                "Lỗi kết nối camera: " + e.getMessage(), Toast.LENGTH_SHORT).show());
            } finally {
                if (mjpegInputStream != null) {
                    try {
                        mjpegInputStream.close();
                    } catch (IOException ignored) { }
                }
                if (connection != null) {
                    connection.disconnect();
                }
            }
        });
    }
    private void setupUDPClient() {
        udpClient = new UDPClient(message -> Log.d("UDPClient", message));
        udpClient.init();
        udpClient.startContinuousSending();
    }

    private void setupJoystickControls() {
        joystickRight.setOnJoystickMoveListener((x, y) -> {
            j2Y = clampValue((short) -y, (short) -100, (short) 100);
            j2X = clampValue((short) -x, (short) -100, (short) 100);
            checkAndSendData();
            updateJoystickText();
        });
    }
    private short clampValue(short value, short min, short max) {
        return (short) Math.max(min, Math.min(max, value));
    }
    private void updateJoystickText() {
        uiHandler.post(() -> tvRight.setText(String.format("X: %d | Y: %d | S: %d", j2X, j2Y, speed)));
    }
    private void setupSpeedControl() {
        barSpeed.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                speed = (short) progress;
                checkAndSendData();
            }
            @Override public void onStartTrackingTouch(SeekBar seekBar) {}
            @Override public void onStopTrackingTouch(SeekBar seekBar) {}
        });
    }
    private void checkAndSendData() {
        udpExecutor.execute(() -> {
            if (j2Y != lastY || j2X != lastX || speed != lastSpeed) {
                udpClient.updateData(j2Y, j2X, speed);
                lastY = j2Y;
                lastX = j2X;
                lastSpeed = speed;
            }
        });
    }

    private void stopMjpegStream() {
        isStreaming = false;
    }

    // Class xử lý luồng MJPEG
    public static class MjpegInputStream {
        private final InputStream in;

        public MjpegInputStream(InputStream inputStream) {
            this.in = inputStream;
        }

        // Đọc một frame JPEG từ luồng MJPEG
        public Bitmap readMjpegFrame() throws IOException {
            int prev = 0;
            int cur;
            ByteArrayOutputStream baos = new ByteArrayOutputStream();
            boolean started = false;

            while ((cur = in.read()) != -1) {
                if (prev == 0xFF && cur == 0xD8) { // Start Of Image (SOI)
                    started = true;
                    baos.reset();
                    baos.write(0xFF);
                    baos.write(0xD8);
                } else if (prev == 0xFF && cur == 0xD9 && started) { // End Of Image (EOI)
                    baos.write(cur);
                    break;
                } else if (started) {
                    baos.write(cur);
                }
                prev = cur;
            }

            byte[] jpegData = baos.toByteArray();

            if (jpegData.length > 0) {
                return BitmapFactory.decodeByteArray(jpegData, 0, jpegData.length);
            } else {
                return null;
            }
        }

        public void close() throws IOException {
            in.close();
        }
    }
}