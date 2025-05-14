package com.espressif.ui.adapters;

import java.net.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.atomic.AtomicReference;

public class UDPClient {
    public volatile String serverIP;
    public volatile int serverPort;
    private DatagramSocket socket;
    private volatile boolean isSending = false;
    private Thread sendingThread;
    private final AtomicReference<short[]> dataRef = new AtomicReference<>(new short[3]);
    private final byte[] byteBuffer = new byte[6];
    private LogCallback logCallback;

    public interface LogCallback {
        void log(String message);
    }

    public UDPClient(LogCallback logCallback) {
        this.logCallback = logCallback;
        serverIP = "192.168.1.50";
        serverPort = 6500;
    }

    public void init() {
        try {
            socket = new DatagramSocket();
            socket.setSendBufferSize(64 * 1024);
        } catch (Exception e) {
            log("Init error: " + e.getMessage());
        }
    }

    public void startContinuousSending() {
        if (isSending) return;
        isSending = true;

        sendingThread = new Thread(() -> {
            try {
                InetAddress address = InetAddress.getByName(serverIP);
                DatagramPacket packet = new DatagramPacket(byteBuffer, byteBuffer.length, address, serverPort);

                while (isSending) {
                    short[] currentData = dataRef.get();
                    synchronized (byteBuffer) {
                        ByteBuffer.wrap(byteBuffer).order(ByteOrder.BIG_ENDIAN)
                                .putShort(currentData[0])
                                .putShort(currentData[1])
                                .putShort(currentData[2]);
                    }
                    socket.send(packet);
                    Thread.sleep(1); // Giảm tải CPU nhưng vẫn đảm bảo tốc độ gửi nhanh
                }
            } catch (Exception e) {
                log("Sending error: " + e.getMessage());
            }
        });
        sendingThread.setPriority(Thread.MAX_PRIORITY);
        sendingThread.start();
    }

    public void updateData(short val1, short val2, short val3) {
        short[] newData = {val1, val2, val3};
        dataRef.set(newData);
    }

    public void updateTarget(String ip, int port) {
        try {
            InetAddress.getByName(ip); // Validate IP
            serverIP = ip;
            serverPort = port;
        } catch (UnknownHostException e) {
            log("Invalid IP: " + ip);
        }
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
        if (socket != null && !socket.isClosed()) {
            socket.close();
        }
    }
}
