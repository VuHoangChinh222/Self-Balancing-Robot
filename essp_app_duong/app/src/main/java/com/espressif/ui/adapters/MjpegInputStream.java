package com.espressif.ui.adapters;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
public class MjpegInputStream {
    private final InputStream in;

    public MjpegInputStream(InputStream inputStream) {
        this.in = inputStream;
    }

    /**
     * Đọc một frame JPEG từ luồng MJPEG.
     * Luồng MJPEG gồm các ảnh JPEG nối tiếp nhau,
     * frame bắt đầu bằng 0xFFD8 (SOI), kết thúc 0xFFD9 (EOI).
     * Hàm này sẽ đọc đến khi tìm thấy một frame hoàn chỉnh,
     * rồi decode thành Bitmap trả về.
     */
    public Bitmap readMjpegFrame() throws IOException {
        int prev = 0;
        int cur;
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        boolean started = false;

        while ((cur = in.read()) != -1) {
            if (prev == 0xFF && cur == 0xD8) { // SOI - start of image
                started = true;
                baos.reset();
                baos.write(0xFF);
                baos.write(0xD8);
            } else if (prev == 0xFF && cur == 0xD9 && started) { // EOI - end of image
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
