package com.espressif.ui.activities;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.drawable.Drawable;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

import androidx.core.content.ContextCompat;

import com.espressif.wifi_provisioning.R;

public class JoystickView extends View {


    // Ảnh viền (background) và ảnh tâm (center)
    private Drawable backgroundDrawable;
    private Drawable centerDrawable;

    // Tọa độ tâm joystick (vị trí thật để vẽ)
    private float centerX;
    private float centerY;

    // Bán kính của viền joystick và bán kính của tâm joystick
    private float radiusBackground;
    private float radiusCenter;

    // Giá trị X, Y sau khi quy đổi ([-1000, 1000])
    private int currentXValue = 0;
    private int currentYValue = 0;

    // Listener để MainActivity có thể nhận callback
    private OnJoystickMoveListener listener;

    public JoystickView(Context context) {
        super(context);
        init(context);
    }

    public JoystickView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(context);
    }

    public JoystickView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init(context);
    }

    private void init(Context context) {
        // Gắn drawable
        backgroundDrawable = ContextCompat.getDrawable(context, R.drawable.joystick_background);
        centerDrawable = ContextCompat.getDrawable(context, R.drawable.ball);
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);

        // Giả sử ta muốn bán kính viền lớn gấp 5 lần bán kính tâm
        // => ta cần tính ra kích thước khả dụng của View
        // Ta sẽ tính kích thước cho background và center theo tỉ lệ.

        int width = getWidth();
        int height = getHeight();

        // Lấy kích thước nhỏ hơn để đảm bảo là hình vuông
        int size = Math.min(width, height);

        // Ta quy ước radiusCenter = size / 6
        // => radiusBackground = 5 * radiusCenter = size / 6 * 5 = 5size/6
        // (Hoặc bạn có thể tuỳ chỉnh công thức khác. Miễn đảm bảo background > center)
        radiusCenter = size / 6.5f;
        radiusBackground = radiusCenter * 3f; // gấp 2.8 lần

        // Tâm mặc định ở chính giữa
        centerX = width / 2f;
        centerY = height / 2f;

        // Cập nhật bounds cho background
        // background có tâm cũng là (centerX, centerY)
        // width của background = 2 * radiusBackground
        backgroundDrawable.setBounds(
                (int)(centerX - radiusBackground),
                (int)(centerY - radiusBackground),
                (int)(centerX + radiusBackground),
                (int)(centerY + radiusBackground)
        );

        // Cập nhật bounds cho center (tâm)
        centerDrawable.setBounds(
                (int)(centerX - radiusCenter),
                (int)(centerY - radiusCenter),
                (int)(centerX + radiusCenter),
                (int)(centerY + radiusCenter)
        );
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        // Vẽ background
        backgroundDrawable.draw(canvas);
        // Vẽ tâm
        centerDrawable.draw(canvas);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        float x = event.getX();
        float y = event.getY();

        switch (event.getAction()) {
            case MotionEvent.ACTION_DOWN:
            case MotionEvent.ACTION_MOVE:
                // Tính vector (x - centerX, y - centerY)
                float dx = x - getWidth()/2f;
                float dy = y - getHeight()/2f;

                // Tính độ dài từ tâm
                float distance = (float) Math.sqrt(dx*dx + dy*dy);

                // Nếu vượt quá bán kính background, ta giới hạn

//                if (distance > radiusBackground) {
//                    // Chuẩn hoá vector (dx, dy) về độ dài = radiusBackground
//                    float ratio = radiusBackground / distance;
//                    dx *= ratio;
//                    dy *= ratio;
//                }
                // Giới hạn di chuyển sao cho center không vượt quá background (điểm chạm: border của center chạm border của background)
                float maxDistance = radiusBackground - radiusCenter;
                if (distance > maxDistance) {
                    float ratio = maxDistance / distance;
                    dx *= ratio;
                    dy *= ratio;
                }

                // Cập nhật toạ độ tâm joystick
                centerX = getWidth()/2f + dx;
                centerY = getHeight()/2f + dy;

                // Cập nhật bounds cho centerDrawable
                centerDrawable.setBounds(
                        (int)(centerX - radiusCenter),
                        (int)(centerY - radiusCenter),
                        (int)(centerX + radiusCenter),
                        (int)(centerY + radiusCenter)
                );

                // Tính giá trị X, Y trong khoảng [-100, 100]
                // dx, dy hiện đang trong khoảng [-radiusBackground, radiusBackground]
                float percentX = dx / maxDistance; // [-1, 1]
                float percentY = dy / maxDistance; // [-1, 1]

                currentXValue = (int) (percentX * 100);
                currentYValue = (int) (percentY * 100);

                // Gửi callback
                if (listener != null) {
                    listener.onValueChanged(currentXValue, currentYValue);
                }

                invalidate();
                return true;

            case MotionEvent.ACTION_UP:
            case MotionEvent.ACTION_CANCEL:
                // Trả joystick về giữa
                resetToCenter();
                invalidate();
                return true;
        }
        return super.onTouchEvent(event);
    }

    /**
     * Đặt joystick về vị trí trung tâm (0,0)
     */
    private void resetToCenter() {
        centerX = getWidth() / 2f;
        centerY = getHeight() / 2f;
        centerDrawable.setBounds(
                (int)(centerX - radiusCenter),
                (int)(centerY - radiusCenter),
                (int)(centerX + radiusCenter),
                (int)(centerY + radiusCenter)
        );
        currentXValue = 0;
        currentYValue = 0;
        if (listener != null) {
            listener.onValueChanged(currentXValue, currentYValue);
        }
    }

    /**
     * Cho phép Activity lắng nghe sự kiện thay đổi giá trị joystick
     */
    public void setOnJoystickMoveListener(OnJoystickMoveListener listener) {
        this.listener = listener;
    }

    /**
     * Interface callback
     */
    public interface OnJoystickMoveListener {
        void onValueChanged(int xValue, int yValue);
    }
}