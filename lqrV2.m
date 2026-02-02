clear all;
close all;
clc;

fprintf('=========================================================================\n');
fprintf('BẮT ĐẦU THIẾT KẾ BỘ ĐIỀU KHIỂN LQR CHO ROBOT CÂN BẰNG (ĐÃ SỬA LỖI)\n');
fprintf('=========================================================================\n\n');

%% ========== PHẦN 1: THÔNG SỐ ĐỘNG CƠ THỰC TẾ ==========

fprintf('========== THÔNG SỐ ĐỘNG CƠ ==========\n');

% Thông số động cơ BLDC 350W
P_motor = 350;              % W - Công suất động cơ
N_max = 690;                % RPM - Tốc độ tối đa
tau_rated = 4.8;            % N.m - Mô-men tối đa

% Chuyển đổi RPM sang rad/s
omega_max = N_max * 2 * pi / 60;  % rad/s
fprintf('Tốc độ tối đa: %.2f RPM = %.2f rad/s\n', N_max, omega_max);

% Tính hằng số động cơ từ thông số thực tế
% Giả định động cơ hoạt động ở điện áp 42V (10S battery)
V_battery = 42.0;           % V - Điện áp pin 10S (4.2V x 10)

% Hằng số EMF ngược: Ke = V / omega (khi không tải)
% Thực tế Ke thường nhỏ hơn V_battery/omega_max do tổn thất
Ke = V_battery / omega_max * 0.85;  % V/(rad/s)
fprintf('Hằng số EMF ngược Ke: %.4f V/(rad/s)\n', Ke);

% Hằng số mô-men: Kt ≈ Ke (trong đơn vị SI)
Kt = Ke;                    % N.m/A
fprintf('Hằng số mô-men Kt: %.4f N.m/A\n', Kt);

% Dòng điện định mức: I = tau / Kt
I_rated = tau_rated / Kt;   % A
fprintf('Dòng điện định mức: %.2f A\n', I_rated);

% Điện trở cuộn dây: R = V / I (ước tính thô)
R_motor = V_battery / I_rated * 0.5;  % Ohm (hệ số 0.5 do EMF ngược)
fprintf('Điện trở cuộn dây R: %.4f Ohm\n', R_motor);

% Moment quán tính rotor (ước tính cho động cơ 350W)
J_motor = 0.0002;           % kg.m^2 - Moment quán tính rotor

fprintf('\n');

%% ========== PHẦN 2: THÔNG SỐ VẬT LÝ ROBOT ==========

fprintf('========== THÔNG SỐ ROBOT ==========\n');

% --- Kích thước và khối lượng ---
m_total = 12.5;             % kg - Tổng khối lượng robot
wheel_base = 0.26;          % m - Khoảng cách giữa 2 bánh xe (26cm)

% Ước tính bán kính bánh xe từ tốc độ tối đa
% v_max = omega_max * r
% Giả định tốc độ tối đa robot khoảng 7.2 m/s
v_max_robot = 7.2;          % m/s
r = 0.1; % m - Bán kính bánh xe
fprintf('Bán kính bánh xe ước tính: %.4f m (%.2f cm)\n', r, r*100);

% --- Thông số hình học (từ mô tả của bạn) ---
% Chiều cao từ trục động cơ đến mặt đất: 10cm
% Chiều cao tổng: 27cm
% Ngang: 26cm (không tính bánh xe)

% Hộp dưới: 9cm x 19cm x 12cm (cao x dài x rộng)
% Chứa: STM32, 2x ZS-X11H, ICM42688P, ESP32, mạch nguồn 5V
m_box1 = 2.5;               % kg - Khối lượng hộp dưới + thiết bị
L_box1 = 0.19;              % m - Chiều dài
W_box1 = 0.12;              % m - Chiều rộng
H_box1 = 0.09;              % m - Chiều cao
h_box1 = 0.10 + H_box1/2;   % m - Chiều cao trọng tâm từ trục (10cm + 4.5cm)

% Hộp trên: 8.5cm x 16cm x 10cm
% Chứa: Pin 10S-2P (6x9x3.5cm) và Pin 3S-1P
m_box2 = 1.0;               % kg - Khối lượng hộp trên (chỉ vỏ)
L_box2 = 0.16;              % m
W_box2 = 0.10;              % m
H_box2 = 0.085;             % m
h_box2 = 0.10 + 0.09 + H_box2/2;  % m - Trọng tâm hộp trên

% Pin 10S-2P: 6cm x 9cm x 3.5cm
m_battery_10S = 0.65;       % kg - Khối lượng pin 10S-2P (18650: ~45g/cell, 20 cells)
h_battery_10S = h_box2;     % m - Đặt trong hộp trên

% Pin 3S-1P
m_battery_3S = 0.15;        % kg - Khối lượng pin 3S-1P (3 cells)
h_battery_3S = h_box2;      % m - Đặt trong hộp trên

% Động cơ và bánh xe
m_motor = 0.8;              % kg - Khối lượng 1 động cơ BLDC 350W
m_wheel = 0.3;              % kg - Khối lượng 1 bánh xe
h_motor = 0.10;             % m - Chiều cao động cơ (ở trục)

% Các thiết bị khác
m_other = m_total - m_box1 - m_box2 - m_battery_10S - m_battery_3S - 2*m_motor - 2*m_wheel;
h_other = 0.10;             % m - Ước tính

fprintf('Khối lượng chi tiết:\n');
fprintf('  Hộp dưới + thiết bị: %.2f kg\n', m_box1);
fprintf('  Hộp trên: %.2f kg\n', m_box2);
fprintf('  Pin 10S-2P: %.2f kg\n', m_battery_10S);
fprintf('  Pin 3S-1P: %.2f kg\n', m_battery_3S);
fprintf('  2x Động cơ: %.2f kg\n', 2*m_motor);
fprintf('  2x Bánh xe: %.2f kg\n', 2*m_wheel);
fprintf('  Khác: %.2f kg\n', m_other);
fprintf('  TỔNG: %.2f kg\n', m_total);

% --- Tính trọng tâm tổng ---
L = (m_box1*h_box1 + m_box2*h_box2 + m_battery_10S*h_battery_10S + ...
     m_battery_3S*h_battery_3S + m_other*h_other + 2*m_motor*h_motor) / m_total;

fprintf('\nChiều cao trọng tâm từ trục bánh xe: L = %.4f m (%.2f cm)\n', L, L*100);

% --- Moment quán tính ---
% Sử dụng công thức I = m*h^2 + (1/12)*m*(L^2 + W^2) cho hình hộp
I_box1 = m_box1 * h_box1^2 + (1/12)*m_box1*(L_box1^2 + W_box1^2);
I_box2 = m_box2 * h_box2^2 + (1/12)*m_box2*(L_box2^2 + W_box2^2);
I_battery_10S = m_battery_10S * h_battery_10S^2;
I_battery_3S = m_battery_3S * h_battery_3S^2;
I_motor_total = 2 * m_motor * h_motor^2;
I_other = m_other * h_other^2;

% Moment quán tính tổng quanh trục bánh xe
I_body = I_box1 + I_box2 + I_battery_10S + I_battery_3S + I_motor_total + I_other;

fprintf('Moment quán tính tổng: I = %.4f kg.m^2\n', I_body);

% Moment quán tính bánh xe (ước tính như đĩa tròn)
I_wheel = m_wheel * r^2 / 2;
fprintf('Moment quán tính 1 bánh xe: %.6f kg.m^2\n', I_wheel);

% --- Hằng số vật lý ---
g = 9.81;                   % m/s^2 - Gia tốc trọng trường

fprintf('\n');

%% ========== PHẦN 3: XÂY DỰNG MÔ HÌNH TRẠNG THÁI ==========

fprintf('========== MÔ HÌNH TUYẾN TÍNH HÓA ==========\n');

% Vector trạng thái: x = [theta, theta_dot, x_pos, x_dot]'
% theta: góc nghiêng (rad)
% theta_dot: vận tốc góc (rad/s)
% x_pos: vị trí robot (m)
% x_dot: vận tốc tịnh tiến (m/s)

% Input: u = [tau_L, tau_R]' - Mô-men 2 động cơ (N.m)

% Tuyến tính hóa tại điểm cân bằng: [0, 0, 0, 0]

% Ma trận A (Động học hệ thống)
A = zeros(4, 4);
A(1, 2) = 1;  % d(theta)/dt = theta_dot

% theta_ddot = (m*g*L*sin(theta) - m*L*r*x_ddot*cos(theta)) / (I_body + m*L^2)
% Tuyến tính hóa tại theta = 0: sin(theta) ≈ theta, cos(theta) ≈ 1
A(2, 1) = (m_total * g * L) / (I_body + m_total*L^2);

A(3, 4) = 1;  % d(x_pos)/dt = x_dot

% x_ddot từ phương trình chuyển động
% Đơn giản hóa: x_ddot = (tau_total - m*L*theta_ddot) / (m*r^2)
A(4, 1) = -(m_total * g * L) / (m_total * r^2 + I_body/r^2);

fprintf('Ma trận A (Động học):\n');
disp(A);

% Ma trận B (Ảnh hưởng điều khiển)
B = zeros(4, 2);

% Ảnh hưởng của mô-men động cơ lên theta_ddot
B(2, 1) = -1 / ((I_body + m_total*L^2) * r);
B(2, 2) = -1 / ((I_body + m_total*L^2) * r);

% Ảnh hưởng của mô-men động cơ lên x_ddot
coef = 1 / (m_total * r^2 + I_body/r^2 + 2*I_wheel);
B(4, 1) = (1 + I_body/(m_total*L^2)) * coef / r;
B(4, 2) = (1 + I_body/(m_total*L^2)) * coef / r;

fprintf('\nMa trận B (Điều khiển):\n');
disp(B);

% Ma trận C và D
C = eye(4);
D = zeros(4, 2);

% --- Kiểm tra tính điều khiển được ---
controllability_matrix = ctrb(A, B);
rank_ctrb = rank(controllability_matrix);

fprintf('\n--- Kiểm tra tính điều khiển được ---\n');
fprintf('Rank của ma trận điều khiển: %d / %d\n', rank_ctrb, size(A, 1));

if rank_ctrb == size(A, 1)
    fprintf('✓ Hệ thống ĐIỀU KHIỂN ĐƯỢC!\n');
else
    fprintf('✗ Hệ thống KHÔNG điều khiển được!\n');
    error('Không thể thiết kế LQR!');
end

fprintf('\n');

%% ========== PHẦN 4: MÔ HÌNH MỞ RỘNG (5 TRẠNG THÁI) ==========

fprintf('========== MÔ HÌNH MỞ RỘNG (ĐÃ SỬA) ==========\n');
fprintf('⚠ Đã loại bỏ trạng thái yaw và yaw_rate vì không điều khiển được trực tiếp\n');
fprintf('   Để điều khiển yaw, cần thêm cơ chế vi sai vào B_aug\n\n');

% Vector trạng thái mở rộng: 
% x_aug = [theta, theta_dot, x_pos, x_dot, x_integral]'
% Đã loại bỏ yaw và yaw_rate

% Ma trận A mở rộng (5x5)
A_aug = zeros(5, 5);

% Copy phần gốc
A_aug(1:4, 1:4) = A;

% Thêm phương trình tích phân: d(x_integral)/dt = x_pos
A_aug(5, 3) = 1;

fprintf('Ma trận A mở rộng (5x5):\n');
disp(A_aug);

% Ma trận B mở rộng (5x2)
B_aug = zeros(5, 2);

% Copy phần gốc
B_aug(1:4, :) = B;

% Không có input nào ảnh hưởng trực tiếp đến x_integral
B_aug(5, :) = [0, 0];

fprintf('\nMa trận B mở rộng (5x2):\n');
disp(B_aug);

% --- Kiểm tra tính điều khiển được (mở rộng) ---
controllability_matrix_aug = ctrb(A_aug, B_aug);
rank_ctrb_aug = rank(controllability_matrix_aug);

fprintf('\n--- Kiểm tra tính điều khiển được (mở rộng) ---\n');
fprintf('Rank: %d / %d\n', rank_ctrb_aug, size(A_aug, 1));

if rank_ctrb_aug == size(A_aug, 1)
    fprintf('✓ Hệ thống mở rộng ĐIỀU KHIỂN ĐƯỢC!\n');
else
    fprintf('⚠ Hệ thống mở rộng KHÔNG hoàn toàn điều khiển được!\n');
    fprintf('  Vẫn có thể thiết kế LQR nhưng cần thận trọng.\n');
end

fprintf('\n');

%% ========== PHẦN 5: THIẾT KẾ LQR ==========

fprintf('========== THIẾT KẾ LQR ==========\n');

% Ma trận trọng số Q (5x5)
% Q = diag([theta, theta_dot, x_pos, x_dot, x_integral])
Q_aug = diag([200, 10, 10, 10, 1]);

fprintf('Ma trận Q:\n');
disp(Q_aug);

% Ma trận trọng số R (2x2)
% R = diag([tau_L, tau_R])
R_aug = diag([4.5, 4.5]);

fprintf('Ma trận R:\n');
disp(R_aug);

% Tính LQR
try
    [K_aug, S_aug, e_aug] = lqr(A_aug, B_aug, Q_aug, R_aug);
    fprintf('\n✓ LQR đã được tính thành công!\n');
catch ME
    fprintf('\n✗ LỖI khi tính LQR:\n');
    fprintf('   %s\n', ME.message);
    error('Không thể tiếp tục!');
end

fprintf('\nMa trận K (Hệ số LQR) - 2x5:\n');
disp(K_aug);

fprintf('\nCác cực của hệ thống kín:\n');
disp(e_aug);

% Kiểm tra ổn định
if all(real(e_aug) < 0)
    fprintf('✓ Hệ thống STABLE (tất cả cực có phần thực âm)\n');
else
    fprintf('✗ Hệ thống KHÔNG STABLE!\n');
    warning('Hệ thống có cực không ổn định!');
end

fprintf('\n');

%% ========== PHẦN 6: MÔ PHỎNG ==========

fprintf('========== MÔ PHỎNG HỆ THỐNG ==========\n');

% Điều kiện ban đầu: robot nghiêng 10 độ
x0 = zeros(5, 1);
x0(1) = 10 * pi / 180;  % theta = 10 độ

fprintf('Điều kiện ban đầu:\n');
fprintf('  theta = %.2f độ\n', x0(1) * 180/pi);
fprintf('  Các trạng thái khác = 0\n\n');

% Thời gian mô phỏng
t = 0:0.01:3;  % 3 giây

% Tạo hệ thống kín: dx/dt = (A - B*K)*x
A_closed = A_aug - B_aug * K_aug;
sys_closed = ss(A_closed, B_aug, eye(5), zeros(5, 2));

% Mô phỏng đáp ứng
[y, t_sim, x] = initial(sys_closed, x0, t);

% Tính tín hiệu điều khiển
u = -K_aug * x';

fprintf('✓ Mô phỏng hoàn tất!\n');
fprintf('  Góc nghiêng cuối: %.4f độ\n', x(end, 1) * 180/pi);
fprintf('  Vị trí cuối: %.4f m\n', x(end, 3));

fprintf('\n');

%% ========== PHẦN 7: VẼ ĐỒ THỊ ==========

fprintf('========== VẼ ĐỒ THỊ ==========\n');

% Đồ thị 1: Các trạng thái và điều khiển
figure('Name', 'LQR Response', 'Position', [100, 100, 1400, 900]);

% Theta
subplot(3, 3, 1);
plot(t_sim, x(:, 1) * 180/pi, 'b-', 'LineWidth', 2);
grid on;
xlabel('Thời gian (s)');
ylabel('Góc (độ)');
title('Góc nghiêng \theta');
yline(0, 'k--', 'LineWidth', 1);

% Theta_dot
subplot(3, 3, 2);
plot(t_sim, x(:, 2) * 180/pi, 'r-', 'LineWidth', 2);
grid on;
xlabel('Thời gian (s)');
ylabel('Vận tốc góc (độ/s)');
title('Vận tốc góc d\theta/dt');
yline(0, 'k--', 'LineWidth', 1);

% X position
subplot(3, 3, 3);
plot(t_sim, x(:, 3), 'g-', 'LineWidth', 2);
grid on;
xlabel('Thời gian (s)');
ylabel('Vị trí (m)');
title('Vị trí x');
yline(0, 'k--', 'LineWidth', 1);

% X velocity
subplot(3, 3, 4);
plot(t_sim, x(:, 4), 'm-', 'LineWidth', 2);
grid on;
xlabel('Thời gian (s)');
ylabel('Vận tốc (m/s)');
title('Vận tốc dx/dt');
yline(0, 'k--', 'LineWidth', 1);

% X integral
subplot(3, 3, 5);
plot(t_sim, x(:, 5), 'c-', 'LineWidth', 2);
grid on;
xlabel('Thời gian (s)');
ylabel('Tích phân vị trí (m.s)');
title('Tích phân ∫x dt');
yline(0, 'k--', 'LineWidth', 1);

% Phase plot: theta vs theta_dot
subplot(3, 3, 6);
plot(x(:, 1) * 180/pi, x(:, 2) * 180/pi, 'b-', 'LineWidth', 2);
hold on;
plot(x(1, 1) * 180/pi, x(1, 2) * 180/pi, 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(x(end, 1) * 180/pi, x(end, 2) * 180/pi, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
hold off;
grid on;
xlabel('\theta (độ)');
ylabel('d\theta/dt (độ/s)');
title('Phase plot: \theta vs d\theta/dt');
legend('궤đạo', 'Bắt đầu', 'Kết thúc');

% Phase plot: x vs x_dot
subplot(3, 3, 7);
plot(x(:, 3), x(:, 4), 'g-', 'LineWidth', 2);
hold on;
plot(x(1, 3), x(1, 4), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(x(end, 3), x(end, 4), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
hold off;
grid on;
xlabel('x (m)');
ylabel('dx/dt (m/s)');
title('Phase plot: x vs dx/dt');
legend('Quỹ đạo', 'Bắt đầu', 'Kết thúc');

% Control signal - Motor LEFT
subplot(3, 3, 8);
plot(t_sim, u(1, :), 'b-', 'LineWidth', 2);
grid on;
xlabel('Thời gian (s)');
ylabel('Mô-men (N.m)');
title('Tín hiệu điều khiển \tau_L');
yline(tau_rated, 'k--', 'Max', 'LineWidth', 1.5);
yline(-tau_rated, 'k--', 'Min', 'LineWidth', 1.5);

% Control signal - Motor RIGHT
subplot(3, 3, 9);
plot(t_sim, u(2, :), 'r-', 'LineWidth', 2);
grid on;
xlabel('Thời gian (s)');
ylabel('Mô-men (N.m)');
title('Tín hiệu điều khiển \tau_R');
yline(tau_rated, 'k--', 'Max', 'LineWidth', 1.5);
yline(-tau_rated, 'k--', 'Min', 'LineWidth', 1.5);

sgtitle('Đáp ứng hệ thống LQR với nhiễu ban đầu 10 độ', 'FontSize', 14, 'FontWeight', 'bold');

% Đồ thị 2: Chuyển đổi sang PWM
figure('Name', 'PWM Signals', 'Position', [100, 100, 1200, 500]);

% Chuyển mô-men sang PWM
% tau = Kt * I
% V = I * R + Ke * omega
% PWM = (V / V_battery) * PWM_max

PWM_max = 10000;
omega = x(:, 4) / r;  % Vận tốc góc bánh xe

I_L = u(1, :)' / Kt;
I_R = u(2, :)' / Kt;

V_L = I_L * R_motor + Ke * omega;
V_R = I_R * R_motor + Ke * omega;

PWM_L = (V_L / V_battery) * PWM_max;
PWM_R = (V_R / V_battery) * PWM_max;

% Giới hạn PWM
PWM_L = max(min(PWM_L, PWM_max), -PWM_max);
PWM_R = max(min(PWM_R, PWM_max), -PWM_max);

subplot(1, 2, 1);
plot(t_sim, PWM_L, 'b-', 'LineWidth', 2);
hold on;
yline(PWM_max, 'r--', 'Max PWM', 'LineWidth', 1.5);
yline(-PWM_max, 'r--', 'Min PWM', 'LineWidth', 1.5);
yline(0, 'k:', 'LineWidth', 1);
hold off;
grid on;
xlabel('Thời gian (s)');
ylabel('PWM');
title('PWM Động cơ TRÁI');
ylim([-PWM_max*1.2, PWM_max*1.2]);

subplot(1, 2, 2);
plot(t_sim, PWM_R, 'b-', 'LineWidth', 2);
hold on;
yline(PWM_max, 'r--', 'Max PWM', 'LineWidth', 1.5);
yline(-PWM_max, 'r--', 'Min PWM', 'LineWidth', 1.5);
yline(0, 'k:', 'LineWidth', 1);
hold off;
grid on;
xlabel('Thời gian (s)');
ylabel('PWM');
title('PWM Động cơ PHẢI');
ylim([-PWM_max*1.2, PWM_max*1.2]);

sgtitle('Tín hiệu PWM cho ZS-X11H', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('Đã vẽ đồ thị!\n\n');

%% ========== PHẦN 8: PHÂN TÍCH CHI TIẾT ==========

fprintf('========== PHÂN TÍCH CHI TIẾT ==========\n');

tau_max_L = max(abs(u(1, :)));
tau_max_R = max(abs(u(2, :)));
PWM_max_L = max(abs(PWM_L));
PWM_max_R = max(abs(PWM_R));

fprintf('Mô-men tối đa:\n');
fprintf('  Động cơ TRÁI: %.3f N.m (%.1f%% của định mức)\n', tau_max_L, tau_max_L/tau_rated*100);
fprintf('  Động cơ PHẢI: %.3f N.m (%.1f%% của định mức)\n', tau_max_R, tau_max_R/tau_rated*100);

fprintf('\nPWM tối đa:\n');
fprintf('  Động cơ TRÁI: %.0f (%.1f%% của max)\n', PWM_max_L, PWM_max_L/PWM_max*100);
fprintf('  Động cơ PHẢI: %.0f (%.1f%% của max)\n', PWM_max_R, PWM_max_R/PWM_max*100);

if tau_max_L > tau_rated || tau_max_R > tau_rated
    fprintf('\n⚠ CẢNH BÁO: Mô-men yêu cầu vượt quá định mức động cơ!\n');
    fprintf('   Cần giảm Q hoặc tăng R để giảm mô-men điều khiển.\n');
end

if PWM_max_L > PWM_max || PWM_max_R > PWM_max
    fprintf('\n⚠ CẢNH BÁO: PWM vượt quá giới hạn!\n');
else
    fprintf('\n✓ PWM nằm trong giới hạn an toàn.\n');
end

% Tính năng lượng tiêu thụ
energy = trapz(t_sim, abs(u(1, :)).*abs(omega') + abs(u(2, :)).*abs(omega'));
fprintf('\nNăng lượng tiêu thụ ước tính: %.2f J (trong 3 giây)\n', energy);

fprintf('\n');

%% ========== PHẦN 9: THỬ NGHIỆM CÁC CẤU HÌNH ==========

fprintf('========== THỬ NGHIỆM CÁC CẤU HÌNH KHÁC ==========\n\n');

configs = {
    'Aggressive', diag([200, 10, 10, 10, 1]),  diag([4.0, 4.0]);
    'Balanced',   diag([200, 20, 15, 10, 2]),  diag([5.5, 5.5]);
    'Smooth',     diag([100, 10, 5, 5, 1]),    diag([6.0, 6.0]);
    'Position',  diag([150, 15, 50, 20, 10]), diag([4.0, 4.0]);
};

results = cell(size(configs, 1), 5);

for i = 1:size(configs, 1)
    fprintf('--- Cấu hình: %s ---\n', configs{i,1});
    
    Q_test = configs{i,2};
    R_test = configs{i,3};
    
    [K_test, ~, e_test] = lqr(A_aug, B_aug, Q_test, R_test);
    
    % Mô phỏng
    sys_test = ss(A_aug - B_aug*K_test, B_aug, eye(5), zeros(5, 2));
    [~, ~, x_test] = initial(sys_test, x0, t);
    u_test = -K_test * x_test';
    
    tau_max_test = max(max(abs(u_test)));
    is_stable = all(real(e_test) < 0);
    
    fprintf('Mô-men tối đa: %.3f N.m\n', tau_max_test);
    if is_stable
        fprintf('Trạng thái: ✓ Ổn định\n');
    else
        fprintf('Trạng thái: ✗ KHÔNG ổn định\n');
    end
    
    results{i, 1} = configs{i, 1};
    results{i, 2} = K_test;
    results{i, 3} = e_test;
    results{i, 4} = is_stable;
    results{i, 5} = tau_max_test;
    
    fprintf('\n');
end

%% ========== PHẦN 10: XUẤT FILE ==========

fprintf('========== XUẤT KẾT QUẢ ==========\n');

% Xuất file text
fileID = fopen('LQR_Coefficients_350W_FIXED.txt', 'w');

fprintf(fileID, '=========================================================================\n');
fprintf(fileID, 'LQR CONTROLLER COEFFICIENTS - ROBOT CÂN BẰNG (ĐÃ SỬA LỖI)\n');
fprintf(fileID, 'Ngày tạo: %s\n', datestr(now));
fprintf(fileID, '=========================================================================\n\n');

fprintf(fileID, '--- THÔNG SỐ ROBOT ---\n');
fprintf(fileID, 'Động cơ: BLDC 350W, 690 RPM, 4.8 N.m\n');
fprintf(fileID, 'Khối lượng: %.2f kg\n', m_total);
fprintf(fileID, 'Bán kính bánh xe: %.4f m\n', r);
fprintf(fileID, 'Khoảng cách bánh: %.3f m\n', wheel_base);
fprintf(fileID, 'Chiều cao trọng tâm: %.4f m\n', L);
fprintf(fileID, 'Moment quán tính: %.6f kg.m^2\n', I_body);
fprintf(fileID, 'Điện áp pin: %.1f V (10S)\n', V_battery);
fprintf(fileID, 'PWM max: %d\n\n', PWM_max);

fprintf(fileID, '--- TRẠNG THÁI (ĐÃ GIẢM TỪ 7 → 5) ---\n');
fprintf(fileID, '  x[0] = theta       (góc nghiêng)\n');
fprintf(fileID, '  x[1] = theta_dot   (vận tốc góc)\n');
fprintf(fileID, '  x[2] = x_pos       (vị trí)\n');
fprintf(fileID, '  x[3] = x_dot       (vận tốc tịnh tiến)\n');
fprintf(fileID, '  x[4] = x_integral  (tích phân vị trí)\n\n');

fprintf(fileID, '--- HỆ SỐ LQR KHUYẾN NGHỊ (Balanced) ---\n');
fprintf(fileID, '// Copy đoạn code này vào STM32:\n\n');

fprintf(fileID, '// Động cơ TRÁI\n');
fprintf(fileID, 'float K_L_theta       = %.8ff;\n', K_aug(1,1));
fprintf(fileID, 'float K_L_theta_dot   = %.8ff;\n', K_aug(1,2));
fprintf(fileID, 'float K_L_x           = %.8ff;\n', K_aug(1,3));
fprintf(fileID, 'float K_L_x_dot       = %.8ff;\n', K_aug(1,4));
fprintf(fileID, 'float K_L_x_integral  = %.8ff;\n\n', K_aug(1,5));

fprintf(fileID, '// Động cơ PHẢI\n');
fprintf(fileID, 'float K_R_theta       = %.8ff;\n', K_aug(2,1));
fprintf(fileID, 'float K_R_theta_dot   = %.8ff;\n', K_aug(2,2));
fprintf(fileID, 'float K_R_x           = %.8ff;\n', K_aug(2,3));
fprintf(fileID, 'float K_R_x_dot       = %.8ff;\n', K_aug(2,4));
fprintf(fileID, 'float K_R_x_integral  = %.8ff;\n\n', K_aug(2,5));

fprintf(fileID, '// Công thức tính điều khiển:\n');
fprintf(fileID, '// tau_L = -(K_L_theta*theta + K_L_theta_dot*theta_dot + K_L_x*x + K_L_x_dot*x_dot + K_L_x_integral*x_int)\n');
fprintf(fileID, '// tau_R = -(K_R_theta*theta + K_R_theta_dot*theta_dot + K_R_x*x + K_R_x_dot*x_dot + K_R_x_integral*x_int)\n\n');

fprintf(fileID, '// Thông số động cơ\n');
fprintf(fileID, '#define KT %.6ff          // N.m/A\n', Kt);
fprintf(fileID, '#define KE %.6ff          // V/(rad/s)\n', Ke);
fprintf(fileID, '#define R_MOTOR %.4ff     // Ohm\n', R_motor);
fprintf(fileID, '#define WHEEL_RADIUS %.4ff  // m\n', r);
fprintf(fileID, '#define V_BATTERY %.1ff     // V\n', V_battery);
fprintf(fileID, '#define PWM_MAX %d\n\n', PWM_max);

fprintf(fileID, '\n--- CÁC CẤU HÌNH THỬ NGHIỆM ---\n\n');

for i = 1:size(results, 1)
    fprintf(fileID, 'Cấu hình: %s\n', results{i,1});
    fprintf(fileID, 'Mô-men max: %.3f N.m\n', results{i,5});
    if results{i,4}
        fprintf(fileID, 'Trạng thái: Ổn định\n');
    else
        fprintf(fileID, 'Trạng thái: KHÔNG ổn định\n');
    end
    K_temp = results{i,2};
    fprintf(fileID, 'K_L = [%.8f, %.8f, %.8f, %.8f, %.8f]\n', K_temp(1,:));
    fprintf(fileID, 'K_R = [%.8f, %.8f, %.8f, %.8f, %.8f]\n\n', K_temp(2,:));
end

fprintf(fileID, '\n--- GHI CHÚ QUAN TRỌNG ---\n');
fprintf(fileID, '⚠ ĐÃ LOẠI BỎ điều khiển yaw (góc quay) vì không điều khiển được trực tiếp\n');
fprintf(fileID, '  - Để điều khiển yaw, cần thêm cơ chế vi sai: tau_yaw = (tau_R - tau_L) * wheel_base / 2\n');
fprintf(fileID, '  - Hoặc thiết kế controller riêng cho yaw dựa trên sai lệch tốc độ 2 bánh\n\n');

fclose(fileID);

% Lưu workspace
save('LQR_Workspace_350W_FIXED.mat');

fprintf('✓ Đã xuất file: LQR_Coefficients_350W_FIXED.txt\n');
fprintf('✓ Đã lưu workspace: LQR_Workspace_350W_FIXED.mat\n\n');

%% ========== HOÀN THÀNH ==========

fprintf('=========================================================================\n');
fprintf('                    HOÀN THÀNH THIẾT KẾ LQR!\n');
fprintf('=========================================================================\n\n');

fprintf('CÁC BƯỚC TIẾP THEO:\n');
fprintf('1. Kiểm tra file LQR_Coefficients_350W_FIXED.txt\n');
fprintf('2. Copy các hệ số vào code STM32\n');
fprintf('3. Triển khai công thức điều khiển:\n');
fprintf('   tau_L = -sum(K_L .* x_state)\n');
fprintf('   tau_R = -sum(K_R .* x_state)\n\n');

fprintf('4. Nếu cần điều chỉnh:\n');
fprintf('   - Robot giật: GIẢM Q, TĂNG R\n');
fprintf('   - Robot phản ứng chậm: TĂNG Q, GIẢM R\n');
fprintf('   - Robot không đứng vững: TĂNG Q(1,1) (theta)\n');
fprintf('   - Sai số vị trí: TĂNG Q(5,5) (x_integral)\n\n');

fprintf('5. Để điều khiển YAW (góc quay):\n');
fprintf('   - Thêm controller riêng: yaw_correction = Kp_yaw * yaw_error\n');
fprintf('   - Áp dụng vi sai: tau_L -= yaw_correction, tau_R += yaw_correction\n\n');

fprintf('Chúc bạn thành công!\n');
fprintf('=========================================================================\n');