// http://0.0.0.0/
#include <stdio.h>
#include <string.h>
#include <esp_camera.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_netif.h>
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>
#include "qrcode.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "driver/uart.h"
#include <esp_timer.h>

// Add these global variables before the stream_handler function
static uint8_t *jpg_buf = NULL;
static size_t jpg_buf_len = 0;
#define UART_NUM UART_NUM_1
#define UART_TX_PIN 1 // Điều chỉnh theo pins thực tế của bạn
#define UART_RX_PIN 3
#define UART_BAUD_RATE 115200
#define BUF_SIZE 1024
#define UDP_PORT 1234

// Định nghĩa các chân GPIO cho ESP32-CAM
#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27
#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

// Constants for QR Code and Wi-Fi provisioning
#define PROV_QR_VERSION "v1"
#define PROV_TRANSPORT_BLE "ble"
#define QRCODE_BASE_URL "https://espressif.github.io/esp-jumpstart/qrcode.html"
#define EXAMPLE_PROV_SEC2_USERNAME "wifiprov"
#define EXAMPLE_PROV_SEC2_PWD "abcd1234"
// Biến cờ cho biết đã kết nối Wi-Fi
static bool wifi_connected = false;
static const char *TAG = "app";
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;
// Hardcoded salt and verifier for security
static const char sec2_salt[] = {
    0x03, 0x6e, 0xe0, 0xc7, 0xbc, 0xb9, 0xed, 0xa8, 0x4c, 0x9e, 0xac, 0x97, 0xd9, 0x3d, 0xec, 0xf4};

static const char sec2_verifier[] = {
    0x7c, 0x7c, 0x85, 0x47, 0x65, 0x08, 0x94, 0x6d, 0xd6, 0x36, 0xaf, 0x37, 0xd7, 0xe8, 0x91, 0x43,
    0x78, 0xcf, 0xfd, 0x61, 0x6c, 0x59, 0xd2, 0xf8, 0x39, 0x08, 0x12, 0x72, 0x38, 0xde, 0x9e, 0x24,
    0xa4, 0x70, 0x26, 0x1c, 0xdf, 0xa9, 0x03, 0xc2, 0xb2, 0x70, 0xe7, 0xb1, 0x32, 0x24, 0xda, 0x11,
    0x1d, 0x97, 0x18, 0xdc, 0x60, 0x72, 0x08, 0xcc, 0x9a, 0xc9, 0x0c, 0x48, 0x27, 0xe2, 0xae, 0x89,
    0xaa, 0x16, 0x25, 0xb8, 0x04, 0xd2, 0x1a, 0x9b, 0x3a, 0x8f, 0x37, 0xf6, 0xe4, 0x3a, 0x71, 0x2e,
    0xe1, 0x27, 0x86, 0x6e, 0xad, 0xce, 0x28, 0xff, 0x54, 0x46, 0x60, 0x1f, 0xb9, 0x96, 0x87, 0xdc,
    0x57, 0x40, 0xa7, 0xd4, 0x6c, 0xc9, 0x77, 0x54, 0xdc, 0x16, 0x82, 0xf0, 0xed, 0x35, 0x6a, 0xc4,
    0x70, 0xad, 0x3d, 0x90, 0xb5, 0x81, 0x94, 0x70, 0xd7, 0xbc, 0x65, 0xb2, 0xd5, 0x18, 0xe0, 0x2e,
    0xc3, 0xa5, 0xf9, 0x68, 0xdd, 0x64, 0x7b, 0xb8, 0xb7, 0x3c, 0x9c, 0xfc, 0x00, 0xd8, 0x71, 0x7e,
    0xb7, 0x9a, 0x7c, 0xb1, 0xb7, 0xc2, 0xc3, 0x18, 0x34, 0x29, 0x32, 0x43, 0x3e, 0x00, 0x99, 0xe9,
    0x82, 0x94, 0xe3, 0xd8, 0x2a, 0xb0, 0x96, 0x29, 0xb7, 0xdf, 0x0e, 0x5f, 0x08, 0x33, 0x40, 0x76,
    0x52, 0x91, 0x32, 0x00, 0x9f, 0x97, 0x2c, 0x89, 0x6c, 0x39, 0x1e, 0xc8, 0x28, 0x05, 0x44, 0x17,
    0x3f, 0x68, 0x02, 0x8a, 0x9f, 0x44, 0x61, 0xd1, 0xf5, 0xa1, 0x7e, 0x5a, 0x70, 0xd2, 0xc7, 0x23,
    0x81, 0xcb, 0x38, 0x68, 0xe4, 0x2c, 0x20, 0xbc, 0x40, 0x57, 0x76, 0x17, 0xbd, 0x08, 0xb8, 0x96,
    0xbc, 0x26, 0xeb, 0x32, 0x46, 0x69, 0x35, 0x05, 0x8c, 0x15, 0x70, 0xd9, 0x1b, 0xe9, 0xbe, 0xcc,
    0xa9, 0x38, 0xa6, 0x67, 0xf0, 0xad, 0x50, 0x13, 0x19, 0x72, 0x64, 0xbf, 0x52, 0xc2, 0x34, 0xe2,
    0x1b, 0x11, 0x79, 0x74, 0x72, 0xbd, 0x34, 0x5b, 0xb1, 0xe2, 0xfd, 0x66, 0x73, 0xfe, 0x71, 0x64,
    0x74, 0xd0, 0x4e, 0xbc, 0x51, 0x24, 0x19, 0x40, 0x87, 0x0e, 0x92, 0x40, 0xe6, 0x21, 0xe7, 0x2d,
    0x4e, 0x37, 0x76, 0x2f, 0x2e, 0xe2, 0x68, 0xc7, 0x89, 0xe8, 0x32, 0x13, 0x42, 0x06, 0x84, 0x84,
    0x53, 0x4a, 0xb3, 0x0c, 0x1b, 0x4c, 0x8d, 0x1c, 0x51, 0x97, 0x19, 0xab, 0xae, 0x77, 0xff, 0xdb,
    0xec, 0xf0, 0x10, 0x95, 0x34, 0x33, 0x6b, 0xcb, 0x3e, 0x84, 0x0f, 0xb9, 0xd8, 0x5f, 0xb8, 0xa0,
    0xb8, 0x55, 0x53, 0x3e, 0x70, 0xf7, 0x18, 0xf5, 0xce, 0x7b, 0x4e, 0xbf, 0x27, 0xce, 0xce, 0xa8,
    0xb3, 0xbe, 0x40, 0xc5, 0xc5, 0x32, 0x29, 0x3e, 0x71, 0x64, 0x9e, 0xde, 0x8c, 0xf6, 0x75, 0xa1,
    0xe6, 0xf6, 0x53, 0xc8, 0x31, 0xa8, 0x78, 0xde, 0x50, 0x40, 0xf7, 0x62, 0xde, 0x36, 0xb2, 0xba};

// Function to get the device's service name based on MAC address
static void get_device_service_name(char *service_name, size_t max)
{
    uint8_t eth_mac[6];
    const char *ssid_prefix = "PROV_";
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
    snprintf(service_name, max, "%s%02X%02X%02X",
             ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
}

// Function to generate and display the QR code for Wi-Fi provisioning
static void wifi_prov_print_qr(const char *name, const char *username,
                               const char *pop, const char *transport)
{
    if (!name || !transport)
    {
        ESP_LOGW(TAG, "Cannot generate QR code payload. Data missing.");
        return;
    }

    char payload[150] = {0};
    if (pop)
    {
        snprintf(payload, sizeof(payload),
                 "{\"ver\":\"%s\",\"name\":\"%s\",\"pop\":\"%s\",\"transport\":\"%s\"}",
                 PROV_QR_VERSION, name, pop, transport);
    }
    else
    {
        snprintf(payload, sizeof(payload),
                 "{\"ver\":\"%s\",\"name\":\"%s\",\"transport\":\"%s\"}",
                 PROV_QR_VERSION, name, transport);
    }

    ESP_LOGI(TAG, "Scan this QR code from the ESP Provisioning app:");
    esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
    esp_qrcode_generate(&cfg, payload);
    ESP_LOGI(TAG, "Or enter this URL in browser:");
    ESP_LOGI(TAG, "%s?data=%s", QRCODE_BASE_URL, payload);
    ESP_LOGI(TAG, "Name: %s", name);
    if (pop)
    {
        ESP_LOGI(TAG, "POP: %s", pop);
    }
}

// Function to retrieve the security salt
static esp_err_t example_get_sec2_salt(const char **salt, uint16_t *salt_len)
{
    ESP_LOGI(TAG, "Development mode: using hard coded salt");
    *salt = sec2_salt;
    *salt_len = sizeof(sec2_salt);
    return ESP_OK;
}

// Function to retrieve the security verifier
static esp_err_t example_get_sec2_verifier(const char **verifier, uint16_t *verifier_len)
{
    ESP_LOGI(TAG, "Development mode: using hard coded verifier");
    *verifier = sec2_verifier;
    *verifier_len = sizeof(sec2_verifier);
    return ESP_OK;
}
esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    char *part_buf[64];
    static int64_t last_frame = 0;

    // Set response headers
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while (true)
    {
        // Capture frame with timeout
        fb = esp_camera_fb_get();
        if (!fb)
        {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        if (fb->format != PIXFORMAT_JPEG)
        {
            bool jpeg_converted = frame2jpg(fb, 80, &jpg_buf, &jpg_buf_len);
            if (!jpeg_converted)
            {
                ESP_LOGE(TAG, "JPEG compression failed");
                esp_camera_fb_return(fb);
                res = ESP_FAIL;
                break;
            }
        }
        else
        {
            jpg_buf_len = fb->len;
            jpg_buf = fb->buf;
        }

        // Construct and send frame
        size_t hlen = snprintf((char *)part_buf, 64,
                               "Content-Type: image/jpeg\r\n"
                               "Content-Length: %u\r\n"
                               "\r\n",
                               jpg_buf_len);

        if ((res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen)) != ESP_OK)
        {
            ESP_LOGE(TAG, "Send header failed: %d", res);
            break;
        }

        if ((res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_buf_len)) != ESP_OK)
        {
            ESP_LOGE(TAG, "Send image failed: %d", res);
            break;
        }

        if ((res = httpd_resp_send_chunk(req, "\r\n--frame\r\n", 12)) != ESP_OK)
        {
            ESP_LOGE(TAG, "Send boundary failed: %d", res);
            break;
        }

        // Clean up
        if (fb->format != PIXFORMAT_JPEG)
        {
            free(jpg_buf);
        }
        esp_camera_fb_return(fb);

        // Add delay between frames
        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000; // Convert to ms
        if (frame_time < 35)
        { // Limit to ~30fps
            vTaskDelay((35 - frame_time) / portTICK_PERIOD_MS);
        }
    }

    // Clean up on error
    if (fb)
    {
        esp_camera_fb_return(fb);
    }

    return res;
}
// ---------- HTTP Handlers ----------

esp_err_t favicon_handler(httpd_req_t *req)
{
    static const uint8_t dummy_favicon[] = {0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x10};
    httpd_resp_set_type(req, "image/x-icon");
    return httpd_resp_send(req, (const char *)dummy_favicon, sizeof(dummy_favicon));
}

// ---------- Web Server ----------
void start_web_server()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_uri_t capture_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = stream_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &capture_uri);

        httpd_uri_t favicon_uri = {
            .uri = "/favicon.ico",
            .method = HTTP_GET,
            .handler = favicon_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &favicon_uri);

        ESP_LOGI(TAG, "HTTP server started");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to start HTTP server");
    }
}
// Hàm xử lý sự kiện Wi-Fi
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        if (event_id == WIFI_EVENT_STA_START)
        {
            esp_wifi_connect();
        }
        else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
        {
            ESP_LOGI(TAG, "Disconnected. Reconnecting...");
            esp_wifi_connect();
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        wifi_connected = true;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Wi-Fi connected and IP received.");
    }
}
void init_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}
void udp_server_task(void *pvParameters)
{
    char rx_buffer[3]; // 2 bytes data + null terminator
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        goto CLEAN_UP;
    }

    ESP_LOGI(TAG, "UDP server listening on port %d", UDP_PORT);

    while (1)
    {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, 2, 0, (struct sockaddr *)&source_addr, &socklen);

        if (len == 2)
        {                            // Nhận đúng 2 bytes
            int8_t x = rx_buffer[0]; // Byte thứ nhất là giá trị X
            int8_t y = rx_buffer[1]; // Byte thứ hai là giá trị Y

            // Kiểm tra giá trị hợp lệ (-100 đến 100)
            if (x >= -100 && x <= 100 && y >= -100 && y <= 100)
            {
                char uart_buffer[20];
                int uart_len = snprintf(uart_buffer, sizeof(uart_buffer), "X:%d,Y:%d\n", x, y);
                uart_write_bytes(UART_NUM, uart_buffer, uart_len);
                ESP_LOGI(TAG, "Sent to UART: %s", uart_buffer);
            }
        }
    }

CLEAN_UP:
    close(sock);
    vTaskDelete(NULL);
}
// Main application entry point
void app_main(void)
{
    // Khởi tạo NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Tạo event group
    wifi_event_group = xEventGroupCreate();

    // Khởi tạo mạng và vòng lặp sự kiện
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Đăng ký sự kiện Wi-Fi
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    // Cấu hình Wi-Fi
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Khởi tạo provisioning
    wifi_prov_mgr_config_t prov_config = {
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM};
    ESP_ERROR_CHECK(wifi_prov_mgr_init(prov_config));
    wifi_prov_mgr_reset_provisioning();

    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (!provisioned)
    {
        ESP_LOGI(TAG, "Starting provisioning");
        char service_name[12];
        get_device_service_name(service_name, sizeof(service_name));

        wifi_prov_security_t security = WIFI_PROV_SECURITY_2;
        const char *username = EXAMPLE_PROV_SEC2_USERNAME;
        const char *pop = EXAMPLE_PROV_SEC2_PWD;

        wifi_prov_security2_params_t sec2_params = {};
        ESP_ERROR_CHECK(example_get_sec2_salt(&sec2_params.salt, &sec2_params.salt_len));
        ESP_ERROR_CHECK(example_get_sec2_verifier(&sec2_params.verifier, &sec2_params.verifier_len));

        uint8_t custom_service_uuid[] = {
            0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf,
            0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02};

        wifi_prov_scheme_ble_set_service_uuid(custom_service_uuid);
        wifi_prov_mgr_disable_auto_stop(1000);

        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security,
                                                         (const void *)&sec2_params,
                                                         service_name,
                                                         NULL));
        wifi_prov_print_qr(service_name, username, pop, PROV_TRANSPORT_BLE);
    }
    else
    {
        ESP_LOGI(TAG, "Already provisioned. Starting Wi-Fi STA");
        wifi_prov_mgr_deinit();
    }

    // Chờ kết nối Wi-Fi (blocking chờ đến khi có IP)
    ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);

    // ====== Wi-Fi đã kết nối thành công ======

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);

    // Khởi tạo UART
    init_uart();

    // Tạo task UDP
    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);

    // Cấu hình camera
    camera_config_t camera_config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_VGA,
        .pixel_format = PIXFORMAT_JPEG,
        .jpeg_quality = 10,
        .fb_count = 2,
    };

    ret = esp_camera_init(&camera_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(ret));
        return;
    }

    // Bắt đầu server web
    start_web_server();

    // Vòng lặp chụp ảnh
    while (1)
    {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb)
        {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        ESP_LOGI(TAG, "Captured image size: %d bytes", fb->len);

        // TODO: xử lý ảnh tại đây (ví dụ: gửi qua WebSocket hoặc HTTP stream)

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}