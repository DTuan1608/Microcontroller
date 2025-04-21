#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"
#include "fd_forward.h"
#include "mtmn.h"
#include "image_util.h"
#include "dl_lib_matrix3dq.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_server.h"

// Định nghĩa macro để chọn phiên bản mạng nơ-ron
#define CONFIG_MTMN_LITE_QUANT

// Định nghĩa cấu hình camera
#define CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Cấu hình Wi-Fi
#define WIFI_SSID "DTuan"      // Thay bằng SSID của bạn
#define WIFI_PASS "160820048"  // Thay bằng mật khẩu Wi-Fi của bạn

static const char *TAG = "face_detect_stream";

// Hàm xử lý HTTP GET cho MJPEG stream
static esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    char *part_buf[64];
    static const char *BOUNDARY = "\r\n--frame_boundary\r\n";
    static const char *CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame_boundary";
    static const char *JPEG_HEADER = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

    // Thiết lập Content-Type cho MJPEG stream
    res = httpd_resp_set_type(req, CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    while (true) {
        // Lấy frame từ camera
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        // Gửi boundary
        res = httpd_resp_send_chunk(req, BOUNDARY, strlen(BOUNDARY));
        if (res != ESP_OK) {
            esp_camera_fb_return(fb);
            break;
        }

        // Gửi header của frame JPEG
        snprintf((char *)part_buf, sizeof(part_buf), JPEG_HEADER, fb->len);
        res = httpd_resp_send_chunk(req, (const char *)part_buf, strlen((const char *)part_buf));
        if (res != ESP_OK) {
            esp_camera_fb_return(fb);
            break;
        }

        // Gửi dữ liệu JPEG
        res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        if (res != ESP_OK) {
            esp_camera_fb_return(fb);
            break;
        }

        esp_camera_fb_return(fb);
        vTaskDelay(100 / portTICK_PERIOD_MS); // Điều chỉnh tốc độ stream (100ms mỗi frame)
    }

    // Kết thúc stream
    httpd_resp_send_chunk(req, NULL, 0);
    return res;
}

// Hàm khởi tạo HTTP server
static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;

    // Đăng ký URI handler cho stream
    httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    ESP_LOGI(TAG, "Starting web server on port: %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &stream_uri);
    }

    return server;
}

// Hàm xử lý sự kiện Wi-Fi
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Retrying Wi-Fi connection...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

// Hàm khởi tạo Wi-Fi
static void wifi_init(void) {
    // Khởi tạo NVS (Non-Volatile Storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Khởi tạo TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Cấu hình Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Đăng ký handler sự kiện Wi-Fi
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    // Cấu hình Wi-Fi STA (Station mode)
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi initialized, connecting to SSID: %s", WIFI_SSID);
}

void app_main(void) {
    // Khởi tạo giao tiếp Serial
    printf("Starting Face Detect Stream\n");

    // Khởi tạo Wi-Fi
    wifi_init();

    // Cấu hình camera
    camera_config_t config = {
        .pin_pwdn = PWDN_GPIO_NUM,
        .pin_reset = RESET_GPIO_NUM,
        .pin_xclk = XCLK_GPIO_NUM,
        .pin_sccb_sda = SIOD_GPIO_NUM,
        .pin_sccb_scl = SIOC_GPIO_NUM,
        .pin_d7 = Y9_GPIO_NUM,
        .pin_d6 = Y8_GPIO_NUM,
        .pin_d5 = Y7_GPIO_NUM,
        .pin_d4 = Y6_GPIO_NUM,
        .pin_d3 = Y5_GPIO_NUM,
        .pin_d2 = Y4_GPIO_NUM,
        .pin_d1 = Y3_GPIO_NUM,
        .pin_d0 = Y2_GPIO_NUM,
        .pin_vsync = VSYNC_GPIO_NUM,
        .pin_href = HREF_GPIO_NUM,
        .pin_pclk = PCLK_GPIO_NUM,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_QVGA,
        .jpeg_quality = 12,
        .fb_count = 1,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .fb_location = CAMERA_FB_IN_PSRAM,
    };

    // Khởi tạo camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        printf("Camera init failed with error 0x%x\n", err);
        return;
    }

    printf("Camera initialized successfully\n");

    // Khởi động HTTP server
    start_webserver();

    // Vòng lặp chính (phát hiện khuôn mặt)
    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            printf("Camera capture failed\n");
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
        if (!image_matrix) {
            printf("Matrix allocation failed\n");
            esp_camera_fb_return(fb);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        if (!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)) {
            printf("fmt2rgb888 failed\n");
            dl_matrix3du_free(image_matrix);
            esp_camera_fb_return(fb);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        }

        mtmn_config_t mtmn_config = {0};
        mtmn_config.type = FAST;
        mtmn_config.min_face = 40;
        mtmn_config.pyramid_times = 1;
        mtmn_config.p_threshold.score = 0.6;
        mtmn_config.p_threshold.nms = 0.7;
        mtmn_config.p_threshold.candidate_number = 20;

        box_array_t *boxes = face_detect(image_matrix, &mtmn_config);
        if (boxes != NULL) {
            printf("Faces detected: %d\n", boxes->len);
            dl_lib_free(boxes->box);
            dl_lib_free(boxes);
        } else {
            printf("No faces detected\n");
        }

        dl_matrix3du_free(image_matrix);
        esp_camera_fb_return(fb);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}