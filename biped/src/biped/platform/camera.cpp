/**
 *  @file   camera.cpp
 *  @author Simon Yu
 *  @date   01/11/2023
 *  @brief  Camera class source.
 *
 *  This file implements the camera class.
 */

/*
 *  External headers.
 */
#include <esp_camera.h>
#include <esp_timer.h>
#include <functional>
#include <img_converters.h>

/*
 *  Project headers.
 */
#include "platform/camera.h"
#include "common/parameter.h"
#include "common/pin.h"
#include "platform/serial.h"

/*
 *  Biped namespace.
 */
namespace biped
{
Camera::Camera() : httpd_handle_(nullptr)
{
    /*
     *  Declare camera config struct.
     */
    camera_config_t camera_config;

    /*
     *  Populate camera config struct with parameters.
     */
    camera_config.ledc_channel = LEDC_CHANNEL_0;
    camera_config.ledc_timer = LEDC_TIMER_0;
    camera_config.pin_d0 = ESP32Pin::camera_d0;
    camera_config.pin_d1 = ESP32Pin::camera_d1;
    camera_config.pin_d2 = ESP32Pin::camera_d2;
    camera_config.pin_d3 = ESP32Pin::camera_d3;
    camera_config.pin_d4 = ESP32Pin::camera_d4;
    camera_config.pin_d5 = ESP32Pin::camera_d5;
    camera_config.pin_d6 = ESP32Pin::camera_d6;
    camera_config.pin_d7 = ESP32Pin::camera_d7;
    camera_config.pin_xclk = ESP32Pin::camera_xclk;
    camera_config.pin_pclk = ESP32Pin::camera_pclk;
    camera_config.pin_vsync = ESP32Pin::camera_vsync;
    camera_config.pin_href = ESP32Pin::camera_href;
    camera_config.pin_sscb_sda = ESP32Pin::camera_sscb_sda;
    camera_config.pin_sscb_scl = ESP32Pin::camera_sscb_scl;
    camera_config.pin_pwdn = ESP32Pin::camera_pwdn;
    camera_config.pin_reset = ESP32Pin::camera_reset;
    camera_config.xclk_freq_hz = CameraParameter::xclk_frequency;
    camera_config.frame_size = FRAMESIZE_HVGA;
    camera_config.pixel_format = PIXFORMAT_JPEG;
    camera_config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    camera_config.fb_location = CAMERA_FB_IN_DRAM;
    camera_config.jpeg_quality = CameraParameter::jpeg_quality;
    camera_config.fb_count = CameraParameter::frame_buffer_count;

    /*
     *  Initialize camera driver and validate the initialization.
     */
    if (esp_camera_init(&camera_config) != ESP_OK)
    {
        Serial(LogLevel::error) << "Failed to initialize camera.";
        return;
    }

    /*
     *  Get camera sensor pointer.
     */
    sensor_t *camera_sensor = esp_camera_sensor_get();

    /*
     *  Validate camera sensor pointer and set camera image
     *  orientation.
     */
    if (camera_sensor)
    {
        camera_sensor->set_hmirror(camera_sensor, 1);
        camera_sensor->set_vflip(camera_sensor, 1);
    }
    else
    {
        Serial(LogLevel::warn) << "Camera sensor missing.";
    }
}

void
Camera::stream()
{
    /*
     *  Declare and set HTTP web server config struct.
     */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16;

    /*
     *  Declare and set HTTP web server URI struct.
     */
    httpd_uri_t index_uri = {.uri = "/", .method = HTTP_GET, .handler = streamHandler, .user_ctx =
    NULL, };

    /*
     *  Start HTTP web server and register URI handler.
     */
    if (httpd_start(&httpd_handle_, &config) == ESP_OK)
    {
        httpd_register_uri_handler(httpd_handle_, &index_uri);
    }
}

esp_err_t
Camera::streamHandler(httpd_req_t* req)
{
    /*
     *  Declare variables.
     */
    camera_fb_t *fb = nullptr;
    struct timeval timestamp;
    esp_err_t res = ESP_OK;
    size_t jpg_buf_len = 0;
    uint8_t *jpg_buf = nullptr;
    char *part_buf[128];

    /*
     *  Set HTTP response type.
     */
    res = httpd_resp_set_type(req, stream_content_type_.c_str());

    /*
     *  Validate status.
     */
    if (res != ESP_OK)
    {
        return res;
    }

    /*
     *  Set HTTP response header.
     */
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "60");

    while (true)
    {
        /*
         *  Get camera frame buffer.
         */
        fb = esp_camera_fb_get();

        if (!fb)
        {
            res = ESP_FAIL;
        }
        else
        {
            /*
             *  Store frame buffer time stamps.
             */
            timestamp.tv_sec = fb->timestamp.tv_sec;
            timestamp.tv_usec = fb->timestamp.tv_usec;

            /*
             *  Convert frame buffer format to JPEG.
             */
            if (fb->format != PIXFORMAT_JPEG)
            {
                bool jpeg_converted = frame2jpg(fb, 80, &jpg_buf, &jpg_buf_len);
                esp_camera_fb_return(fb);
                fb = NULL;

                if (!jpeg_converted)
                {
                    res = ESP_FAIL;
                }
            }
            else
            {
                jpg_buf_len = fb->len;
                jpg_buf = fb->buf;
            }
        }

        /*
         *  Send stream boundary.
         */
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, stream_boundary_.c_str(),
                    strlen(stream_boundary_.c_str()));
        }

        /*
         *  Send stream part.
         */
        if (res == ESP_OK)
        {
            size_t hlen = snprintf((char*) part_buf, 128, stream_part_.c_str(), jpg_buf_len,
                    timestamp.tv_sec, timestamp.tv_usec);
            res = httpd_resp_send_chunk(req, (const char*) part_buf, hlen);
        }

        /*
         *  Send frame buffer.
         */
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, (const char*) jpg_buf, jpg_buf_len);
        }

        /*
         *  Clean up.
         */
        if (fb)
        {
            esp_camera_fb_return(fb);
            fb = NULL;
            jpg_buf = NULL;
        }
        else if (jpg_buf)
        {
            free(jpg_buf);
            jpg_buf = NULL;
        }

        /*
         *  Validate status.
         */
        if (res != ESP_OK)
        {
            break;
        }
    }

    /*
     *  Return status.
     */
    return res;
}

/*
 *  Initialize static class member variables.
 */
const std::string Camera::stream_boundary_ = std::string("\r\n--")
        + std::string(CameraParameter::stream_part_boundary) + std::string("\r\n");
const std::string Camera::stream_content_type_ = std::string("multipart/x-mixed-replace;boundary=")
        + std::string(CameraParameter::stream_part_boundary);
const std::string Camera::stream_part_ =
        "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";
}   // namespace biped
