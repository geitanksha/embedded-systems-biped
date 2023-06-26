/**
 *  @file   camera.h
 *  @author Simon Yu
 *  @date   01/11/2023
 *  @brief  Camera class header.
 *
 *  This file defines the camera class.
 */

/*
 *  Include guard.
 */
#ifndef PLATFORM_CAMERA_H_
#define PLATFORM_CAMERA_H_

/*
 *  External headers.
 */
#include <esp_http_server.h>
#include <string>

/*
 *  Biped namespace.
 */
namespace biped
{
/**
 *  @brief  Camera class.
 *
 *  This class provides functions for streaming
 *  images from the camera.
 */
class Camera
{
public:

    /**
     *  @brief  Camera class constructor.
     *
     *  This constructor initializes all class member variables.
     *  Additionally, the constructor configures the camera.
     */
    Camera();

    /**
     *  @brief  Stream the camera images.
     *
     *  This function creates an HTTP web server and continuously
     *  stream the camera images via the web server.
     */
    void
    stream();

private:

    /**
     *  @param  req HTTP request struct.
     *  @return ESP error type.
     *  @brief  Handle HTTP requests and camera image streaming.
     *
     *  This function handles HTTP requests and the streaming
     *  of the camera images, and returns an ESP error status.
     */
    static esp_err_t
    streamHandler(httpd_req_t* req);

    httpd_handle_t httpd_handle_;   //!< HTTP web server handle.
    static const std::string stream_boundary_;  //!< Stream boundary string.
    static const std::string stream_content_type_;  //!< Stream content type string.
    static const std::string stream_part_;  //!< Stream part string.
};
}   // namespace biped

#endif  // PLATFORM_CAMERA_H_
