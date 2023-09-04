/*
Copyright (c) 2013, Broadcom Europe Ltd
Copyright (c) 2013, James Hughes
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

extern "C" {
#include "interface/mmal/mmal.h"
#include "RaspiCamControl.h"
}

#include "mmal_cxx_helper.h"

typedef std::function<void(const uint8_t*, const uint8_t*)> buffer_callback_t;

/** Structure containing all state information for the current run
 */
struct RASPIVID_STATE {
  RASPIVID_STATE()
    : camera_component(nullptr)
    , splitter_component(nullptr)
    , image_encoder_component(nullptr)
    , video_encoder_component(nullptr)
    , splitter_connection(nullptr)
    , image_encoder_connection(nullptr)
    , video_encoder_connection(nullptr)
    , splitter_pool(nullptr, mmal::default_delete_pool)
    , image_encoder_pool(nullptr, mmal::default_delete_pool)
    , video_encoder_pool(nullptr, mmal::default_delete_pool){}

  bool isInit;
  int width;      /// Requested width of image
  int height;     /// requested height of image
  int framerate;  /// Requested frame rate (fps)
  int quality;
  bool enable_raw_pub;  // Enable Raw publishing
  bool enable_imv_pub;  // Enable publishing of inline motion vectors

  int camera_id = 0;

  RASPICAM_CAMERA_PARAMETERS camera_parameters;  /// Camera setup parameters

  mmal::component_ptr camera_component;
  mmal::component_ptr splitter_component;
  mmal::component_ptr image_encoder_component;
  mmal::component_ptr video_encoder_component;

  mmal::connection_ptr splitter_connection;       /// Pointer to camera => splitter
  mmal::connection_ptr image_encoder_connection;  /// Pointer to splitter => encoder
  mmal::connection_ptr video_encoder_connection;  /// Pointer to camera => encoder

  mmal::pool_ptr splitter_pool;       // Pointer buffer pool used by splitter (raw) output
  mmal::pool_ptr image_encoder_pool;  // Pointer buffer pool used by encoder (jpg) output
  mmal::pool_ptr video_encoder_pool;  // Pointer buffer pool used by encoder (h264) output
};

/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state state structure to assign defaults to
 */
void configure_parameters(RASPIVID_STATE& state);

/**
 *  buffer header callback function for image encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void image_encoder_buffer_callback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer);

/**
 *  buffer header callback function for video encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void video_encoder_buffer_callback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer);

static void splitter_buffer_callback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer);

/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return 0 if failed, pointer to component if successful
 *
 */
static MMAL_COMPONENT_T* create_camera_component(RASPIVID_STATE& state);

/**
 * Create the image encoder component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_image_encoder_component(RASPIVID_STATE& state);

/**
 * Create the video encoder component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_video_encoder_component(RASPIVID_STATE& state);

/**
 * Create the splitter component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_splitter_component(RASPIVID_STATE& state);

/**
 * Connect two specific ports together
 *
 * @param output_port Pointer the output port
 * @param input_port Pointer the input port
 * @param Pointer to a mmal connection pointer, reassigned if function
 * successful
 * @return Returns a MMAL_STATUS_T giving result of operation
 *
 */
static MMAL_STATUS_T connect_ports(MMAL_PORT_T* output_port, MMAL_PORT_T* input_port,
                                   mmal::connection_ptr& connection);

/**
 * init_cam

 */
int init_cam(RASPIVID_STATE& state, buffer_callback_t cb_raw = nullptr, buffer_callback_t cb_compressed = nullptr, buffer_callback_t cb_motion = nullptr);

int start_capture(RASPIVID_STATE& state);

int close_cam(RASPIVID_STATE& state);
