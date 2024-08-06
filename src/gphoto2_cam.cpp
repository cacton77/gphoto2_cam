// Copyright 2014 Robert Bosch, LLC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Robert Bosch, LLC nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#define CLEAR(x) memset(&(x), 0, sizeof(x))

extern "C" {
#include <linux/videodev2.h>  // Defines V4L2 format constants
#include <malloc.h>  // for memalign
#include <sys/mman.h>  // for mmap
#include <sys/stat.h>  // for stat
#include <unistd.h>  // for getpagesize()
#include <fcntl.h>  // for O_* constants and open()
}

#include <chrono>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"

#include "gphoto2_cam/gphoto2_cam.hpp"
#include "gphoto2_cam/conversions.hpp"
#include "gphoto2_cam/utils.hpp"


#include <gphoto2/gphoto2-camera.h>

namespace gphoto2_cam
{

using utils::io_method_t;

struct ConfigOption {
    std::string name;
    std::string value;
};

void getWidgetValue(CameraWidget *widget, config_map_t &config_map) {

    const char *name;
    gp_widget_get_name(widget, &name);

    // if (name == nullptr) {
    //   name = "";
    // }

    CameraWidgetType type;
    gp_widget_get_type(widget, &type);

    // Handle different types of widgets
    // (char*) for GP_WIDGET_MENU, GP_WIDGET_TEXT, GP_WIDGET_RADIO, 
    // (float) for GP_WIDGET_RANGE, 
    // (int) for GP_WIDGET_DATE, GP_WIDGET_TOGGLE, and 
    // (CameraWidgetCallback) for GP_WIDGET_BUTTON. 

    int id;
    int readonly;
    const char* label;

    gp_widget_get_id(widget, &id);
    gp_widget_get_readonly(widget, &readonly);
    gp_widget_get_label(widget, &label);

    // if (label == nullptr) {
    //   label = "";
    // }

    switch (type) {
        case GP_WIDGET_WINDOW: break;
        case GP_WIDGET_SECTION: break;
        case GP_WIDGET_TEXT: {
            char *value;
            if (gp_widget_get_value(widget, &value) == GP_OK) {


                config_map[name] = char_config_t{
                  name, label, id, readonly, value, {}
                };
                // std::cout << name << ": " << std::endl;
                // std::cout << "\t" << "Type: " << "TEXT" << std::endl;
                // std::cout << "\t" << "ID: " << id << std::endl;
                // std::cout << "\t" << "Read only: " << readonly << std::endl;
                // std::cout << "\t" << "Label: " << label << std::endl;
                // std::cout << "\t" << "Value: " << value << std::endl;
            }
            break;
        }
        case GP_WIDGET_RANGE: {
            float value;
            if (gp_widget_get_value(widget, &value) == GP_OK) {
                float min;
                float max;
                float increment;

                gp_widget_get_range(widget, &min, &max, &increment);

                config_map[name] = float_config_t{
                  name, label, id, readonly, value, min, max, increment
                };
                // std::cout << name << ": " << std::endl;
                // std::cout << "\t" << "Type: " << "RANGE" << std::endl;
                // std::cout << "\t" << "ID: " << id << std::endl;
                // std::cout << "\t" << "Read only: " << readonly << std::endl;
                // std::cout << "\t" << "Label: " << label << std::endl;
                // std::cout << "\t" << "Value: " << value << std::endl;
                // std::cout << "\t" << "Range: " << std::endl;
                // std::cout << "\t\t" << "Min: " << &min << std::endl;
                // std::cout << "\t\t" << "Max: " << &max << std::endl;
                // std::cout << "\t\t" << "Inc: " << &increment << std::endl;
            }
            break;
        }
        case GP_WIDGET_TOGGLE: {
            int value;
            if (gp_widget_get_value(widget, &value) == GP_OK) {

                config_map[name] = int_config_t{
                  name, label, id, readonly, value
                };
                // std::cout << name << ": " << std::endl;
                // std::cout << "\t" << "Type: " << "TOGGLE" << std::endl;
                // std::cout << "\t" << "ID: " << id << std::endl;
                // std::cout << "\t" << "Read only: " << readonly << std::endl;
                // std::cout << "\t" << "Label: " << label << std::endl;
                // std::cout << "\t" << "Value: " << (value ? "On" : "Off") << std::endl;
                // options.push_back({name, std::to_string(value)});
            }
            break;
        }
        case GP_WIDGET_MENU:
        case GP_WIDGET_RADIO: {
            char *value;
            if (gp_widget_get_value(widget, &value) == GP_OK) {
              if (value == nullptr) {
                std::cout << "Choice is null" << std::endl;
              }
              else {

                std::vector<std::string>  choices;

                // New code to print choices
                int choiceCount = gp_widget_count_choices(widget);
                for (int i = 0; i < choiceCount; ++i) {
                    const char* choice;
                    if (choice == nullptr) {
                      continue;
                    }
                    if (gp_widget_get_choice(widget, i, &choice) == GP_OK) {
                        choices.push_back(std::string(choice));
                    }
                }
                config_map[name] = char_config_t{
                  name, 
                  label, 
                  id, 
                  readonly, 
                  std::string(value), 
                  choices 
                };
              }
                // options.push_back({name, value});
            }
            break;
        }
        case GP_WIDGET_BUTTON: {
            int value;
            if (gp_widget_get_value(widget, &value) == GP_OK) {

                // std::cout << name << ": " << std::endl;
                // std::cout << "\t" << "Type: " << "BUTTON" << std::endl;
                // std::cout << "\t" << "ID: " << id << std::endl;
                // std::cout << "\t" << "Read only: " << readonly << std::endl;
                // std::cout << "\t" << "Label: " << label << std::endl;
                // std::cout << "\t" << "Value: " << (value ? "On" : "Off") << std::endl;
            }
            break;
        }
        case GP_WIDGET_DATE:
        // Add cases for other widget types as needed
        default:
            // Handle or log unsupported widget types
            break;
    }

    // Recurse into children if any.
    int children = gp_widget_count_children(widget);
    for (int i = 0; i < children; ++i) {
        CameraWidget *child;
        if (gp_widget_get_child(widget, i, &child) == GP_OK) {
            getWidgetValue(child, config_map);
        }
    }
}

std::mutex gPhoto2Cam::gp_mutex;


gPhoto2Cam::gPhoto2Cam():
  // m_number_of_buffers(4), m_buffers(new gphoto2_cam::utils::buffer[m_number_of_buffers]), m_image(),
  m_camera(), m_context(), m_file(),
  m_image(),
  // m_avframe(NULL), m_avcodec(NULL), m_avoptions(NULL),
  // m_avcodec_context(NULL), 
  m_is_capturing(false), m_framerate(0)
  // m_epoch_time_shift_us(gphoto2_cam::utils::get_epoch_time_shift_us()), m_supported_formats()
{}

gPhoto2Cam::~gPhoto2Cam()
{
  shutdown();
}


/// @brief Fill destination image with source image. If required, convert a given
/// V4L2 Image into another type. Look up possible V4L2 pixe formats in the
/// `linux/videodev2.h` header file.
/// @param src a pointer to a V4L2 source image
/// @param dest a pointer to where the source image should be copied (if required)
/// @param bytes_used number of bytes used by the src buffer


void gPhoto2Cam::stop_capturing()
{
  
}

void gPhoto2Cam::start_capturing()
{
  if (m_is_capturing) {return;}

  m_is_capturing = true;
}

void gPhoto2Cam::uninit_device()
{
  // m_buffers.reset();
}


void gPhoto2Cam::init_device()
{
  // struct v4l2_capability cap; // struct v4l2_cropcap cropcap; // struct v4l2_crop crop; // if (-1 == gphoto2_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QUERYCAP), &cap)) { //   if (EINVAL == errno) { //     throw std::invalid_argument("Device is not a V4L2 device"); //   } else { //     throw std::invalid_argument("Unable to query device capabilities"); //   } // } // if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) { //   throw std::invalid_argument("Device is not a video capture device"); // } // switch (m_io) { //   case io_method_t::IO_METHOD_READ: //     if (!(cap.capabilities & V4L2_CAP_READWRITE)) { //       throw std::invalid_argument("Device does not support read i/o"); //     } //     break; //   case io_method_t::IO_METHOD_MMAP: //   case io_method_t::IO_METHOD_USERPTR: //     if (!(cap.capabilities & V4L2_CAP_STREAMING)) { //       throw std::invalid_argument("Device does not support streaming i/o"); //     } //     break; //   case io_method_t::IO_METHOD_UNKNOWN: //     throw std::invalid_argument("Unsupported IO method specified"); // } /* Select video input, video standard and tune here. */ // CLEAR(cropcap); // cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // if (0 == gphoto2_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_CROPCAP), &cropcap)) { //   crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; //   crop.c = cropcap.defrect; /* reset to default */ //   if (-1 == gphoto2_cam::utils::xioctl(m_fd, VIDIOC_S_CROP, &crop)) { //     switch (errno) { //       case EINVAL: //         /* Cropping not supported. */ //         break; //       default: //         /* Errors ignored. */ //         break; //     } //   } // } else { //   /* Errors ignored. */ // } // m_image.v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // m_image.v4l2_fmt.fmt.pix.width = m_image.width; // m_image.v4l2_fmt.fmt.pix.height = m_image.height; // m_image.v4l2_fmt.fmt.pix.pixelformat = m_image.pixel_format->v4l2(); // m_image.v4l2_fmt.fmt.pix.field = V4L2_FIELD_ANY; // // Set v4l2 capture format // // Note VIDIOC_S_FMT may change width and height // if (-1 == gphoto2_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_S_FMT), &m_image.v4l2_fmt)) { //   throw strerror(errno); // } // struct v4l2_streamparm stream_params; // memset(&stream_params, 0, sizeof(stream_params)); // stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // if (gphoto2_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_G_PARM), &stream_params) < 0) { //   throw strerror(errno); // } // if (!stream_params.parm.capture.capability && V4L2_CAP_TIMEPERFRAME) { //   throw "V4L2_CAP_TIMEPERFRAME not supported"; // } // // TODO(lucasw) need to get list of valid numerator/denominator pairs // // and match closest to what user put in.  // stream_params.parm.capture.timeperframe.numerator = 1; // stream_params.parm.capture.timeperframe.denominator = m_framerate; // if (gphoto2_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_S_PARM), &stream_params) < 0) { //   throw std::invalid_argument("Couldn't set camera framerate"); // } // switch (m_io) { //   case io_method_t::IO_METHOD_READ: //     init_read(); //     break; //   case io_method_t::IO_METHOD_MMAP: //     init_mmap(); //     break; //   case io_method_t::IO_METHOD_USERPTR: //     init_userp(); //     break; //   case io_method_t::IO_METHOD_UNKNOWN: //     // TODO(flynneva): log something //     break;
  // }
}

void gPhoto2Cam::close_device()
{
  // Device is already closed
  if (m_camera == NULL) {return;}

  // Close the camera connection
  int ret = gp_camera_exit(m_camera, m_context);
  if (ret < GP_OK) {
    throw "Failed to close camera connection";
  }

  // Free the camera and context resources
  gp_camera_free(m_camera);
  gp_context_unref(m_context);
  
  // Set the camera and context pointers to NULL
  m_camera = NULL;
  m_context = NULL;

  // Device is already closed
  // if (m_fd == -1) {return;}

  // if (-1 == close(m_fd)) {
  //   throw strerror(errno);
  // }

  // m_fd = -1;
}

void gPhoto2Cam::open_device()
{
  int ret;
  Camera *camera = NULL;
  GPContext *context;

  // Create a new context
  context = gp_context_new();

  // Initialize the camera
  ret = gp_camera_new(&camera);
  if (ret < GP_OK) {
    throw "Failed to initialize camera";
  }

  // Open a session with the camera
  ret = gp_camera_init(camera, context);
  if (ret < GP_OK) {
    throw "Failed to open session with camera";
  }

  // Store the camera and context for later use
  m_camera = camera;
  m_context = context;

  // gp_camera_unref(camera);
  // gp_context_unref(context);

}

void gPhoto2Cam::configure(
  parameters_t & parameters,
  config_map_t & config_map)
{
  m_device_name = parameters.device_name;

  // Open device file descriptor before anything else
  std::cout << "Opening device..." << std::endl;
  open_device();
  std::cout << "Device open." << std::endl;

  m_image.width = static_cast<int>(parameters.image_width);
  m_image.height = static_cast<int>(parameters.image_height);
  // m_image.set_number_of_pixels();

  // Do this before calling set_bytes_per_line and set_size_in_bytes
  // m_image.pixel_format = set_pixel_format(parameters);
  // m_image.set_bytes_per_line();
  // m_image.set_size_in_bytes();
  // m_framerate = parameters.framerate;

  // Get all config
  CameraWidget *rootWidget = nullptr;

  std::cout << "Getting config..." << std::endl;
  int result;
  result = gp_camera_get_config(m_camera, &rootWidget, m_context);
  if (result != GP_OK) {
    std::cerr << "Failed to get camera config: " << gp_result_as_string(result) << std::endl;
  }
  std::cout << "Config loaded." << std::endl;

  getWidgetValue(rootWidget, config_map);

  std::cout << "Config parsed." << std::endl;

  gp_widget_free(rootWidget);

  init_device();
}

void gPhoto2Cam::start()
{
  start_capturing();
}

void gPhoto2Cam::shutdown()
{
  stop_capturing();
  uninit_device();
  close_device();
}

/// @brief Grab new image from V4L2 device, return pointer to image
/// @return pointer to image data
char * gPhoto2Cam::get_image()
{
  // if ((m_image.width == 0) || (m_image.height == 0)) {
  //   return nullptr;
  // }
  // grab the image
  grab_image();
  return m_image.data;
}

cv::Mat gPhoto2Cam::get_image_cv()
{
  // if ((m_image.width == 0) || (m_image.height == 0)) {
  //   return nullptr;
  // }
  // grab the image
  grab_image();
  return m_image.image;
}

/// @brief Overload get_image so users can pass in an image pointer to fill
/// @param destination destination to fill in with image
void gPhoto2Cam::get_image(char * destination)
{
  // if ((m_image.width == 0) || (m_image.height == 0)) {
  //   return;
  // }
  // Set the destination pointer to be filled
  m_image.data = destination;

  // grab the image
  grab_image();

}

void gPhoto2Cam::grab_image()
{

  std::lock_guard<std::mutex> lock(gp_mutex);

  int ret;
  const char *data;
  unsigned long int size;

  // Step 1: Ensure the camera is initialized and context is set up
  // Assuming m_camera and m_context are already initialized

  // Step 2: Capture preview
  ret = gp_file_new(&m_file);
  if (ret < GP_OK) {
      std::cerr << "Failed to create file object for preview image." << std::endl;
      return;
  }

  ret = gp_camera_capture_preview(m_camera, m_file, m_context);
  if (ret < GP_OK) {
      std::cerr << "Failed to capture preview image." << std::endl;
      gp_file_free(m_file);
      return;
  }

  // Step 3: Extract image data
  ret = gp_file_get_data_and_size(m_file, &data, &size);
  if (ret < GP_OK) {
      std::cerr << "Failed to get image data from file." << std::endl;
      gp_file_free(m_file);
      return;
  }

  if (m_image.data != nullptr) {
    delete[] m_image.data;
  }

  // std::cout << "Image size: " << size << std::endl;

  std::vector<uchar> jpeg_data(data, data + size);
  cv::Mat image = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
  if (image.empty()) {
      std::cerr << "Failed to decode image data." << std::endl;
      gp_file_free(m_file);
      return;
  }

  m_image.image = image.clone();

  clock_gettime(CLOCK_REALTIME, &m_image.stamp);

  // Print size of m_image.data


  // Step 4: Assign data to m_image
  // Assuming m_image.data is a suitable container for the image data, like std::vector<char>
  // m_image.data.assign(data, data + size);

  // Clean up
  gp_file_free(m_file);

}

/**
* Set video device parameter via call to v4l-utils.
*
* @param param The name of the parameter to set
* @param param The value to assign
*/
bool gPhoto2Cam::set_gphoto2_parameter(const std::string & param, int value)
{
  char buf[33];
  snprintf(buf, sizeof(buf), "%i", value);
  return set_gphoto2_parameter(param, buf);
}

/**
* Set video device parameter via call to v4l-utils.
*
* @param param The name of the parameter to set
* @param param The value to assign
*/
// bool gPhoto2Cam::set_gphoto2_parameter(const std::string & param, const std::string & value)
// {

// }

// Handle different types of widgets
// (char*) for GP_WIDGET_MENU, GP_WIDGET_TEXT, GP_WIDGET_RADIO, 
bool gPhoto2Cam::set_char_config(char_config_t & char_config) {

  std::lock_guard<std::mutex> lock(gp_mutex);

  // Get the widget from the camera based on the name supplied in char_config
  CameraWidget *widget = nullptr;
  int result = gp_camera_get_single_config(m_camera, char_config.name.c_str(), &widget, m_context);
  // Print result if failed
  if (result != GP_OK) {
    std::cerr << "Failed to get config: " << gp_result_as_string(result) << std::endl;
    return false;
  } else {
    // Set the value of the widget
    result = gp_widget_set_value(widget, char_config.value.c_str());
    // Print result if failed
    if (result != GP_OK) {
      std::cerr << "Failed to set config: " << gp_result_as_string(result) << std::endl;
      return false;
    }
    // Set the widget back to the camera
    result = gp_camera_set_single_config(m_camera, char_config.name.c_str(), widget, m_context);
    // Print result if failed
    if (result != GP_OK) {
      std::cerr << "Failed to set config: " << gp_result_as_string(result) << std::endl;
      return false;
    }
    // Free the widget
    gp_widget_free(widget);
    return true;

  }
}

// (float) for GP_WIDGET_RANGE, 
bool gPhoto2Cam::set_float_config(float_config_t & float_config) {

  std::lock_guard<std::mutex> lock(gp_mutex);

  // Get the widget from the camera based on the name supplied in float_config
  CameraWidget *widget = nullptr;
  int result = gp_camera_get_single_config(m_camera, float_config.name.c_str(), &widget, m_context);
  // Print result if failed
  if (result != GP_OK) {
    std::cerr << "Failed to get config: " << gp_result_as_string(result) << std::endl;
    return false;
  } else {
    // Set the value of the widget
    result = gp_widget_set_value(widget, &float_config.value);
    // Print result if failed
    if (result != GP_OK) {
      std::cerr << "Failed to set config: " << gp_result_as_string(result) << std::endl;
      return false;
    }
    // Set the widget back to the camera
    result = gp_camera_set_single_config(m_camera, float_config.name.c_str(), widget, m_context);
    // Print result if failed
    if (result != GP_OK) {
      std::cerr << "Failed to set config: " << gp_result_as_string(result) << std::endl;
      return false;
    }
    // Free the widget
    gp_widget_free(widget);
    return true;

  }
}

// (int) for GP_WIDGET_DATE, GP_WIDGET_TOGGLE, and 
bool gPhoto2Cam::set_int_config(int_config_t & int_config) {

  std::lock_guard<std::mutex> lock(gp_mutex);

  // Get the widget from the camera based on the name supplied in int_config
  CameraWidget *widget = nullptr;
  int result = gp_camera_get_single_config(m_camera, int_config.name.c_str(), &widget, m_context);
  // Print result if failed
  if (result != GP_OK) {
    std::cerr << "Failed to get config: " << gp_result_as_string(result) << std::endl;
    return false;
  } else {
    // Set the value of the widget
    result = gp_widget_set_value(widget, &int_config.value);
    // Print result if failed
    if (result != GP_OK) {
      std::cerr << "Failed to set config: " << gp_result_as_string(result) << std::endl;
      return false;
    }
    // Set the widget back to the camera
    result = gp_camera_set_single_config(m_camera, int_config.name.c_str(), widget, m_context);
    // Print result if failed
    if (result != GP_OK) {
      std::cerr << "Failed to set config: " << gp_result_as_string(result) << std::endl;
      return false;
    }
    // Free the widget
    gp_widget_free(widget);
    return true;

  }
}

// (CameraWidgetCallback) for GP_WIDGET_BUTTON. 

}  // namespace gphoto2_cam
