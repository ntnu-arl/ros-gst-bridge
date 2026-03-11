/* gst_bridge
 * Copyright (C) 2020-2021 Brett Downing <brettrd@brettrd.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
/**
 * SECTION:element-gstrosimagesink
 *
 * The rosimagesink element pipe video data into ROS2.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 -v videotestsrc ! rosimagesink node_name="gst_image" topic="/imagetopic"
 * ]|
 * Streams test tones as ROS image messages on topic.
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst_bridge/rosimagesink.h>
#include <jpeglib.h>
#include <cstdio>
#include <vector>
#include <pts_meta_map.h>

GST_DEBUG_CATEGORY_STATIC(rosimagesink_debug_category);
#define GST_CAT_DEFAULT rosimagesink_debug_category

/* prototypes */

static void rosimagesink_set_property(
  GObject * object, guint property_id, const GValue * value, GParamSpec * pspec);
static void rosimagesink_get_property(
  GObject * object, guint property_id, GValue * value, GParamSpec * pspec);

static void rosimagesink_init(Rosimagesink * sink);

static gboolean rosimagesink_open(RosBaseSink * sink);
static gboolean rosimagesink_close(RosBaseSink * sink);
static gboolean rosimagesink_setcaps(GstBaseSink * gst_base_sink, GstCaps * caps);
static GstFlowReturn rosimagesink_render(
  RosBaseSink * base_sink, GstBuffer * buffer, rclcpp::Time msg_time);

enum {
  PROP_0,
  PROP_ROS_TOPIC,
  PROP_ROS_FRAME_ID,
  PROP_ROS_ENCODING,
  PROP_COMPRESSED,
  PROP_COMPRESSION_QUALITY,
};

/* pad templates */

static GstStaticPadTemplate rosimagesink_sink_template = GST_STATIC_PAD_TEMPLATE(
  "sink", GST_PAD_SINK, GST_PAD_ALWAYS, GST_STATIC_CAPS(ROS_IMAGE_MSG_CAPS "; image/jpeg"));

/* class initialization */

G_DEFINE_TYPE_WITH_CODE(
  Rosimagesink, rosimagesink, GST_TYPE_ROS_BASE_SINK,
  GST_DEBUG_CATEGORY_INIT(
    rosimagesink_debug_category, "rosimagesink", 0, "debug category for rosimagesink element"))

static void rosimagesink_class_init(RosimagesinkClass * klass)
{
  GObjectClass * object_class = G_OBJECT_CLASS(klass);
  GstElementClass * element_class = GST_ELEMENT_CLASS(klass);
  GstBaseSinkClass * basesink_class = GST_BASE_SINK_CLASS(klass);
  RosBaseSinkClass * ros_base_sink_class = GST_ROS_BASE_SINK_CLASS(klass);

  object_class->set_property = rosimagesink_set_property;
  object_class->get_property = rosimagesink_get_property;

  /* Setting up pads and setting metadata should be moved to
     base_class_init if you intend to subclass this class. */
  gst_element_class_add_static_pad_template(element_class, &rosimagesink_sink_template);

  gst_element_class_set_static_metadata(
    element_class, "rosimagesink", "Sink", "a gstreamer sink that publishes image data into ROS",
    "BrettRD <brettrd@brettrd.com>");

  g_object_class_install_property(
    object_class, PROP_ROS_TOPIC,
    g_param_spec_string(
      "ros-topic", "pub-topic", "ROS topic to be published on", "gst_image_pub",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_ROS_FRAME_ID,
    g_param_spec_string(
      "ros-frame-id", "frame-id", "frame_id of the image message", "",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_ROS_ENCODING,
    g_param_spec_string(
      "ros-encoding", "encoding-string", "A hack to flexibly set the encoding string", "",
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_COMPRESSED,
    g_param_spec_boolean(
      "compressed", "compressed", "Enable JPEG compression", FALSE,
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
    object_class, PROP_COMPRESSION_QUALITY,
    g_param_spec_int(
      "compression-quality", "compression-quality", "JPEG compression quality (0-100)", 0, 100, 85,
      (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  //access gstreamer base sink events here
  basesink_class->set_caps =
    GST_DEBUG_FUNCPTR(rosimagesink_setcaps);  //gstreamer informs us what caps we're using.

  //supply the calls ros base sink needs to negotiate upstream formats and manage the publisher
  ros_base_sink_class->open =
    GST_DEBUG_FUNCPTR(rosimagesink_open);  //let the base sink know how we register publishers
  ros_base_sink_class->close =
    GST_DEBUG_FUNCPTR(rosimagesink_close);  //let the base sink know how we destroy publishers
  ros_base_sink_class->render =
    GST_DEBUG_FUNCPTR(rosimagesink_render);  // gives us a buffer to package
}

static void rosimagesink_init(Rosimagesink * sink)
{
  RosBaseSink * ros_base_sink GST_ROS_BASE_SINK(sink);
  ros_base_sink->node_name = g_strdup("gst_image_sink_node");
  sink->pub_topic = g_strdup("gst_image_pub");
  sink->frame_id = g_strdup("image_frame");
  sink->encoding = g_strdup("");
  sink->init_caps = g_strdup("");
  sink->compressed = FALSE;
  sink->compression_quality = 85;
  sink->input_is_jpeg = FALSE;
}

void rosimagesink_set_property(
  GObject * object, guint property_id, const GValue * value, GParamSpec * pspec)
{
  RosBaseSink * ros_base_sink = GST_ROS_BASE_SINK(object);
  Rosimagesink * sink = GST_ROSIMAGESINK(object);

  GST_DEBUG_OBJECT(sink, "set_property");

  switch (property_id) {
    case PROP_ROS_TOPIC:
      if (ros_base_sink->node_if) {
        RCLCPP_ERROR(
          ros_base_sink->node_if->logging->get_logger(), "can't change topic name once opened");
      } else {
        g_free(sink->pub_topic);
        sink->pub_topic = g_value_dup_string(value);
      }
      break;

    case PROP_ROS_FRAME_ID:
      g_free(sink->frame_id);
      sink->frame_id = g_value_dup_string(value);
      break;

    case PROP_ROS_ENCODING:
      g_free(sink->encoding);
      sink->encoding = g_value_dup_string(value);
      break;

    case PROP_COMPRESSED:
      if (ros_base_sink->node_if) {
        RCLCPP_ERROR(
          ros_base_sink->node_if->logging->get_logger(), "can't change compression setting once opened");
      } else {
        sink->compressed = g_value_get_boolean(value);
      }
      break;

    case PROP_COMPRESSION_QUALITY:
      sink->compression_quality = g_value_get_int(value);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
      break;
  }
}

void rosimagesink_get_property(
  GObject * object, guint property_id, GValue * value, GParamSpec * pspec)
{
  Rosimagesink * sink = GST_ROSIMAGESINK(object);

  GST_DEBUG_OBJECT(sink, "get_property");
  switch (property_id) {
    case PROP_ROS_TOPIC:
      g_value_set_string(value, sink->pub_topic);
      break;

    case PROP_ROS_FRAME_ID:
      g_value_set_string(value, sink->frame_id);
      break;

    case PROP_ROS_ENCODING:
      g_value_set_string(value, sink->encoding);
      break;

    case PROP_COMPRESSED:
      g_value_set_boolean(value, sink->compressed);
      break;

    case PROP_COMPRESSION_QUALITY:
      g_value_set_int(value, sink->compression_quality);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, property_id, pspec);
      break;
  }
}

/* open the device with given specs */
static gboolean rosimagesink_open(RosBaseSink * ros_base_sink)
{
  Rosimagesink * sink = GST_ROSIMAGESINK(ros_base_sink);
  GST_DEBUG_OBJECT(sink, "open");
  // Publisher creation is deferred to setcaps() where the actual input format
  // (raw video vs pre-encoded JPEG) is known, avoiding DDS type conflicts.
  return TRUE;
}

/* close the device */
static gboolean rosimagesink_close(RosBaseSink * ros_base_sink)
{
  Rosimagesink * sink = GST_ROSIMAGESINK(ros_base_sink);
  GST_DEBUG_OBJECT(sink, "close");
  sink->pub.reset();
  sink->compressed_pub.reset();
  return TRUE;
}

/* check the caps, register a node and open an publisher */
static gboolean rosimagesink_setcaps(GstBaseSink * gst_base_sink, GstCaps * caps)
{
  RosBaseSink * ros_base_sink = GST_ROS_BASE_SINK(gst_base_sink);
  Rosimagesink * sink = GST_ROSIMAGESINK(ros_base_sink);

  GstStructure * caps_struct;
  gint width, height, depth, endianness, rate_num, rate_den;
  const gchar * format_str;
  GstVideoFormat format_enum;
  const GstVideoFormatInfo * format_info;

  GST_DEBUG_OBJECT(sink, "setcaps");

  // Fast path: upstream is already a JPEG encoder (e.g. nvjpegenc).
  // Skip all video-format parsing and ensure we have a compressed publisher.
  caps_struct = gst_caps_get_structure(caps, 0);
  if (g_strcmp0(gst_structure_get_name(caps_struct), "image/jpeg") == 0) {
    sink->input_is_jpeg = TRUE;
    if (!sink->compressed_pub) {
      rclcpp::QoS qos = rclcpp::SensorDataQoS().reliable();
      sink->pub.reset();  // drop raw publisher first to avoid DDS type conflict
      sink->compressed_pub = rclcpp::create_publisher<sensor_msgs::msg::CompressedImage>(
        ros_base_sink->node_if->parameters, ros_base_sink->node_if->topics, sink->pub_topic, qos);
    }
    RCLCPP_INFO(
      ros_base_sink->node_if->logging->get_logger(),
      "setcaps: JPEG passthrough mode (GPU encoder)");
    return TRUE;
  }
  sink->input_is_jpeg = FALSE;

  if (!gst_caps_is_fixed(caps)) {
    RCLCPP_ERROR(ros_base_sink->node_if->logging->get_logger(), "caps is not fixed");
  }

  if (ros_base_sink->node_if)
    RCLCPP_INFO(
      ros_base_sink->node_if->logging->get_logger(), "preparing video with caps '%s'",
      gst_caps_to_string(caps));

  caps_struct = gst_caps_get_structure(caps, 0);
  if (!gst_structure_get_int(caps_struct, "width", &width))
    RCLCPP_ERROR(ros_base_sink->node_if->logging->get_logger(), "setcaps missing width");
  if (!gst_structure_get_int(caps_struct, "height", &height))
    RCLCPP_ERROR(ros_base_sink->node_if->logging->get_logger(), "setcaps missing height");
  if (!gst_structure_get_fraction(caps_struct, "framerate", &rate_num, &rate_den))
    RCLCPP_ERROR(ros_base_sink->node_if->logging->get_logger(), "setcaps missing framerate");

  format_str = gst_structure_get_string(caps_struct, "format");

  if (format_str) {
    format_enum = gst_video_format_from_string(format_str);
    format_info = gst_video_format_get_info(format_enum);
    depth = format_info->pixel_stride[0];

    //allow the encoding to be overridden by parameters
    //but update it if it's blank
    if (0 == g_strcmp0(sink->init_caps, "")) {
      g_free(sink->init_caps);
      sink->init_caps = gst_caps_to_string(caps);
    }
    if (0 == g_strcmp0(sink->encoding, "")) {
      g_free(sink->encoding);
      sink->encoding = g_strdup(gst_bridge::getRosEncoding(format_enum).c_str());
    }

    RCLCPP_INFO(
      ros_base_sink->node_if->logging->get_logger(), "setcaps format string is %s ", format_str);
    RCLCPP_INFO(
      ros_base_sink->node_if->logging->get_logger(), "setcaps n_components is %d",
      format_info->n_components);
    RCLCPP_INFO(
      ros_base_sink->node_if->logging->get_logger(), "setcaps bits is %d", format_info->bits);
    RCLCPP_INFO(ros_base_sink->node_if->logging->get_logger(), "setcaps pixel_stride is %d", depth);

    if (format_info->bits < 8) {
      depth = depth / 8;
      RCLCPP_ERROR(ros_base_sink->node_if->logging->get_logger(), "low bits per pixel");
    }
    endianness = GST_VIDEO_FORMAT_INFO_IS_LE(format_info) ? G_LITTLE_ENDIAN : G_BIG_ENDIAN;
  } else {
    RCLCPP_ERROR(ros_base_sink->node_if->logging->get_logger(), "setcaps missing format");
    if (!gst_structure_get_int(caps_struct, "endianness", &endianness))
      RCLCPP_ERROR(ros_base_sink->node_if->logging->get_logger(), "setcaps missing endianness");
    return false;
  }

  //collect a bunch of parameters to shoehorn into a message format
  sink->width = width;
  sink->height = height;
  sink->step = width * depth;     //full row step size in bytes
  sink->endianness = endianness;  // XXX used without init
  //sink->sample_rate = rate;

  // Create the publisher now that we know the output type (deferred from open()).
  rclcpp::QoS qos = rclcpp::SensorDataQoS().reliable();
  if (!sink->compressed) {
    if (!sink->pub) {
      sink->pub = rclcpp::create_publisher<sensor_msgs::msg::Image>(
        ros_base_sink->node_if->parameters, ros_base_sink->node_if->topics, sink->pub_topic, qos);
    }
  } else {
    if (!sink->compressed_pub) {
      sink->compressed_pub = rclcpp::create_publisher<sensor_msgs::msg::CompressedImage>(
        ros_base_sink->node_if->parameters, ros_base_sink->node_if->topics, sink->pub_topic, qos);
    }
  }

  return true;
}

static GstFlowReturn rosimagesink_render(
  RosBaseSink * ros_base_sink, GstBuffer * buf, rclcpp::Time msg_time)
{
  GstMapInfo info;
  Rosimagesink * sink = GST_ROSIMAGESINK(ros_base_sink);
  GST_DEBUG_OBJECT(sink, "render");

  guint64 pts = GST_BUFFER_PTS(buf);
  FrameMetaData meta;

  if (GST_CLOCK_TIME_IS_VALID(pts) && PtsMetaMap::getInstance().getAndRemoveMeta(pts, meta)) {
    if (ros_base_sink->node_if) {
      RCLCPP_INFO(ros_base_sink->node_if->logging->get_logger(),
                  "Frame PTS: %llu | Exposure: %llu ns | Analog Gain: %f | ISP Digital Gain: %f",
                  (unsigned long long)pts,
                  (unsigned long long)meta.exposureTime,
                  meta.analogGain,
                  meta.digitalGain);
    }
  } else {
    // If it misses, we log a warning debug to avoid spam, but let you know it dropped
    if (ros_base_sink->node_if) {
      RCLCPP_DEBUG(ros_base_sink->node_if->logging->get_logger(),
                  "Metadata not found in PTS Map for PTS: %llu", (unsigned long long)pts);
    }
  }

  gst_buffer_map(buf, &info, GST_MAP_READ);

  if (sink->input_is_jpeg) {
    // Input is already JPEG-encoded by the GPU (nvjpegenc).
    // Just wrap the bytes in a CompressedImage message — no CPU encoding.
    sensor_msgs::msg::CompressedImage compressed_msg;
    compressed_msg.header.stamp = msg_time;
    compressed_msg.header.frame_id = sink->frame_id;
    compressed_msg.format = "jpeg";
    compressed_msg.data.assign(info.data, info.data + info.size);
    sink->compressed_pub->publish(compressed_msg);
  } else if (!sink->compressed) {
    // Publish raw image
    sensor_msgs::msg::Image msg;
    msg.header.stamp = msg_time;
    msg.header.frame_id = sink->frame_id;
    msg.width = sink->width;
    msg.height = sink->height;
    msg.encoding = sink->encoding;
    msg.is_bigendian = (sink->endianness == G_BIG_ENDIAN);
    msg.step = sink->step;
    msg.data.assign(info.data, info.data + info.size);
    sink->pub->publish(msg);
  } else {
    // Publish compressed JPEG image using libjpeg
    sensor_msgs::msg::CompressedImage compressed_msg;
    compressed_msg.header.stamp = msg_time;
    compressed_msg.header.frame_id = sink->frame_id;
    compressed_msg.format = "jpeg";

    // Create JPEG compression
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    // Set up memory destination
    unsigned char* jpeg_buffer = nullptr;
    unsigned long jpeg_size = 0;
    jpeg_mem_dest(&cinfo, &jpeg_buffer, &jpeg_size);

    // Set compression parameters
    cinfo.image_width = sink->width;
    cinfo.image_height = sink->height;
    cinfo.input_components = 3;  // JPEG output is always RGB (3 components)
    cinfo.in_color_space = JCS_RGB;
    
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, sink->compression_quality, TRUE);
    
    jpeg_start_compress(&cinfo, TRUE);

    // Convert RGBA to RGB and process row by row
    std::vector<unsigned char> rgb_row(sink->width * 3);
    JSAMPROW row_pointer[1];
    unsigned char* image_data = info.data;
    
    while (cinfo.next_scanline < cinfo.image_height) {
      // Convert RGBA to RGB for current row
      unsigned char* rgba_row = &image_data[cinfo.next_scanline * sink->step];
      for (int x = 0; x < sink->width; x++) {
        rgb_row[x * 3 + 0] = rgba_row[x * 4 + 0]; // R
        rgb_row[x * 3 + 1] = rgba_row[x * 4 + 1]; // G  
        rgb_row[x * 3 + 2] = rgba_row[x * 4 + 2]; // B
        // Skip alpha channel (rgba_row[x * 4 + 3])
      }
      
      row_pointer[0] = rgb_row.data();
      jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    // Copy compressed data to ROS message
    if (jpeg_buffer && jpeg_size > 0) {
      compressed_msg.data.assign(jpeg_buffer, jpeg_buffer + jpeg_size);
      free(jpeg_buffer);
    }

    sink->compressed_pub->publish(compressed_msg);
  }

  gst_buffer_unmap(buf, &info);
  return GST_FLOW_OK;
}
