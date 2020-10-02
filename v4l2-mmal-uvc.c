/*
 * v4l2-mmal-uvc application
 *
 * Copyright (C) 2020 Kin Wei Lee
 *
 * based on uvc-gadget laurent.pinchart@ideasonboard.com
 * based on v4l2_mmal by 6by9 Raspberry Pi (Trading) Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 */

#define __STDC_FORMAT_MACROS

#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>

#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <time.h>
#include <sched.h>

#include <linux/usb/ch9.h>
#include <linux/usb/video.h>
#include <linux/videodev2.h>

#include "uvc.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "bcm_host.h"
#include "interface/vcsm/user-vcsm.h"

#include "v4l2-mmal-uvc.h"

#ifndef V4L2_BUF_FLAG_ERROR
#define V4L2_BUF_FLAG_ERROR	0x0040
#endif

//output image size fixed to 1280x720 
#define WIDTH 1280
#define HEIGHT 720
#define MAX_PLANES 8
int debug = 0;
#define print(...) do { if (debug) printf(__VA_ARGS__); } while (0)

/* Enable debug prints. */
#undef  ENABLE_BUFFER_DEBUG
//#define ENABLE_BUFFER_DEBUG
#undef  ENABLE_USB_REQUEST_DEBUG

#define CLEAR(x)	memset (&(x), 0, sizeof (x))
#define max(a, b)	(((a) > (b)) ? (a) : (b))

#define clamp(val, min, max) ({                 \
      typeof(val) __val = (val);              \
      typeof(min) __min = (min);              \
      typeof(max) __max = (max);              \
      (void) (&__val == &__min);              \
      (void) (&__val == &__max);              \
      __val = __val < __min ? __min: __val;   \
      __val > __max ? __max: __val; })

//#define ARRAY_SIZE(a) ((sizeof(a) / sizeof(a[0])))
#define pixfmtstr(x) 	(x) & 0xff, ((x) >> 8) & 0xff, ((x) >> 16) & 0xff, \
                      ((x) >> 24) & 0xff

/*
* The UVC webcam gadget kernel driver (g_webcam.ko) supports changing
* the Brightness attribute of the Processing Unit (PU). by default. If
* the underlying video capture device supports changing the Brightness
* attribute of the image being acquired (like the Virtual Video, VIVI
* driver), then we should route this UVC request to the respective
* video capture device.
*
* Incase, there is no actual video capture device associated with the
* UVC gadget and we wish to use this application as the final
* destination of the UVC specific requests then we should return
* pre-cooked (static) responses to GET_CUR(BRIGHTNESS) and
* SET_CUR(BRIGHTNESS) commands to keep command verifier test tools like
* UVC class specific test suite of USBCV, happy.
*
* Note that the values taken below are in sync with the VIVI driver and
* must be changed for your specific video capture device. These values
* also work well in case there in no actual video capture device.
*/
#define PU_BRIGHTNESS_MIN_VAL		0
#define PU_BRIGHTNESS_MAX_VAL		255
#define PU_BRIGHTNESS_STEP_SIZE		1
#define PU_BRIGHTNESS_DEFAULT_VAL	7

/* declarations */
int default_format = 0;		/* V4L2_PIX_FMT_YUYV */
static int video_queue_buffer (struct v4l2_device *dev, int index);
/* ---------------------------------------------------------------------------
* UVC specific stuff
*/

struct uvc_frame_info
{
  unsigned int width;
  unsigned int height;
  unsigned int intervals[8];
};

struct uvc_format_info
{
  unsigned int fcc;
  const struct uvc_frame_info *frames;
};
static const struct uvc_frame_info uvc_frames_yuyv[] = {
//  {640, 360, {66666, 100000, 500000, 0},},
//  {736, 480, {500000, 0},},
  {1280, 720, {400000, 0},},
  {0, 0, {0,},},
};



static const struct uvc_frame_info uvc_frames_mjpeg[] = {
//  {640, 360, {66666, 100000, 500000, 0},},
  {1280, 720, {500000},},
  {0, 0, {0,},},
};


static const struct uvc_format_info uvc_formats[] = {
//  {V4L2_PIX_FMT_YUYV, uvc_frames_yuyv},
  {V4L2_PIX_FMT_MJPEG, uvc_frames_mjpeg},
};

/* ---------------------------------------------------------------------------
 * V4L2 and UVC device instances
 */

static void
time_me (char *fn_name, struct v4l2_device *dev)
{
  struct timespec stt;
  static int i = 0;
  static float ft = 0;
  clock_gettime (CLOCK_MONOTONIC, &stt);
  if (i)
    ft +=
      ((float) (stt.tv_nsec - dev->last.tv_nsec) / 1e9 +
       (float) (stt.tv_sec - dev->last.tv_sec));
  if (i % 100 == 0)
    printf ("%s %0.2ffps\n", fn_name, i / ft );
  dev->last = stt;
  i++;
}



/* forward declarations */
static int uvc_video_stream (struct uvc_device *dev, int enable);


// print out some structure info
void
v4l2_dump (char *txt, struct v4l2_device *dev)
{
  print ("%s\n", txt);
  print ("dev->v4l2_fd %d\n", dev->v4l2_fd);
  print ("dev->type    %d\n", dev->type);
  print ("dev->memtype %d\n", dev->memtype);
}

static bool
video_is_mplane (struct v4l2_device *dev)
{
  return dev->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE || dev->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
}

static bool
video_is_capture (struct v4l2_device *dev)
{
  return dev->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE || dev->type == V4L2_BUF_TYPE_VIDEO_CAPTURE;
}

static bool
video_is_output (struct v4l2_device *dev)
{
  return dev->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE || dev->type == V4L2_BUF_TYPE_VIDEO_OUTPUT;
}


static bool
video_has_fd (struct v4l2_device *dev)
{
  return dev->v4l2_fd != -1;
}

static int
video_set_fd (struct v4l2_device *dev, int fd)
{
  if (video_has_fd (dev)) {
    print ("Can't set fd (already open).\n");
    return -1;
  }

  dev->v4l2_fd = fd;

  return 0;
}

static const struct v4l2_format_info *
v4l2_format_by_fourcc (unsigned int fourcc)
{
  unsigned int i;

  for (i = 0; i < ARRAY_SIZE (pixel_formats); ++i) {
    if (pixel_formats[i].fourcc == fourcc)
      return &pixel_formats[i];
  }

  return NULL;
}

static const struct v4l2_format_info *
v4l2_format_by_name (const char *name)
{
  unsigned int i;

  for (i = 0; i < ARRAY_SIZE (pixel_formats); ++i) {
    if (strcasecmp (pixel_formats[i].name, name) == 0)
      return &pixel_formats[i];
  }

  return NULL;
}

static const char *
v4l2_format_name (unsigned int fourcc)
{
  const struct v4l2_format_info *info;
  static char name[5];
  unsigned int i;

  info = v4l2_format_by_fourcc (fourcc);
  if (info)
    return info->name;

  for (i = 0; i < 4; ++i) {
    name[i] = fourcc & 0xff;
    fourcc >>= 8;
  }

  name[4] = '\0';
  return name;
}

static const struct
{
  const char *name;
  enum v4l2_field field;
} fields[] = {
  {"any", V4L2_FIELD_ANY},
  {"none", V4L2_FIELD_NONE},
  {"top", V4L2_FIELD_TOP},
  {"bottom", V4L2_FIELD_BOTTOM},
  {"interlaced", V4L2_FIELD_INTERLACED},
  {"seq-tb", V4L2_FIELD_SEQ_TB},
  {"seq-bt", V4L2_FIELD_SEQ_BT},
  {"alternate", V4L2_FIELD_ALTERNATE},
  {"interlaced-tb", V4L2_FIELD_INTERLACED_TB},
  {"interlaced-bt", V4L2_FIELD_INTERLACED_BT},
};

static enum v4l2_field
v4l2_field_from_string (const char *name)
{
  unsigned int i;

  for (i = 0; i < ARRAY_SIZE (fields); ++i) {
    if (strcasecmp (fields[i].name, name) == 0)
      return fields[i].field;
  }

  return -1;
}

static const char *
v4l2_field_name (enum v4l2_field field)
{
  unsigned int i;

  for (i = 0; i < ARRAY_SIZE (fields); ++i) {
    if (fields[i].field == field)
      return fields[i].name;
  }

  return "unknown";
}

static void
video_set_buf_type (struct v4l2_device *dev, enum v4l2_buf_type type)
{
  dev->type = type;
}

int
video_querycap (struct v4l2_device *dev, unsigned int *capabilities)
{
  struct v4l2_capability cap;
  unsigned int caps;
  int ret;

  memset (&cap, 0, sizeof cap);
  ret = ioctl (dev->v4l2_fd, VIDIOC_QUERYCAP, &cap);
  if (ret < 0)
    return 0;

  caps = cap.capabilities & V4L2_CAP_DEVICE_CAPS ? cap.device_caps : cap.capabilities;

  print
    ("Device `%s' on `%s' (driver '%s') is a video %s (%s mplanes) device.\n",
     cap.card, cap.bus_info, cap.driver,
     caps & (V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_VIDEO_CAPTURE) ?
     "capture" : "output",
     caps & (V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_VIDEO_OUTPUT_MPLANE) ?
     "with" : "without");

  *capabilities = caps;

  return 0;
}


static int
cap_get_buf_type (unsigned int capabilities)
{
  if (capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
    return V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  }
  else if (capabilities & V4L2_CAP_VIDEO_OUTPUT_MPLANE) {
    return V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  }
  else if (capabilities & V4L2_CAP_VIDEO_CAPTURE) {
    return V4L2_BUF_TYPE_VIDEO_CAPTURE;
  }
  else if (capabilities & V4L2_CAP_VIDEO_OUTPUT) {
    return V4L2_BUF_TYPE_VIDEO_OUTPUT;
  }
  else {
    print ("Device supports neither capture nor output.\n");
    return -EINVAL;
  }

  return 0;
}

static int
video_open (struct v4l2_device *dev, const char *devname)
{
  if (video_has_fd (dev)) {
    print ("Can't open device (already open).\n");
    return -1;
  }

  dev->v4l2_fd = open (devname, O_RDWR);
  if (dev->v4l2_fd < 0) {
    print ("Error opening device %s: %s (%d).\n", devname,
	   strerror (errno), errno);
    return dev->v4l2_fd;
  }

  print ("Device %s opened.\n", devname);

  dev->opened = 1;

  return 0;
}

static void
video_close (struct v4l2_device *dev)
{
  unsigned int i;

  for (i = 0; i < dev->num_planes; i++)
    free (dev->pattern[i]);

  free (dev->buffers);
  if (dev->opened)
    close (dev->v4l2_fd);
}


static int
video_get_format (struct v4l2_device *dev)
{
  struct v4l2_format fmt;
  unsigned int i;
  int ret;

  memset (&fmt, 0, sizeof fmt);
  fmt.type = dev->type;

  ret = ioctl (dev->v4l2_fd, VIDIOC_G_FMT, &fmt);
  if (ret < 0) {
    print ("Unable to get format: %s (%d).\n", strerror (errno), errno);
    return ret;
  }

  if (video_is_mplane (dev)) {
    dev->width = fmt.fmt.pix_mp.width;
    dev->height = fmt.fmt.pix_mp.height;
    dev->num_planes = fmt.fmt.pix_mp.num_planes;
    
    print ("Video format: %s (%08x) %ux%u field %s, %u planes: \n",
           v4l2_format_name (fmt.fmt.pix_mp.pixelformat),
           fmt.fmt.pix_mp.pixelformat, fmt.fmt.pix_mp.width,
           fmt.fmt.pix_mp.height, v4l2_field_name (fmt.fmt.pix_mp.field),
           fmt.fmt.pix_mp.num_planes);
    
    for (i = 0; i < fmt.fmt.pix_mp.num_planes; i++) {
      dev->plane_fmt[i].bytesperline = fmt.fmt.pix_mp.plane_fmt[i].bytesperline;
      dev->plane_fmt[i].sizeimage = fmt.fmt.pix_mp.plane_fmt[i].bytesperline ?
      fmt.fmt.pix_mp.plane_fmt[i].sizeimage : 0;
      
      print (" * Stride %u, buffer size %u\n",
             fmt.fmt.pix_mp.plane_fmt[i].bytesperline,
             fmt.fmt.pix_mp.plane_fmt[i].sizeimage);
    }
  }
  else {
    dev->width = fmt.fmt.pix.width;
    dev->height = fmt.fmt.pix.height;
    dev->num_planes = 1;

    dev->plane_fmt[0].bytesperline = fmt.fmt.pix.bytesperline;
    dev->plane_fmt[0].sizeimage = fmt.fmt.pix.bytesperline ? fmt.fmt.pix.sizeimage : 0;

    print
      ("Video format: %s (%08x) %ux%u (stride %u) field %s buffer size %u\n",
       v4l2_format_name (fmt.fmt.pix.pixelformat), fmt.fmt.pix.pixelformat,
       fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.bytesperline,
       v4l2_field_name (fmt.fmt.pix_mp.field), fmt.fmt.pix.sizeimage);
  }

  return 0;
}

static int
format_bpp (__u32 pixfmt)
{
  switch (pixfmt) {
  case V4L2_PIX_FMT_BGR24:
  case V4L2_PIX_FMT_RGB24:
    return 4;
  case V4L2_PIX_FMT_YUYV:
  case V4L2_PIX_FMT_YVYU:
  case V4L2_PIX_FMT_UYVY:
  case V4L2_PIX_FMT_VYUY:
    return 2;
  case V4L2_PIX_FMT_SRGGB8:
  case V4L2_PIX_FMT_SBGGR8:
  case V4L2_PIX_FMT_SGRBG8:
  case V4L2_PIX_FMT_SGBRG8:
    return 1;
  default:
    return 1;
  }
}

/*static int
video_set_quality (struct v4l2_device *dev, unsigned int quality)
{
  struct v4l2_jpegcompression jpeg;
  int ret;

  if (quality == (unsigned int) -1)
    return 0;

  memset (&jpeg, 0, sizeof jpeg);
  jpeg.quality = quality;

  ret = ioctl (dev->v4l2_fd, VIDIOC_S_JPEGCOMP, &jpeg);
  if (ret < 0) {
    print ("Unable to set quality to %u: %s (%d).\n", quality,
	   strerror (errno), errno);
    return ret;
  }

  ret = ioctl (dev->v4l2_fd, VIDIOC_G_JPEGCOMP, &jpeg);
  if (ret >= 0)
    print ("Quality set to %u\n", jpeg.quality);

  return 0;
}*/

static int
video_set_format (struct v4l2_device *dev, unsigned int w, unsigned int h,
		  unsigned int format, unsigned int stride,
		  unsigned int buffer_size, enum v4l2_field field,
		  unsigned int flags)
{
  struct v4l2_format fmt;
  unsigned int i;
  int ret;
  print ("video_set_format width %d, height %d\n", w, h);
  memset (&fmt, 0, sizeof fmt);
  fmt.type = dev->type;

  if (video_is_mplane (dev)) {
    const struct v4l2_format_info *info = v4l2_format_by_fourcc (format);

    fmt.fmt.pix_mp.width = w;
    fmt.fmt.pix_mp.height = h;
    fmt.fmt.pix_mp.pixelformat = format;
    fmt.fmt.pix_mp.field = field;
    fmt.fmt.pix_mp.num_planes = info->n_planes;
    fmt.fmt.pix_mp.flags = flags;

    for (i = 0; i < fmt.fmt.pix_mp.num_planes; i++) {
      fmt.fmt.pix_mp.plane_fmt[i].bytesperline = stride;
      fmt.fmt.pix_mp.plane_fmt[i].sizeimage = buffer_size;
    }
  }
  else {
    fmt.fmt.pix.width = w;
    fmt.fmt.pix.height = h;
    fmt.fmt.pix.pixelformat = format;
    fmt.fmt.pix.field = field;
    print ("stride is %d\n", stride);
    if (!stride)
      stride = ((w + 31) & ~31) * format_bpp (format);
    print ("stride is now %d\n", stride);
    fmt.fmt.pix.bytesperline = stride;
    fmt.fmt.pix.sizeimage = buffer_size;
    fmt.fmt.pix.priv = V4L2_PIX_FMT_PRIV_MAGIC;
    fmt.fmt.pix.flags = flags;
  }

  ret = ioctl (dev->v4l2_fd, VIDIOC_S_FMT, &fmt);
  if (ret < 0) {
    printf("Unable to set format: %s (%d).\n", strerror (errno), errno);
    return ret;
  }

  if (video_is_mplane (dev)) {
    printf("Video format set: %s (%08x) %ux%u field %s, %u planes: \n",
           v4l2_format_name (fmt.fmt.pix_mp.pixelformat),
           fmt.fmt.pix_mp.pixelformat, fmt.fmt.pix_mp.width,
           fmt.fmt.pix_mp.height, v4l2_field_name (fmt.fmt.pix_mp.field),
           fmt.fmt.pix_mp.num_planes);
    
    for (i = 0; i < fmt.fmt.pix_mp.num_planes; i++) {
      printf(" * Stride %u, buffer size %u\n",
             fmt.fmt.pix_mp.plane_fmt[i].bytesperline,
             fmt.fmt.pix_mp.plane_fmt[i].sizeimage);
    }
  }
  else {
    printf
      ("Video format set: %s (%08x) %ux%u (stride %u) field %s buffer size %u\n",
       v4l2_format_name (fmt.fmt.pix.pixelformat), fmt.fmt.pix.pixelformat,
       fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.bytesperline,
       v4l2_field_name (fmt.fmt.pix.field), fmt.fmt.pix.sizeimage);
  }

  return 0;
}

// create dma buffer using vcm and link it to v4l2 buffer
static int
buffer_export (int v4l2fd, enum v4l2_buf_type bt, int index, int *dmafd,
	       unsigned int *vcsm_hdl)
{
  struct v4l2_exportbuffer expbuf;
  unsigned int vcsm_handle;

  memset (&expbuf, 0, sizeof (expbuf));
  expbuf.type = bt;
  expbuf.index = index;
  if (ioctl (v4l2fd, VIDIOC_EXPBUF, &expbuf)) {
    print ("Failed to EXPBUF\n");
    return -1;
  }
  *dmafd = expbuf.fd;

  print ("Importing DMABUF %d into VCSM...\n", expbuf.fd);
  vcsm_handle = vcsm_import_dmabuf (expbuf.fd, "V4L2 buf");
  if (vcsm_handle)
    print ("...done. vcsm_handle %u\n", vcsm_handle);
  else
    print ("...done. Failed\n");
  *vcsm_hdl = vcsm_handle;
  return vcsm_handle ? 0 : -1;
}

static int
video_buffer_mmap (struct v4l2_device *dev, struct mmal_buffer *buffer,
		   struct v4l2_buffer *v4l2buf)
{
  unsigned int length;
  unsigned int offset;
  unsigned int i;

  for (i = 0; i < dev->num_planes; i++) {
    if (video_is_mplane (dev)) {
      length = v4l2buf->m.planes[i].length;
      offset = v4l2buf->m.planes[i].m.mem_offset;
    }
    else {
      length = v4l2buf->length;
      offset = v4l2buf->m.offset;
    }

    buffer->mem[i] = mmap (0, length, PROT_READ | PROT_WRITE, MAP_SHARED,
			   dev->v4l2_fd, offset);

    if (buffer->mem[i] == MAP_FAILED) {
      print ("Unable to map buffer %u/%u: %s (%d)\n", buffer->idx, i, strerror (errno), errno);
      return -1;
    }

    buffer->size[i] = length;
    buffer->padding[i] = 0;

    print ("Buffer %u/%u mapped at address %p.\n", buffer->idx, i, buffer->mem[i]);
  }

  return 0;
}

static int
video_buffer_munmap (struct v4l2_device *dev, struct mmal_buffer *buffer)
{
  unsigned int i;
  int ret;

  for (i = 0; i < dev->num_planes; i++) {
    ret = munmap (buffer->mem[i], buffer->size[i]);
    if (ret < 0) {
      print ("Unable to unmap buffer %u/%u: %s (%d)\n",
	     buffer->idx, i, strerror (errno), errno);
    }

    buffer->mem[i] = NULL;
  }

  return 0;
}

/*
static int
video_buffer_alloc_userptr (struct v4l2_device *dev,
			    struct mmal_buffer *buffer,
			    struct v4l2_buffer *v4l2buf, unsigned int offset,
			    unsigned int padding)
{
  int page_size = getpagesize ();
  unsigned int length;
  unsigned int i;
  int ret;

  for (i = 0; i < dev->num_planes; i++) {
    if (video_is_mplane (dev))
      length = v4l2buf->m.planes[i].length;
    else
      length = v4l2buf->length;

    ret = posix_memalign (&buffer->mem[i], page_size,
			  length + offset + padding);
    if (ret < 0) {
      print ("Unable to allocate buffer %u/%u (%d)\n", buffer->idx, i, ret);
      return -ENOMEM;
    }

    buffer->mem[i] += offset;
    buffer->size[i] = length;
    buffer->padding[i] = padding;

    print ("Buffer %u/%u allocated at address %p.\n",
	   buffer->idx, i, buffer->mem[i]);
  }

  return 0;
}
*/

static void
video_buffer_free_userptr (struct v4l2_device *dev,
			   struct mmal_buffer *buffer)
{
  unsigned int i;

  for (i = 0; i < dev->num_planes; i++) {
    free (buffer->mem[i]);
    buffer->mem[i] = NULL;
  }
}

static void
get_ts_flags (uint32_t flags, const char **ts_type, const char **ts_source)
{
  switch (flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) {
  case V4L2_BUF_FLAG_TIMESTAMP_UNKNOWN:
    *ts_type = "unk";
    break;
  case V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC:
    *ts_type = "mono";
    break;
  case V4L2_BUF_FLAG_TIMESTAMP_COPY:
    *ts_type = "copy";
    break;
  default:
    *ts_type = "inv";
  }
  switch (flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) {
  case V4L2_BUF_FLAG_TSTAMP_SRC_EOF:
    *ts_source = "EoF";
    break;
  case V4L2_BUF_FLAG_TSTAMP_SRC_SOE:
    *ts_source = "SoE";
    break;
  default:
    *ts_source = "inv";
  }
}

static int
video_alloc_buffers (struct v4l2_device *dev, int nbufs)
{
  struct v4l2_plane planes[MAX_PLANES];
  struct v4l2_requestbuffers rb;
  struct v4l2_buffer buf;
  struct mmal_buffer *buffers;
  unsigned int i;
  int ret;

  memset (&rb, 0, sizeof rb);
  rb.count = nbufs;
  rb.type = dev->type;
  rb.memory = dev->memtype;
  ret = ioctl (dev->v4l2_fd, VIDIOC_REQBUFS, &rb);
  if (ret < 0) {
    print ("V4L2: Unable to request buffers: %s (%d).\n", strerror (errno), errno);
    return ret;
  }
  //printf ("video_alloc_buffers fd%d %d %d\n", dev->v4l2_fd, dev->type, ret);

  printf ("%u buffers requested, V4L2 returned %u bufs.\n", nbufs, rb.count);

  buffers = malloc (rb.count * sizeof buffers[0]);
  if (buffers == NULL)
    return -ENOMEM;

  /* Map the buffers. */
  for (i = 0; i < rb.count; ++i) {
    const char *ts_type, *ts_source;

    memset (&buf, 0, sizeof buf);
    memset (planes, 0, sizeof planes);

    buf.index = i;
    buf.type = dev->type;
    buf.memory = dev->memtype;
    buf.length = MAX_PLANES;
    buf.m.planes = planes;

    ret = ioctl (dev->v4l2_fd, VIDIOC_QUERYBUF, &buf);
    if (ret < 0) {
      print ("Unable to query buffer %u: %s (%d).\n", i, strerror (errno), errno);
      return ret;
    }
    get_ts_flags (buf.flags, &ts_type, &ts_source);
    print ("length: %u offset: %u timestamp type/source: %s/%s\n", buf.length, buf.m.offset, ts_type, ts_source);

    buffers[i].idx = i;

    switch (dev->memtype) {
    case V4L2_MEMORY_MMAP:
      ret = video_buffer_mmap (dev, &buffers[i], &buf);
      break;

    /*case V4L2_MEMORY_USERPTR:
      ret = video_buffer_alloc_userptr (dev, &buffers[i], &buf, offset, padding);
      break;*/

    default:
      ret = -1;
      break;
    }

    if (ret < 0)
      return ret;

    if (!buffer_export(dev->v4l2_fd, dev->type, i, &buffers[i].dma_fd, &buffers[i].vcsm_handle)) {
      dev->can_zero_copy = MMAL_TRUE;
      print ("Exported buffer %d to dmabuf %d, vcsm handle %u\n", i,
             buffers[i].dma_fd, buffers[i].vcsm_handle);
    }
    else {
      if (dev->can_zero_copy) {
        print ("Some buffer exported whilst others not. HELP!\n");
        dev->can_zero_copy = MMAL_FALSE;
      }
    }
    if (dev->mmal_pool) {
      MMAL_BUFFER_HEADER_T *mmal_buf;
      mmal_buf = mmal_queue_get (dev->mmal_pool->queue);
      if (!mmal_buf) {
        print
        ("Failed to get a buffer from the pool. Queue length %d\n", mmal_queue_length (dev->mmal_pool->queue));
        return -1;
      }
      mmal_buf->user_data = &buffers[i];
      if (dev->can_zero_copy)
        mmal_buf->data = (uint8_t *) vcsm_vc_hdl_from_hdl (buffers[i].vcsm_handle);
      else
        mmal_buf->data = buffers[i].mem[0];
      mmal_buf->alloc_size = buf.length;
      buffers[i].mmal = mmal_buf;
      print	("Linking V4L2 buffer index %d ptr %p to MMAL header %p. mmal->data 0x%X\n",
             i, &buffers[i], mmal_buf, (uint32_t) mmal_buf->data);
      /* Put buffer back in the pool */
      mmal_buffer_header_release (mmal_buf);
    }
  }

  dev->timestamp_type = buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK;
  dev->buffers = buffers;
  dev->nbufs = rb.count;
  return 0;
}


static int
video_queue_all_buffers (struct v4l2_device *dev)
{
  unsigned int i;
  int ret;
  print ("V4L2: video_queue_all_buffers try %d \n", dev->nbufs);
  /* Queue the buffers. */
  for (i = 0; i < dev->nbufs; ++i) {
    ret = video_queue_buffer (dev, i);//, fill);
    if (ret < 0)
      return ret;
  }

  print ("V4L2: video_queue_all_buffers ok %d %lld\n", dev->nbufs,
	 dev->qbuf_count);
  return 0;
}

static void
video_verify_buffer (struct v4l2_device *dev, struct v4l2_buffer *buf)
{
  struct mmal_buffer *buffer = &dev->buffers[buf->index];
  unsigned int plane;
  unsigned int i;
  
  for (plane = 0; plane < dev->num_planes; ++plane) {
    const uint8_t *data = buffer->mem[plane] + buffer->size[plane];
    unsigned int errors = 0;
    unsigned int dirty = 0;
    
    if (buffer->padding[plane] == 0)
      continue;
    
    for (i = 0; i < buffer->padding[plane]; ++i) {
      if (data[i] != 0x55) {
        errors++;
        dirty = i + 1;
      }
    }
    
    if (errors) {
      print
      ("Warning: %u bytes overwritten among %u first padding bytes for plane %u\n",
       errors, dirty, plane);
      
      dirty = (dirty + 15) & ~15;
      dirty = dirty > 32 ? 32 : dirty;
      
      for (i = 0; i < dirty; ++i) {
        print ("%02x ", data[i]);
        if (i % 16 == 15)
          print ("\n");
      }
    }
  }
}

static int
video_free_buffers (struct v4l2_device *dev)
{
  struct v4l2_requestbuffers rb;
  unsigned int i;
  int ret;
  
  if (dev->nbufs == 0)
    return 0;
  
  for (i = 0; i < dev->nbufs; ++i) {
    switch (dev->memtype) {
      case V4L2_MEMORY_MMAP:
        if (dev->buffers[i].vcsm_handle) {
          print ("Releasing vcsm handle %u\n", dev->buffers[i].vcsm_handle);
          vcsm_free (dev->buffers[i].vcsm_handle);
        }
        if (dev->buffers[i].dma_fd) {
          print ("Closing dma_buf %d\n", dev->buffers[i].dma_fd);
          close (dev->buffers[i].dma_fd);
        }
        ret = video_buffer_munmap (dev, &dev->buffers[i]);
        if (ret < 0)
          return ret;
        break;
      case V4L2_MEMORY_USERPTR:
        video_buffer_free_userptr (dev, &dev->buffers[i]);
        break;
      default:
        break;
    }
  }
  
  memset (&rb, 0, sizeof rb);
  rb.count = 0;
  rb.type = dev->type;
  rb.memory = dev->memtype;
  
  ret = ioctl (dev->v4l2_fd, VIDIOC_REQBUFS, &rb);
  if (ret < 0) {
    print ("Unable to release buffers: %s (%d).\n", strerror (errno), errno);
    return ret;
  }
  
  print ("%u buffers released.\n", dev->nbufs);
  
  free (dev->buffers);
  dev->nbufs = 0;
  dev->buffers = NULL;
  
  return 0;
}


static int
video_queue_buffer (struct v4l2_device *dev, int index)
{
  struct v4l2_buffer buf;
  struct v4l2_plane planes[MAX_PLANES];
  int ret;
  unsigned int i;

  memset (&buf, 0, sizeof buf);
  memset (&planes, 0, sizeof planes);

  buf.index = index;
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;	//dev->type;
  buf.memory = V4L2_MEMORY_MMAP;	//dev->memtype;
  //print (" video_queue_buf \n");
  /*if (video_is_output (dev))
     { 
     buf.flags = dev->buffer_output_flags;
     if (dev->timestamp_type == V4L2_BUF_FLAG_TIMESTAMP_COPY)
     { 
     struct timespec ts;

     clock_gettime (CLOCK_MONOTONIC, &ts);
     buf.timestamp.tv_sec = ts.tv_sec;
     buf.timestamp.tv_usec = ts.tv_nsec / 1000;
     }
     }

     if (video_is_mplane (dev))
     { 
     buf.m.planes = planes;
     buf.length = dev->num_planes;
     }

     if (dev->memtype == V4L2_MEMORY_USERPTR)
     { 
     print ("v4l2 memtype is V4L2_MEMORY_USERPTR\n");
     if (video_is_mplane (dev))
     { 
     for (i = 0; i < dev->num_planes; i++)
     { 
     buf.m.planes[i].m.userptr = (unsigned long)
     dev->buffers[index].mem[i];
     buf.m.planes[i].length = dev->buffers[index].size[i];
     }
     }
     else
     { 
     buf.m.userptr = (unsigned long) dev->buffers[index].mem[0];
     buf.length = dev->buffers[index].size[0];
     }
     }

     for (i = 0; i < dev->num_planes; i++)
     { 
     if (video_is_output (dev))
     {
     if (video_is_mplane (dev))
     buf.m.planes[i].bytesused = dev->patternsize[i];
     else
     buf.bytesused = dev->patternsize[i];

     memcpy (dev->buffers[buf.index].mem[i], dev->pattern[i],
     dev->patternsize[i]);
     }
     else
     {
     if (fill & BUFFER_FILL_FRAME)
     memset (dev->buffers[buf.index].mem[i], 0x55,
     dev->buffers[index].size[i]);
     if (fill & BUFFER_FILL_PADDING)
     memset (dev->buffers[buf.index].mem[i] +
     dev->buffers[index].size[i],
     0x55, dev->buffers[index].padding[i]);
     }
     } */

  ret = ioctl (dev->v4l2_fd, VIDIOC_QBUF, &buf);
  if (ret < 0)
    print ("MMAL: Unable to queue buffer: %s (%d).\n", strerror (errno),
	   errno);
  dev->qbuf_count++;
  return ret;
}

static int
video_enable (struct v4l2_device *dev, int enable)
{
  int type = dev->type;
  int ret;

  ret =
    ioctl (dev->v4l2_fd, enable ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &type);
  if (ret < 0) {
    print ("Unable to %s streaming: %s (%d).\n",
	   enable ? "start" : "stop", strerror (errno), errno);
    return ret;
  }
  printf ("V4L2: %s\n", enable ? "VIDIOC_STREAMON" : "VIDIOC_STREAMOFF");
  return 0;
}


static unsigned int
video_buffer_bytes_used (struct v4l2_device *dev, struct v4l2_buffer *buf)
{
  unsigned int bytesused = 0;
  unsigned int i;

  if (!video_is_mplane (dev))
    return buf->bytesused;

  for (i = 0; i < dev->num_planes; i++)
    bytesused += buf->m.planes[i].bytesused;

  return bytesused;
}

static int
video_set_dv_timings (struct v4l2_device *dev)
{
  struct v4l2_dv_timings timings;
  v4l2_std_id std;
  int ret;
  
  memset (&timings, 0, sizeof timings);
  ret = ioctl (dev->v4l2_fd, VIDIOC_QUERY_DV_TIMINGS, &timings);
  if (ret >= 0) {
    printf ("QUERY_DV_TIMINGS returned %ux%u pixclk %llu\n", timings.bt.width, timings.bt.height, timings.bt.pixelclock);
    //Can read DV timings, so set them.
    ret = ioctl (dev->v4l2_fd, VIDIOC_S_DV_TIMINGS, &timings);
    if (ret < 0) {
      print ("Failed to set DV timings\n");
      return -1;
    }
    else {
      double tot_height, tot_width;
      const struct v4l2_bt_timings *bt = &timings.bt;
      
      tot_height = bt->height +
      bt->vfrontporch + bt->vsync + bt->vbackporch +
      bt->il_vfrontporch + bt->il_vsync + bt->il_vbackporch;
      tot_width = bt->width + bt->hfrontporch + bt->hsync + bt->hbackporch;
      dev->fps =
      (unsigned int) ((double) bt->pixelclock / (tot_width * tot_height));
      printf ("Framerate is %u\n", dev->fps);
    }
  }
  else {
    memset (&std, 0, sizeof std);
    ret = ioctl (dev->v4l2_fd, VIDIOC_QUERYSTD, &std);
    if (ret >= 0) {
      //Can read standard, so set it.
      ret = ioctl (dev->v4l2_fd, VIDIOC_S_STD, &std);
      if (ret < 0) {
        print ("Failed to set standard\n");
        return -1;
      }
      else {
        // SD video - assume 50Hz / 25fps
        dev->fps = 25;
      }
    }
  }
  return 0;
}

//process data here 'p
static int
v4l2_process_data (struct v4l2_device *dev)
{
  struct v4l2_buffer vbuf;
  struct v4l2_buffer ubuf;
  struct v4l2_plane planes[MAX_PLANES];
  int queue_buffer = 1;
  struct timespec start;
  struct timeval last;
  struct timespec ts;
  const char *ts_type, *ts_source;
  unsigned int size;
  static unsigned int i = 0;
  double bps;
  double fps;
  int ret;
  int dropped_frames = 0;
  static int frames = 0;


/* Return immediately if V4l2 streaming has not yet started. */
  if (!dev->is_streaming)
    return 0;

  if (dev->udev->first_buffer_queued) {
    if (dev->dqbuf_count >= dev->qbuf_count) {
      return 0;
    }
  }

  clock_gettime (CLOCK_MONOTONIC, &start);
  last.tv_sec = start.tv_sec;
  last.tv_usec = start.tv_nsec / 1000;

  /* Dequeue spent buffer rom V4L2 domain. */
  CLEAR (vbuf);
  CLEAR (planes);
  vbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;	//dev->type;
  vbuf.memory = V4L2_MEMORY_MMAP;	// dev->memtype
  vbuf.length = MAX_PLANES;
  vbuf.m.planes = planes;

  ret = ioctl (dev->v4l2_fd, VIDIOC_DQBUF, &vbuf);
  if (ret < 0) {
    print ("V4L2: VIDIOC_DQBUF vbuf failed %s (%d)\n", strerror (errno), errno);
    return ret;
  }
  dev->dqbuf_count++;
  dev->dq_ubuf_ok = false;

//  video_verify_buffer (dev, &vbuf); 

  fps = (vbuf.timestamp.tv_sec - last.tv_sec) * 1000000
    + vbuf.timestamp.tv_usec - last.tv_usec;
  fps = fps ? 1000000.0 / fps : 0.0;

  clock_gettime (CLOCK_MONOTONIC, &ts);
  get_ts_flags (vbuf.flags, &ts_type, &ts_source);
  print
    ("%u (%u) [%c] %s %u %u B %ld.%06ld %ld.%06ld %.3f fps ts %s/%s \n",
     i, vbuf.index, (vbuf.flags & V4L2_BUF_FLAG_ERROR) ? 'E' : '-',
     v4l2_field_name (vbuf.field), vbuf.sequence,
     video_buffer_bytes_used (dev, &vbuf), vbuf.timestamp.tv_sec,
     vbuf.timestamp.tv_usec, ts.tv_sec, ts.tv_nsec / 1000, fps,
     ts_type, ts_source);

  last = vbuf.timestamp;

  //go ize += buf.bytesused; buf send to MMAL input_isp

  print ("Dequeueing buffer at V4L2 side = %d\n", vbuf.index);

  //get data from mmal
  if (dev->mmal_pool) {
    MMAL_BUFFER_HEADER_T *mmal;
    MMAL_STATUS_T status;

    while ((mmal = mmal_queue_get (dev->mmal_pool->queue)) && !mmal->user_data) {
      printf ("Discarding MMAL buffer %p as not mapped\n", mmal);
    }
    //print ("v4l2_process_data\n");
    if (!mmal) {
      printf ("Failed to get MMAL buffer\n");
    }
    else {
      // Need to wait for MMAL to be finished with the buffer before returning to V4L2 //
      queue_buffer = 0;
      if (((struct mmal_buffer *) mmal->user_data)->idx != vbuf.index) {
        printf
        ("Mismatch in expected buffers. V4L2 gave idx %d, MMAL expecting %d\n",
         vbuf.index, ((struct mmal_buffer *) mmal->user_data)->idx);
      }
      
      mmal->length = vbuf.length;	//Deliberately use length as MMAL wants the padding
      
      print("mmal data %d length %d\n",((struct mmal_buffer *) mmal->user_data)->idx, mmal->length);
      if (!dev->starttime.tv_sec)
        dev->starttime = vbuf.timestamp;

      struct timeval pts;
      timersub (&vbuf.timestamp, &dev->starttime, &pts);
      //MMAL PTS is in usecs, so convert from struct timeval
      mmal->pts = (pts.tv_sec * 1000000) + pts.tv_usec;
      if (mmal->pts > (dev->lastpts + dev->frame_time_usec + 1000)) {
        print ("DROPPED FRAME - %lld and %lld, delta %lld\n",
               dev->lastpts, mmal->pts, mmal->pts - dev->lastpts);
        dropped_frames++;
      }
      dev->lastpts = mmal->pts;

      mmal->flags = MMAL_BUFFER_HEADER_FLAG_FRAME_END;
      //print ("about to send isp input buffer to mmal\n"); //mmal->pts = buf.timestamp;
      status = mmal_port_send_buffer (dev->isp->input[0], mmal);
      if (status != MMAL_SUCCESS)
        print ("mmal_port_send_buffer failed %d\n", status);

    }
  }
  i++;
  //video_queue_buffer (dev, vbuf.index, BUFFER_FILL_NONE );
  return 0;
}

/* ---------------------------------------------------------------------------
 * V4L2 generic stuff
 */

static int
v4l2_get_format (struct v4l2_device *dev)
{
  struct v4l2_format fmt;
  int ret;
  CLEAR (fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  ret = ioctl (dev->v4l2_fd, VIDIOC_G_FMT, &fmt);
  if (ret < 0) {
    printf ("V4L2: Unable to get format: %s (%d).\n", strerror (errno), errno);
    return ret;
  }

  printf ("V4L2: Getting current format: %c%c%c%c %ux%u\n",
	  pixfmtstr (fmt.fmt.pix.pixelformat),
	  fmt.fmt.pix.width, fmt.fmt.pix.height);

  return 0;
}

static int
v4l2_set_format (struct v4l2_device *dev, struct v4l2_format *fmt)
{
  int ret;
  printf ("V4L2: Want setting format to: %c%c%c%c %ux%u\n",
	  pixfmtstr (fmt->fmt.pix.pixelformat),
	  fmt->fmt.pix.width, fmt->fmt.pix.height);

  ret = ioctl (dev->v4l2_fd, VIDIOC_S_FMT, fmt);
  if (ret < 0) {
    printf ("V4L2_set_format: Unable to set format %s (%d).\n", strerror (errno), errno);
    return ret;
  }

  printf ("V4L2: Setting format to: %c%c%c%c %ux%u\n",
	  pixfmtstr (fmt->fmt.pix.pixelformat),
	  fmt->fmt.pix.width, fmt->fmt.pix.height);

  return 0;
}

static int
v4l2_set_ctrl (struct v4l2_device *dev, int new_val, int ctrl)
{
  struct v4l2_queryctrl queryctrl;
  struct v4l2_control control;
  int ret;

  CLEAR (queryctrl);

  switch (ctrl) {
  case V4L2_CID_BRIGHTNESS:
    queryctrl.id = V4L2_CID_BRIGHTNESS;
    ret = ioctl (dev->v4l2_fd, VIDIOC_QUERYCTRL, &queryctrl);
    //ret = 0;
      if (-1 == ret) {
        if (errno != EINVAL)
          printf ("V4L2: VIDIOC_QUERYCTRL"
                  " failed: %s (%d).\n", strerror (errno), errno);
        else
          printf ("V4L2_CID_BRIGHTNESS is not"
                  " supported: %s (%d).\n", strerror (errno), errno);
        
        return ret;
    }
    else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
      printf ("V4L2_CID_BRIGHTNESS is not supported.\n");
      ret = -EINVAL;
      return ret;
    }
    else {
      CLEAR (control);
      control.id = V4L2_CID_BRIGHTNESS;
      control.value = new_val;

      ret = ioctl (dev->v4l2_fd, VIDIOC_S_CTRL, &control);
      if (-1 == ret) {
	printf ("V4L2: VIDIOC_S_CTRL failed: %s (%d).\n",
		strerror (errno), errno);
	return ret;
      }
    }
    printf ("V4L2: Brightness control changed to value = 0x%x\n", new_val);
    break;

  default:
    printf ("V4L2: ctrl not supported %d\n", ctrl);
    /* TODO: We don't support any other controls. */
    return -EINVAL;
  }

  return 0;
}


static int
v4l2_open (struct v4l2_device **v4l2, char *devname, struct v4l2_format *s_fmt)
{
  struct v4l2_device *dev;
  struct v4l2_capability cap;
  int fd;
  int ret = -EINVAL;

  fd = open (devname, O_RDWR | O_NONBLOCK, 0);
  if (fd == -1) {
    printf ("V4L2: device open failed: %s (%d).\n", strerror (errno), errno);
    return ret;
  }

  ret = ioctl (fd, VIDIOC_QUERYCAP, &cap);
  if (ret < 0) {
    printf ("V4L2: VIDIOC_QUERYCAP failed: %s (%d).\n",
	    strerror (errno), errno);
    goto err;
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    printf ("V4L2: %s is no video capture device\n", devname);
    goto err;
  }

  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    printf ("V4L2: %s does not support streaming i/o\n", devname);
    goto err;
  }

  dev = calloc (1, sizeof *dev);
  if (dev == NULL) {
    ret = -ENOMEM;
    goto err;
  }

  printf ("V4L2 device is %s on bus %s\n", cap.card, cap.bus_info);

  dev->v4l2_fd = fd;
  //dev->fd = fd;
//print ("opening %d\n",dev->v4l2_fd);
  /* Get the default image format supported. */
  ret = v4l2_get_format (dev);
  if (ret < 0)
    goto err_free;

  /*
   * Set the desired image format.
   * Note: VIDIOC_S_FMT may change width and height.
   */
  ret = v4l2_set_format (dev, s_fmt);
  if (ret < 0)
    goto err_free;

  /* Get the changed image format. */
  ret = v4l2_get_format (dev);
  if (ret < 0)
    goto err_free;

  printf ("v4l2 open succeeded, file descriptor = %d\n", fd);

  *v4l2 = dev;

  return 0;

err_free:
  free (dev);
err:
  close (fd);

  return ret;
}

static void
v4l2_close (struct v4l2_device *dev)
{
  close (dev->v4l2_fd);
  free (dev);
}


/* ---------------------------------------------------------------------------
 * UVC generic stuff
 */

static int
uvc_video_set_format (struct uvc_device *dev)
{
  struct v4l2_format fmt;
  int ret;

  CLEAR (fmt);

  fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  fmt.fmt.pix.width = dev->width;
  fmt.fmt.pix.height = dev->height;
  fmt.fmt.pix.pixelformat = dev->fcc;

  fmt.fmt.pix.field = V4L2_FIELD_ANY;
  if (dev->fcc == V4L2_PIX_FMT_MJPEG)
    fmt.fmt.pix.sizeimage = dev->imgsize * 1;

  ret = ioctl (dev->uvc_fd, VIDIOC_S_FMT, &fmt);
  if (ret < 0) {
    printf ("UVC: Unable to set %c%c%c%c format %s (%d).\n", pixfmtstr (dev->fcc), strerror (errno), errno);
    return ret;
  }
  printf ("UVC: Setting format to: %c%c%c%c %ux%u\n", pixfmtstr (dev->fcc), dev->width, dev->height);

  return 0;
}

static int
uvc_video_stream (struct uvc_device *dev, int enable)
{
  int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  int ret;
  printf ("UVC: uvc_video_stream, enabled %d\n", enable);
  if (!enable) {
    ret = ioctl (dev->uvc_fd, VIDIOC_STREAMOFF, &type);
    if (ret < 0) {
      printf ("UVC: VIDIOC_STREAMOFF failed: %s (%d).\n",  strerror (errno), errno);
      return ret;
    }

    printf ("UVC: Stopping video stream.\n");

    return 0;
  }

  ret = ioctl (dev->uvc_fd, VIDIOC_STREAMON, &type);
  if (ret < 0) {
    printf ("UVC: Unable to start streaming %s (%d).\n",  strerror (errno), errno);
    return ret;
  }

  printf ("UVC: Starting video stream.\n");
  dev->uvc_streamon = 1;
  dev->uvc_shutdown_requested = 0;

  return 0;
}

/*static int
uvc_uninit_device (struct uvc_device *dev)
{
  //unsigned int i;
  //int ret;

  switch (dev->memtype) {

  case V4L2_MEMORY_USERPTR:
  default:
    break;
  }

  return 0;
}redundant */

static int
uvc_video_open (struct uvc_device **uvc, char *devname)
{
  struct uvc_device *dev;
  struct v4l2_capability cap;
  int fd;
  int ret = -EINVAL;

  fd = open (devname, O_RDWR | O_NONBLOCK);
  if (fd == -1) {
    printf ("UVC: device open failed: %s (%d).\n", strerror (errno), errno);
    return ret;
  }

  ret = ioctl (fd, VIDIOC_QUERYCAP, &cap);
  if (ret < 0) {
    printf ("UVC: unable to query uvc device: %s (%d)\n", strerror (errno), errno);
    goto err;
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_OUTPUT)) {
    printf ("UVC: %s is no video output device\n", devname);
    goto err;
  }

  dev = calloc (1, sizeof *dev);
  if (dev == NULL) {
    ret = -ENOMEM;
    goto err;
  }

  printf ("uvc device is %s on bus %s\n", cap.card, cap.bus_info);
  printf ("uvc open succeeded, file descriptor = %d\n", fd);

  dev->uvc_fd = fd;
  *uvc = dev;

  return 0;

err:
  close (fd);
  return ret;
}

static void
uvc_video_close (struct uvc_device *dev)
{
  close (dev->uvc_fd);
  free (dev->imgdata);
  free (dev);
}

/* ---------------------------------------------------------------------------
 * UVC streaming related 'u
 * I think this is called when the UVC buffer has been processed
 */

static int
uvc_video_process (struct uvc_device *dev)
{
  struct v4l2_buffer ubuf;
  //struct v4l2_buffer vbuf;
  static unsigned int i;
  int ret;
  /* 133ms per call 7.5fps!!

   */
  /*
   * Return immediately if UVC video output device has not started
   * streaming yet.
   */
  if (!dev->is_streaming) {
    return 0;
  }

  /* Prepare a v4l2 buffer to be dequeued from UVC domain. */
  CLEAR (ubuf);

  ubuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  ubuf.memory = V4L2_MEMORY_USERPTR;


  /* UVC - V4L2 integrated path. */

  /*
   * Return immediately if V4L2 video capture device has not
   * started streaming yet or if QBUF was not called even once on
   * the UVC side.
   */
  if (!dev->vdev->is_streaming || !dev->first_buffer_queued) {
    print ("UVC: not streaming\n");
    return 0;
  }

  //if(dev->vdev->dq_ubuf_ok==false){ return 0;} 
  time_me("uvc_video_process", dev->vdev);
  /*
   * Do not dequeue buffers from UVC side until there are atleast
   * 2 buffers available at UVC domain.
   */
  //print ("%d %d\n", dev->dqbuf_count,dev->qbuf_count);
  /*  if (!dev->uvc_shutdown_requested){
     if ((dev->dqbuf_count + 1 )  >= dev->qbuf_count){
     //print("UVC: not enough buffers ready\n");
     return 0;
     }
     } */
  if (!dev->uvc_streamon)
  {
    printf("uvc not streaming\n");
    return 0;
  }
  /* Dequeue the spent buffer from UVC domain */
  ret = ioctl (dev->uvc_fd, VIDIOC_DQBUF, &ubuf);
  if (ret < 0) {
    print ("UVC: unable to dequeue ubuf %s (%d)\n", strerror (errno), errno);
    return ret;
  }
  else {
    print ("UVC: dequeue ubuf okay\n");
    dev->vdev->dq_ubuf_ok = true;
  }

  dev->dqbuf_count++;

#ifdef ENABLE_BUFFER_DEBUG
  printf ("DeQueued buffer at UVC side=%d\n", ubuf.index);
#endif

  /*
   * If the dequeued buffer was marked with state ERROR by the
   * underlying UVC driver gadget, do not queue the same to V4l2
   * and wait for a STREAMOFF event on UVC side corresponding to
   * set_alt(0). So, now all buffers pending at UVC end will be
   * dequeued one-by-one and we will enter a state where we once
   * again wait for a set_alt(1) command from the USB host side.
   */

  if (ubuf.flags & V4L2_BUF_FLAG_ERROR) {
    dev->uvc_shutdown_requested = 1;
    printf ("UVC: Possible USB shutdown requested from "
	    "Host, seen during VIDIOC_DQBUF %d\n", ubuf.flags);
    return 0;
  }

  /* Queue the buffer to V4L2 domain */
  //ret = video_queue_buffer (dev->vdev, ubuf.index, BUFFER_FILL_NONE);
  /*CLEAR (vbuf);

     vbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
     vbuf.memory = V4L2_MEMORY_MMAP;
     vbuf.index = ubuf.index;

     ret = ioctl (dev->vdev->v4l2_fd, VIDIOC_QBUF, &vbuf); */
  /*  if (ret < 0) {
     printf ("V4L2: Unable to queue buffer: %s (%d).\n",
     strerror (errno), errno);
     return ret;
     }
     dev->vdev->qbuf_count++;
   */
/*
#ifdef ENABLE_BUFFER_DEBUG
  printf ("ReQueueing buffer at V4L2 side = %d\n", vbuf.index);
#endif
*/
  i++;
  return 0;
}



static int
uvc_video_reqbufs_userptr (struct uvc_device *dev, int nbufs)
{
  struct v4l2_requestbuffers rb;
  //unsigned int i, j, bpl, payload_size;
  int ret;

  CLEAR (rb);

  rb.count = nbufs;
  rb.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  rb.memory = V4L2_MEMORY_USERPTR;

  ret = ioctl (dev->uvc_fd, VIDIOC_REQBUFS, &rb);
  if (ret < 0) {
    if (ret == -EINVAL)
      printf ("UVC: does not support user pointer i/o\n");
    else
      printf ("UVC: VIDIOC_REQBUFS error %s (%d).\n",
	      strerror (errno), errno);
    goto err;
  }
  else {
    printf ("UVC uvc_video_reqbufs_userptr okay %c%c%c%c\n", pixfmtstr(dev->fcc));
  }

  if (!rb.count)
    return 0;

  dev->nbufs = rb.count;
  printf ("UVC: %u buffers allocated.\n", rb.count);

  return 0;

err:
  printf ("UVC uvc_video_reqbufs_userptr failed\n");
  return ret;

}


/*
 * This function is called in response to either:
 * 	- A SET_ALT(interface 1, alt setting 1) command from USB host,
 * 	  if the UVC gadget supports an ISOCHRONOUS video streaming endpoint
 * 	  or,
 *
 *	- A UVC_VS_COMMIT_CONTROL command from USB host, if the UVC gadget
 *	  supports a BULK type video streaming endpoint.
 */
static int
uvc_handle_streamon_event (struct uvc_device *dev)
{
  int ret;

  printf ("uvc_handle_streamon_event\n");
  /*if( V4L2_MEMORY_USERPTR == dev->vdev->memtype){
     print("dev->vdev->io io method userptr\n");
     } */
  ret = uvc_video_reqbufs_userptr (dev, dev->nbufs);
  if (ret < 0)
    goto err;


printf("finished uvc_video_reqbuf\n");
  ret = video_queue_all_buffers (dev->vdev);//, BUFFER_FILL_NONE);
  if (ret < 0)
    goto err;


  /* Start V4L2 capturing now. */
  //ret = v4l2_start_capturing (dev->vdev);
  ret = video_enable (dev->vdev, 1);
  if (ret < 0)
    goto err;

  dev->vdev->is_streaming = 1;
  
  //start mmal
  enable_mmal_input(dev->vdev);
  return 0;

err:
  printf ("uvc_handle_streamon_eventi error\n");
  return ret;
}



/* ---------------------------------------------------------------------------
 * UVC Request processing
 */

static void
uvc_fill_streaming_control (struct uvc_device *dev,
			    struct uvc_streaming_control *ctrl,
			    int iframe, int iformat)
{
  const struct uvc_format_info *format;
  const struct uvc_frame_info *frame;
  unsigned int nframes;

  if (iformat < 0)
    iformat = ARRAY_SIZE (uvc_formats) + iformat;
  if (iformat < 0 || iformat >= (int) ARRAY_SIZE (uvc_formats))
    return;
  format = &uvc_formats[iformat];

  nframes = 0;
  while (format->frames[nframes].width != 0)
    ++nframes;

  if (iframe < 0)
    iframe = nframes + iframe;
  if (iframe < 0 || iframe >= (int) nframes)
    return;
  frame = &format->frames[iframe];

  memset (ctrl, 0, sizeof *ctrl);

  ctrl->bmHint = 1;		//  1 keep dwFrameInterval constant fps const
  ctrl->bFormatIndex = iformat + 1;
  ctrl->bFrameIndex = iframe + 1;
  ctrl->dwFrameInterval = frame->intervals[0];
  //ctrl->wDelay = 10;
  //ctrl->dwClockFrequency=1;
  switch (format->fcc) {
  case V4L2_PIX_FMT_YUYV:
    ctrl->dwMaxVideoFrameSize = frame->width * frame->height * 2;
    break;
  case V4L2_PIX_FMT_MJPEG:
    ctrl->dwMaxVideoFrameSize = frame->width * frame->height * 1;	// dev->imgsize;
    break;
  }
  printf
    ("bmHint %d, bFormatIndex %d, bFrameIndex %d, dwFrameInterval %d, dwMaxVideoFramSize %d\n",
     ctrl->bmHint, ctrl->bFormatIndex, ctrl->bFrameIndex,
     ctrl->dwFrameInterval, ctrl->dwMaxVideoFrameSize);
  printf("%c%c%c%c\n", pixfmtstr(format->fcc));

  /* TODO: the UVC maxpayload transfer size should be filled
   * by the driver.
   */
  if (!dev->bulk) {
    printf ("maxpkt %d, mult %d, burst %d\n", dev->maxpkt, dev->mult, dev->burst);
    ctrl->dwMaxPayloadTransferSize =
      (dev->maxpkt) * (dev->mult + 1) * (dev->burst + 1);

  }
  else {
    ctrl->dwMaxPayloadTransferSize = ctrl->dwMaxVideoFrameSize;
  }
  ctrl->bmFramingInfo = 3;
  ctrl->bPreferedVersion = 1;
  ctrl->bMaxVersion = 1;
}

static void
uvc_events_process_standard (struct uvc_device *dev,
			     struct usb_ctrlrequest *ctrl,
			     struct uvc_request_data *resp)
{
  printf ("standard request\n");
  (void) dev;
  (void) ctrl;
  (void) resp;
}

static void
uvc_events_process_control (struct uvc_device *dev, uint8_t req,
			    uint8_t cs, uint8_t entity_id,
			    uint8_t len, struct uvc_request_data *resp)
{
  switch (entity_id) {
  case 0:
    switch (cs) {
    case UVC_VC_REQUEST_ERROR_CODE_CONTROL:
      /* Send the request error code last prepared. */
      resp->data[0] = dev->request_error_code.data[0];
      resp->length = dev->request_error_code.length;
      break;

    default:
      /*
       * If we were not supposed to handle this
       * 'cs', prepare an error code response.
       */
      dev->request_error_code.data[0] = 0x06;
      dev->request_error_code.length = 1;
      break;
    }
    break;

    /* Camera terminal unit 'UVC_VC_INPUT_TERMINAL'. */
  case 1:
    switch (cs) {
      /*
       * We support only 'UVC_CT_AE_MODE_CONTROL' for CAMERA
       * terminal, as our bmControls[0] = 2 for CT. Also we
       * support only auto exposure.
       */
    case UVC_CT_AE_MODE_CONTROL:
      switch (req) {
      case UVC_SET_CUR:
	printf ("UVC_CT_AE_MODE_CONTROL\n");
	/* Incase of auto exposure, attempts to
	 * programmatically set the auto-adjusted
	 * controls are ignored.
	 */
	resp->data[0] = 0x01;
	resp->length = 1;
	/*
	 * For every successfully handled control
	 * request set the request error code to no
	 * error.
	 */
	dev->request_error_code.data[0] = 0x00;
	dev->request_error_code.length = 1;
	break;

      case UVC_GET_INFO:
	/*
	 * TODO: We support Set and Get requests, but
	 * don't support async updates on an video
	 * status (interrupt) endpoint as of
	 * now.
	 */
	resp->data[0] = 0x03;
	resp->length = 1;
	/*
	 * For every successfully handled control
	 * request set the request error code to no
	 * error.
	 */
	dev->request_error_code.data[0] = 0x00;
	dev->request_error_code.length = 1;
	break;

      case UVC_GET_CUR:
      case UVC_GET_DEF:
      case UVC_GET_RES:
	/* Auto Mode â?? auto Exposure Time, auto Iris. */
	resp->data[0] = 0x02;
	resp->length = 1;
	/*
	 * For every successfully handled control
	 * request set the request error code to no
	 * error.
	 */
	dev->request_error_code.data[0] = 0x00;
	dev->request_error_code.length = 1;
	break;
      default:
	/*
	 * We don't support this control, so STALL the
	 * control ep.
	 */
	resp->length = -EL2HLT;
	/*
	 * For every unsupported control request
	 * set the request error code to appropriate
	 * value.
	 */
	dev->request_error_code.data[0] = 0x07;
	dev->request_error_code.length = 1;
	break;
      }
      break;

    default:
      /*
       * We don't support this control, so STALL the control
       * ep.
       */
      resp->length = -EL2HLT;
      /*
       * If we were not supposed to handle this
       * 'cs', prepare a Request Error Code response.
       */
      dev->request_error_code.data[0] = 0x06;
      dev->request_error_code.length = 1;
      break;
    }
    break;

    /* processing unit 'UVC_VC_PROCESSING_UNIT' */
  case 2:
    switch (cs) {
      /*
       * We support only 'UVC_PU_BRIGHTNESS_CONTROL' for Processing
       * Unit, as our bmControls[0] = 1 for PU.
       */
    case UVC_PU_BRIGHTNESS_CONTROL:
      switch (req) {
      case UVC_SET_CUR:
	printf ("UVC_SET_CUR BRIGHTNESS");
	resp->data[0] = 0x0;
	resp->length = len;
	/*
	 * For every successfully handled control
	 * request set the request error code to no
	 * error
	 */
	dev->request_error_code.data[0] = 0x00;
	dev->request_error_code.length = 1;
	break;
      case UVC_GET_MIN:
	printf ("UVC_GET_MIN BRIGHTNESS");
	resp->data[0] = PU_BRIGHTNESS_MIN_VAL;
	resp->length = 2;
	/*
	 * For every successfully handled control
	 * request set the request error code to no
	 * error
	 */
	dev->request_error_code.data[0] = 0x00;
	dev->request_error_code.length = 1;
	break;
      case UVC_GET_MAX:
	printf ("UVC_GET_MAX BR");
	resp->data[0] = PU_BRIGHTNESS_MAX_VAL;
	resp->length = 2;
	/*
	 * For every successfully handled control
	 * request set the request error code to no
	 * error
	 */
	dev->request_error_code.data[0] = 0x00;
	dev->request_error_code.length = 1;
	break;
      case UVC_GET_CUR:
	printf ("UVC_GET_CUR BR");
	resp->length = 2;
	memcpy (&resp->data[0], &dev->brightness_val, resp->length);
	/*
	 * For every successfully handled control
	 * request set the request error code to no
	 * error
	 */
	dev->request_error_code.data[0] = 0x00;
	dev->request_error_code.length = 1;
	break;
      case UVC_GET_INFO:
	printf ("UVC_GET_INFO BR");
	/*
	 * We support Set and Get requests and don't
	 * support async updates on an interrupt endpt
	 */
	resp->data[0] = 0x03;
	resp->length = 1;
	/*
	 * For every successfully handled control
	 * request, set the request error code to no
	 * error.
	 */
	dev->request_error_code.data[0] = 0x00;
	dev->request_error_code.length = 1;
	break;
      case UVC_GET_DEF:
	printf ("UVC_GET_DEF BR");
	resp->data[0] = PU_BRIGHTNESS_DEFAULT_VAL;
	resp->length = 2;
	/*
	 * For every successfully handled control
	 * request, set the request error code to no
	 * error.
	 */
	dev->request_error_code.data[0] = 0x00;
	dev->request_error_code.length = 1;
	break;
      case UVC_GET_RES:
	printf ("UVC_GET_RES BR");
	resp->data[0] = PU_BRIGHTNESS_STEP_SIZE;
	resp->length = 2;
	/*
	 * For every successfully handled control
	 * request, set the request error code to no
	 * error.
	 */
	dev->request_error_code.data[0] = 0x00;
	dev->request_error_code.length = 1;
	break;
      default:
	printf ("brightness unknown\n");
	/*
	 * We don't support this control, so STALL the
	 * default control ep.
	 */
	resp->length = -EL2HLT;
	/*
	 * For every unsupported control request
	 * set the request error code to appropriate
	 * code.
	 */
	dev->request_error_code.data[0] = 0x07;
	dev->request_error_code.length = 1;
	break;
      }
      break;

    default:
      /*
       * We don't support this control, so STALL the control
       * ep.
       */
      resp->length = -EL2HLT;
      /*
       * If we were not supposed to handle this
       * 'cs', prepare a Request Error Code response.
       */
      dev->request_error_code.data[0] = 0x06;
      dev->request_error_code.length = 1;
      break;
    }

    break;

  default:
    /*
     * If we were not supposed to handle this
     * 'cs', prepare a Request Error Code response.
     */
    dev->request_error_code.data[0] = 0x06;
    dev->request_error_code.length = 1;
    break;

  }

  printf ("control request (req %02x cs %02x)\n", req, cs);
}

static void
uvc_events_process_streaming (struct uvc_device *dev, uint8_t req, uint8_t cs,
			      struct uvc_request_data *resp)
{
  struct uvc_streaming_control *ctrl;

  printf ("streaming request (req %02x cs %02x)\n", req, cs);

  if (cs != UVC_VS_PROBE_CONTROL && cs != UVC_VS_COMMIT_CONTROL) {
    printf ("cs not UVC_VS_PROBE_CONTROL or UVC_VS_COMMIT_CONTROL %d\n", cs);
    return;
  }

  ctrl = (struct uvc_streaming_control *) &resp->data;
  resp->length = sizeof *ctrl;

  switch (req) {
  case UVC_SET_CUR:
    dev->control = cs;
    resp->length = 34;
    break;

  case UVC_GET_CUR:
    if (cs == UVC_VS_PROBE_CONTROL)
      memcpy (ctrl, &dev->probe, sizeof *ctrl);
    else
      memcpy (ctrl, &dev->commit, sizeof *ctrl);
    break;

  case UVC_GET_MIN:
  case UVC_GET_MAX:
  case UVC_GET_DEF:
    uvc_fill_streaming_control (dev, ctrl, req == UVC_GET_MAX ? -1 : 0,	req == UVC_GET_MAX ? -1 : 0);
    break;

  case UVC_GET_RES:
    CLEAR (ctrl);
    break;

  case UVC_GET_LEN:
    resp->data[0] = 0x00;
    resp->data[1] = 0x22;
    resp->length = 2;
    break;

  case UVC_GET_INFO:
    resp->data[0] = 0x03;
    resp->length = 1;
    break;
  default:
    printf ("unhandled streaming request (req %02x cs %02x)\n", req, cs);
    break;
  }
}

static void
uvc_events_process_class (struct uvc_device *dev,
			  struct usb_ctrlrequest *ctrl,
			  struct uvc_request_data *resp)
{
  if ((ctrl->bRequestType & USB_RECIP_MASK) != USB_RECIP_INTERFACE)
    return;

  switch (ctrl->wIndex & 0xff) {
  case UVC_INTF_CONTROL:
    uvc_events_process_control (dev, ctrl->bRequest,
				ctrl->wValue >> 8,
				ctrl->wIndex >> 8, ctrl->wLength, resp);
    break;

  case UVC_INTF_STREAMING:
    uvc_events_process_streaming (dev, ctrl->bRequest,
				  ctrl->wValue >> 8, resp);
    break;

  default:
    break;
  }
}

static void
uvc_events_process_setup (struct uvc_device *dev,
			  struct usb_ctrlrequest *ctrl,
			  struct uvc_request_data *resp)
{
  dev->control = 0;

#ifdef ENABLE_USB_REQUEST_DEBUG
  printf ("\nbRequestType %02x bRequest %02x wValue %04x wIndex %04x "
	  "wLength %04x\n", ctrl->bRequestType, ctrl->bRequest,
	  ctrl->wValue, ctrl->wIndex, ctrl->wLength);
#endif

  switch (ctrl->bRequestType & USB_TYPE_MASK) {
  case USB_TYPE_STANDARD:
    uvc_events_process_standard (dev, ctrl, resp);
    break;

  case USB_TYPE_CLASS:
    uvc_events_process_class (dev, ctrl, resp);
    break;

  default:
    break;
  }
}

static int
uvc_events_process_control_data (struct uvc_device *dev,
                                 uint8_t cs, uint8_t entity_id,
                                 struct uvc_request_data *data)
{
  switch (entity_id) {
      /* Processing unit 'UVC_VC_PROCESSING_UNIT'. */
    case 2:
      switch (cs) {
          /*
           * We support only 'UVC_PU_BRIGHTNESS_CONTROL' for Processing Unit, as our bmControls[0] = 1 for PU.
           */
        case UVC_PU_BRIGHTNESS_CONTROL:
          memcpy (&dev->brightness_val, data->data, data->length);
          /* UVC - V4L2 integrated path. */
          if (0) {
            /*
             * Try to change the Brightness attribute onVideo capture device. Note that this try may
             * succeed or end up with some error on the	video capture side. By default to keep tools
             * like USBCV's UVC test suite happy, we are maintaining a local copy of the current
             * brightness value in 'dev->brightness_val' variable and we return the same value to the
             * Host on receiving a GET_CUR(BRIGHTNESS) control request.
             *
             * FIXME: Keeping in view the point discussed	above, notice that we ignore the return value
             * from the function call below. To be strictly compliant, we should return the same value
             * accordingly.
             */
            v4l2_set_ctrl (dev->vdev, dev->brightness_val, V4L2_CID_BRIGHTNESS);
          }
          break;
          
        default:
          break;
      }
      break;
      
    default:
      break;
  }
  
  printf ("Control Request data phase (cs %02x entity %02x)\n",  cs, entity_id);
  
  return 0;
}

static int
uvc_events_process_data (struct uvc_device *dev, struct uvc_request_data *data)
{
  struct uvc_streaming_control *target;
  struct uvc_streaming_control *ctrl;
  struct v4l2_format fmt;
  const struct uvc_format_info *format;
  const struct uvc_frame_info *frame;
  const unsigned int *interval;
  unsigned int iformat, iframe;
  unsigned int nframes;
  unsigned int *val = (unsigned int *) data->data;
  int ret;

  switch (dev->control) {
  case UVC_VS_PROBE_CONTROL:
    printf ("setting probe control, length = %d\n", data->length);
    target = &dev->probe;
    break;

  case UVC_VS_COMMIT_CONTROL:
    printf ("setting commit control, length = %d\n", data->length);
    target = &dev->commit;
    break;

  default:
    printf ("setting other control, length = %d\n", data->length);

    /*
     * As we support only BRIGHTNESS control, this request is
     * for setting BRIGHTNESS control.
     * Check for any invalid SET_CUR(BRIGHTNESS) requests
     * from Host. Note that we support Brightness levels
     * from 0x0 to 0x10 in a step of 0x1. So, any request
     * with value greater than 0x10 is invalid.
     */
    if (*val > PU_BRIGHTNESS_MAX_VAL) {
      return -EINVAL;
    }
    else {
      ret = uvc_events_process_control_data (dev, UVC_PU_BRIGHTNESS_CONTROL, 2, data);
      if (ret < 0)
	goto err;

      return 0;
    }
  }

  ctrl = (struct uvc_streaming_control *) &data->data;
  iformat = clamp ((unsigned int) ctrl->bFormatIndex, 1U,
		   (unsigned int) ARRAY_SIZE (uvc_formats));
  format = &uvc_formats[iformat - 1];

  nframes = 0;
  while (format->frames[nframes].width != 0)
    ++nframes;

  iframe = clamp ((unsigned int) ctrl->bFrameIndex, 1U, nframes);
  frame = &format->frames[iframe - 1];
  interval = frame->intervals;

  while (interval[0] < ctrl->dwFrameInterval && interval[1])
    ++interval;

  target->bFormatIndex = iformat;
  target->bFrameIndex = iframe;

  switch (format->fcc) {
  case V4L2_PIX_FMT_YUYV:
    target->dwMaxVideoFrameSize = frame->width * frame->height * 2;
    // uses frame size and then dev->width & height get changed
    break;
  case V4L2_PIX_FMT_MJPEG:
    target->dwMaxVideoFrameSize = frame->width * frame->height * 1;
    break;
  }
  printf ("UVC: event proc %c%c%c%c %dx%d\n", pixfmtstr (format->fcc), frame->width, frame->height);
  target->dwFrameInterval = *interval;

  if (dev->control == UVC_VS_COMMIT_CONTROL) {
    dev->fcc = format->fcc;
    dev->width = frame->width;
    dev->height = frame->height;
    printf("UVC: event set video format\n");
    
    ret = uvc_video_set_format (dev);
    if (ret < 0)
      goto err;

  }
  printf
    ("bmHint %d, bFormatIndex %d, bFrameIndex %d, dwFrameInterval %d, dwMaxVideoFramSize %d\n",
     ctrl->bmHint, ctrl->bFormatIndex, ctrl->bFrameIndex,
     ctrl->dwFrameInterval, ctrl->dwMaxVideoFrameSize);
  return 0;

err:
  return ret;
}

static void
uvc_events_process (struct uvc_device *dev)
{
  struct v4l2_event v4l2_event;
  struct uvc_event *uvc_event = (void *) &v4l2_event.u.data;
  struct uvc_request_data resp;
  int ret;

  ret = ioctl (dev->uvc_fd, VIDIOC_DQEVENT, &v4l2_event);
  if (ret < 0) {
    printf ("VIDIOC_DQEVENT failed: %s (%d)\n", strerror (errno), errno);
    return;
  }

  memset (&resp, 0, sizeof resp);
  resp.length = -EL2HLT;

  switch (v4l2_event.type) {
  case UVC_EVENT_CONNECT:
    printf ("UVC_EVENT_CONNECT\n");
    return;

  case UVC_EVENT_DISCONNECT:
    dev->uvc_shutdown_requested = 1;
    printf ("UVC: Possible USB shutdown requested from "
	    "Host, seen via UVC_EVENT_DISCONNECT\n");
    return;

  case UVC_EVENT_SETUP:
    printf ("UVC_EVENT_SETUP\n");
    uvc_events_process_setup (dev, &uvc_event->req, &resp);
    break;

  case UVC_EVENT_DATA:
    printf ("UVC_EVENT_DATA\n");
    ret = uvc_events_process_data (dev, &uvc_event->data);
    if (ret < 0)
      break;

    return;

  case UVC_EVENT_STREAMON:
    printf ("UVC_EVENT_STREAMON \n");
    uvc_handle_streamon_event (dev);
    return;

  case UVC_EVENT_STREAMOFF:
    printf ("UVC_EVENT_STREAMOFF\n");
    /* Stop V4L2 streaming... */
    if (dev->vdev->is_streaming) {
      /* UVC - V4L2 integrated path. */
      //v4l2_stop_capturing (dev->vdev);
      video_enable (dev->vdev, 0);
      dev->vdev->is_streaming = 0;
    }

    /* ... and now UVC streaming.. */
    if (dev->is_streaming) {
      uvc_video_stream (dev, 0);
      //uvc_uninit_device (dev);
      uvc_video_reqbufs_userptr (dev, 0);
      dev->is_streaming = 0;
      dev->first_buffer_queued = 0;
    }

    return;

  default:
    printf ("hey unknown event %d\n", v4l2_event.type);
    break;
  }

  ret = ioctl (dev->uvc_fd, UVCIOC_SEND_RESPONSE, &resp);
  if (ret < 0) {
    printf ("UVCIOC_S_EVENT failed: %s (%d)\n", strerror (errno), errno);
    return;
  }
}

static void
uvc_events_init (struct uvc_device *dev)
{
  struct v4l2_event_subscription sub;
  unsigned int payload_size;

  switch (dev->fcc) {
 
  case V4L2_PIX_FMT_YUYV:
    payload_size = dev->width * dev->height * 2;
    break;
  case V4L2_PIX_FMT_MJPEG:
    payload_size = dev->imgsize;
    break;
  }

  printf ("UVC: events init udev->width %d, udev->height %d %d \n", dev->width, dev->height, dev->probe.dwMaxPayloadTransferSize);
  uvc_fill_streaming_control (dev, &dev->probe, 0, 0);
  uvc_fill_streaming_control (dev, &dev->commit, 0, 0);

  /*if (dev->bulk)
     FIXME Crude hack, must be negotiated with the driver. 
     dev->probe.dwMaxPayloadTransferSize =
     dev->commit.dwMaxPayloadTransferSize = payload_size; */

  memset (&sub, 0, sizeof sub);
  sub.type = UVC_EVENT_SETUP;

  ioctl (dev->uvc_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
  sub.type = UVC_EVENT_DATA;
  ioctl (dev->uvc_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
  sub.type = UVC_EVENT_STREAMON;
  ioctl (dev->uvc_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
  sub.type = UVC_EVENT_STREAMOFF;
  ioctl (dev->uvc_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
}

static void
image_load(struct uvc_device *dev, const char *img)
{
  int fd = -1;
  
  if (img == NULL)
    return;
  
  fd = open(img, O_RDONLY);
  if (fd == -1) {
    printf("Unable to open MJPEG image '%s'\n", img);
    return;
  }
  
  dev->imgsize = lseek(fd, 0, SEEK_END);
  lseek(fd, 0, SEEK_SET);
  dev->imgdata = malloc(dev->imgsize);
  if (dev->imgdata == NULL) {
    printf("Unable to allocate memory for MJPEG image\n");
    dev->imgsize = 0;
    return;
  }
  
  read(fd, dev->imgdata, dev->imgsize);
  close(fd);
}


/* ---------------------------------------------------------------------------
 * process thread  's   
 */
static void *
save_thread (void *arg)
{
  struct v4l2_device *dev = (struct v4l2_device *) arg;
  MMAL_BUFFER_HEADER_T *buffer;
  MMAL_STATUS_T status;
  struct v4l2_buffer ubuf;
  unsigned int bytes_written;
  static int i = 0;
  int ret = 0;
  unsigned char *temp;
  
  //Being lazy and using a timed wait instead of setting up a
  //mechanism for skipping this when destroying the thread
  printf("save_thread started\n");
  /*char *filename;
  
  if(default_format){
    filename="singleframe.raw";
   }
  else{
    filename="singleframe.mjpg";
    }
    
  int fd = open(filename, O_RDONLY);
  bytes_written = lseek(fd, 0, SEEK_END);
  temp = malloc(bytes_written);
  read(fd, temp, bytes_written);
  printf("file %s is %d bytes\n", filename, bytes_written);
  close(fd);*/
    
  CLEAR (ubuf);
  
  ubuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  ubuf.memory = V4L2_MEMORY_USERPTR;
    
  while (!dev->thread_quit) {
    buffer = mmal_queue_timedwait (dev->save_queue, 500);
    if (!buffer)
      continue;

    print("save_thread %p, len %d, size %d \n", buffer,  buffer->length, buffer->alloc_size);
  
   /* if (dev->mjpeg_fd) {
      bytes_written = fwrite (buffer->data, 1, buffer->length, dev->mjpeg_fd);
      fflush (dev->mjpeg_fd);
      if (bytes_written != buffer->length) {
        print ("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
      }
    }*/

    ubuf.m.userptr = (unsigned long) (buffer->data);
    ubuf.length = dev->udev->imgsize;//buffer->length;	// buffer->alloc_size;;bytes_written;//
    ubuf.index = dev->vbuf.index;;	//vbuf.index;L
    ubuf.bytesused = buffer->length;// buffer->alloc_size;	//;bytes_written;//
    
    ubuf.timestamp.tv_sec = dev->lastpts;//buffer->pts;
    ubuf.flags = 0;
    
    /*    ubuf.m.userptr = (unsigned long)temp;
     ubuf.length = buffer->alloc_size;
     ubuf.index = 0;
     */
    //  if(dev->dq_ubuf_ok==false){
    ret = ioctl (dev->udev->uvc_fd, VIDIOC_QBUF, &ubuf);
    if (ret < 0) {
      if(i%20==0) print ("UVC: Unable to queue buffer %d: %s (%d) %d %d.\n", dev->vbuf.index, strerror (errno), errno, i, dev->udev->uvc_fd);
      // Check for a USB disconnect/shutdown event. 0
      if (errno == ENODEV) {
        //dev->udev->uvc_shutdown_requested = 1;
        printf ("UVC: Possible USB shutdown requested from "
                "Host, seen during VIDIOC_QBUF\n");
        return 0;
      }
    }
    else {
      dev->udev->qbuf_count++;
      print
      ("UVC: queueued buffer %d ok q %lld dq %lld, vbuf %lld %lld %d\n",
       ubuf.index, dev->udev->qbuf_count, dev->udev->dqbuf_count,
       dev->qbuf_count, dev->dqbuf_count, i);
      //mmal_buffer_header_mem_unlock(buffer);
      //#ifdef ENABLE_BUFFER_DEBUG
      // printf ("Queueing buffer at UVC side = %d\n", ubuf.index);
      //#endif
    }
    
    if (!dev->udev->first_buffer_queued ) {
      printf ("UVC: udev first buffers start\n");
      uvc_video_stream (dev->udev, 1);
      dev->udev->first_buffer_queued = 1;
      dev->udev->is_streaming = 1;
    }
    
    buffer->length = 0;		// what is this for?
    if (default_format == 0)
      status = mmal_port_send_buffer (dev->isp->output[0], buffer);
    else
      status = mmal_port_send_buffer (dev->encoder->output[0], buffer);
    
    if (status != MMAL_SUCCESS) {
      print ("mmal_port_send_buffer failed on buffer %p, status %d", buffer, status);
    }
    i++;
    //   dev->dq_ubuf_ok = true; //dq_ubuf can be called since a buffer has been queued
  }
  return NULL;
}


static void
buffers_to_isp (struct v4l2_device *dev)
{
  MMAL_BUFFER_HEADER_T *buffer;
  print("buffers_to_isp\n");
  while ((buffer = mmal_queue_get (dev->isp_output_pool->queue)) != NULL) {
    mmal_port_send_buffer (dev->isp->output[0], buffer);
  }

}


/* ---------------------------------------------------------------------------
 * isp_input_callback
 */
static void
isp_input_callback (MMAL_PORT_T * port, MMAL_BUFFER_HEADER_T * buffer)
{
  struct v4l2_device *dev = (struct v4l2_device *) port->userdata;
  unsigned int i;
  print("isp_input_callback %p from %s, len %d, size %d \n", buffer,  port->name,  buffer->length, buffer->alloc_size);
  for (i = 0; i < dev->nbufs; i++) {
    if (dev->buffers[i].mmal == buffer) {
      //v4l2 buffer finished using so qback
      print ("Matches V4L2 buffer index %d / %d\n", i, dev->buffers[i].idx);
      video_queue_buffer (dev, dev->buffers[i].idx);//, BUFFER_FILL_NONE);
      mmal_buffer_header_release (buffer);
      buffer = NULL;
      break;
    }
  }
  if (buffer) {
    print ("Failed to find matching V4L2 buffer for mmal buffer %p\n", buffer);
    mmal_buffer_header_release (buffer);
  }

}


static void isp_output_callback (MMAL_PORT_T * port, MMAL_BUFFER_HEADER_T * buffer)
{
  print("isp_output_callback %p from %s, len %d, size %d \n", buffer,  port->name,  buffer->length, buffer->alloc_size);
  //vcos_log_error("File handle: %p", port->userdata);
  struct v4l2_device *dev = (struct v4l2_device *) port->userdata;
  
  if (dev->render) {
    MMAL_BUFFER_HEADER_T *out = mmal_queue_get (dev->render_pool->queue);
    if (out) {
      mmal_buffer_header_replicate (out, buffer);
      mmal_port_send_buffer (dev->render->input[0], out);
    }
  }
  if (dev->encoder) {
    MMAL_BUFFER_HEADER_T *out = mmal_queue_get (dev->encode_pool->queue);
    if (out) {
      mmal_buffer_header_replicate (out, buffer);
      mmal_port_send_buffer (dev->encoder->input[0], out);
    }
  }
  mmal_buffer_header_release (buffer);

  buffers_to_isp (dev);
}


static void render_encoder_input_callback (MMAL_PORT_T * port, MMAL_BUFFER_HEADER_T * buffer)
{
  print("render_encoder_input_callback %p from %s, len %d, size %d \n", buffer,  port->name,  buffer->length, buffer->alloc_size);
  //vcos_log_error("File handle: %p", port->userdata);
  struct v4l2_device *dev = (struct v4l2_device *) port->userdata;
  mmal_buffer_header_release (buffer);
  buffers_to_isp (dev);
}


/* ---------------------------------------------------------------------------
 * isp_output_callback 'i
 */

static void
encoder_buffer_cb (MMAL_PORT_T * port, MMAL_BUFFER_HEADER_T * buffer)
{
  static int i = 0;
  MMAL_STATUS_T status;
  print("encoder_buffer_cb %p from %s, len %d, size %d \n", buffer,  port->name,  buffer->length, buffer->alloc_size);
  //printf("%d\n",i);
  //vcos_log_error("File handle: %p", port->userdata);
  struct v4l2_device *dev = (struct v4l2_device *) port->userdata;

  if(1)
  {
    //printf ("encoder_buffer_cb %p, %p, %d %d %d %d\n", buffer, buffer->data, buffer->alloc_size, dev->vbuf.index, buffer->length, port->is_enabled);
    //time_me("isp",dev);
    if (port->is_enabled) {
      dev->counter++;
      mmal_queue_put (dev->save_queue, buffer);	//if comment data won't be saved to disk
    }
    else {
      print ("port disabled\n");
      mmal_buffer_header_release (buffer);
    }
  }
  else {
    //skipping buffer data so can just send to output isp
    if (default_format == 0)
      status = mmal_port_send_buffer (dev->isp->output[0], buffer);
    else
      status = mmal_port_send_buffer (dev->encoder->output[0], buffer);
    
    if (status != MMAL_SUCCESS) {
      print ("Failed to send buffer isp\n");
    }
  }
  //mmal_buffer_header_mem_unlock(buffer);
  i++;
}

/* ---------------------------------------------------------------------------
 * isp->input enable, wait until after buffers have been allocated
 */
int
enable_mmal_input (struct v4l2_device *dev)
{
  MMAL_STATUS_T status;
  MMAL_POOL_T *pool;
  MMAL_PORT_T *port_output;
  unsigned int i;

  printf("enable_mmal_input\n");
  status = mmal_port_parameter_set_boolean (dev->isp->input[0], MMAL_PARAMETER_ZERO_COPY, dev->can_zero_copy);
  if (status != MMAL_SUCCESS) {
    print ("Failed to set zero copy\n");
    return -1;
  }
  status = mmal_port_enable (dev->isp->input[0], isp_input_callback);
  if (status != MMAL_SUCCESS) {
    print ("ISP input enable failed\n");
    return -1;
  }
  // start encoding by passing empty buffers to ouput port of isp or encoder
  if(default_format==0){
    pool = dev->isp_output_pool;
    port_output = dev->isp->output[0];
  }
  else
  {
    pool = dev->output_pool;
    port_output = dev->encoder->output[0];
  }
  
  for (i = 0; i < port_output->buffer_num; i++) {
    MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get (pool->queue);
    if (!buffer) {
      print ("buffer issue\n");
      return -1;
    }
    status = mmal_port_send_buffer ( port_output, buffer);
    if (status != MMAL_SUCCESS) {
      print ("mmal_port_send_buffer failed on buffer %p, status %d\n", buffer, status);
      return -1;
    }
  }
  return 0;
}


// setup_mmal mjpeg and yuyv
static int
setup_mmal_comp (struct v4l2_device *dev, int nbufs)
{
  MMAL_STATUS_T status;
  VCOS_STATUS_T vcos_status;
  MMAL_PORT_T *port;
  const struct v4l2_format_info *info;
  struct v4l2_format fmt;
  int ret;
  MMAL_PORT_T *isp_output, *encoder_input = NULL, *encoder_output = NULL;

  status = mmal_component_create ("vc.ril.isp", &dev->isp);
  if (status != MMAL_SUCCESS) {
    print ("Failed to create isp\n");
    return -1;
  }

  if (default_format) {
    status = mmal_component_create ("vc.ril.video_encode", &dev->encoder);
    if (status != MMAL_SUCCESS) {
      print ("Failed to create encoder");
      return -1;
    }
    encoder_input = dev->encoder->input[0];
    encoder_output = dev->encoder->output[0];
  }

  if (0) { //render to HDMI disabled
    status = mmal_component_create ("vc.ril.video_render", &dev->render);
    if (status != MMAL_SUCCESS) {
      print ("Failed to create render\n");
      return -1;
    }
  }

  port = dev->isp->input[0];

  CLEAR(fmt);
  fmt.type = dev->type;
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  dev->memtype = V4L2_MEMORY_MMAP;

  ret = ioctl (dev->v4l2_fd, VIDIOC_G_FMT, &fmt);
  if (ret < 0) {
    print ("Unable to get format: %s (%d).\n", strerror (errno), errno);
    return ret;
  }

  info = v4l2_format_by_fourcc (fmt.fmt.pix.pixelformat);
  if (!info || info->mmal_encoding == MMAL_ENCODING_UNUSED) {
    print ("Unsupported encoding\n");
    return -1;
  }
  
  printf("input size %dx%d field %s, %c%c%c%c\n", 
    fmt.fmt.pix.width, fmt.fmt.pix.height, v4l2_field_name(fmt.fmt.pix.field), pixfmtstr(fmt.fmt.pix.pixelformat));
  port->format->encoding = info->mmal_encoding;
  port->format->es->video.crop.width = fmt.fmt.pix.width;
  port->format->es->video.crop.height = fmt.fmt.pix.height;
  port->format->es->video.width = (port->format->es->video.crop.width + 31) & ~31;
  //mmal_encoding_stride_to_width(port->format->encoding, fmt.fmt.pix.bytesperline);
  /* FIXME - buffer may not be aligned vertically */
  port->format->es->video.height = (fmt.fmt.pix.height + 15) & ~15;
  //Ignore for now, but will be wanted for video encode.
  port->format->es->video.frame_rate.num = 10000;
  port->format->es->video.frame_rate.den = 10000;
  port->buffer_num = nbufs;
  
  //cropping
  if (0) {
    MMAL_PARAMETER_CROP_T crop ={ {MMAL_PARAMETER_CROP, sizeof (MMAL_PARAMETER_CROP_T)}, {0, 0, 0, 0}};
    crop.rect.x = 32;
    crop.rect.y = 32;
    crop.rect.width = 640;
    crop.rect.height = 360;
    mmal_port_parameter_set (port, &crop.hdr);
  }
  
  if (dev->fps) {
    dev->frame_time_usec = 1000000 / dev->fps;
  }

  status = mmal_port_format_commit (port);
  if (status != MMAL_SUCCESS) {
    return -1;
  }
  mmal_log_dump_port (port);

  unsigned int mmal_stride = mmal_encoding_width_to_stride (info->mmal_encoding, port->format->es->video.width);
  if (mmal_stride != fmt.fmt.pix.bytesperline) {
    printf("stride adjustment\n");
    if (video_set_format
        (dev, fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.pixelformat,
         mmal_stride, fmt.fmt.pix.sizeimage, fmt.fmt.pix.field, fmt.fmt.pix.flags) < 0)
      print ("Failed to adjust stride\n");
    else
      // Retrieve settings again so local state is correct
      video_get_format (dev);
  }

  dev->mmal_pool = mmal_pool_create (nbufs, 0);
  if (!dev->mmal_pool) {
    print ("Failed to create pool\n");
    return -1;
  }
  print ("Created pool of length %d, size %d\n", nbufs, 0);

  port->userdata = (struct MMAL_PORT_USERDATA_T *) dev;

  mmal_format_copy (dev->isp->output[0]->format, port->format);

  isp_output = dev->isp->output[0];

  if (default_format == 0){
    isp_output->format->encoding = MMAL_ENCODING_YUYV; // -f0
    isp_output->buffer_num = 8;
  }
  else{
    isp_output->format->encoding = MMAL_ENCODING_I420; // -f1
    isp_output->buffer_num = 3;
  }
  
  isp_output->format->es->video.width = VCOS_ALIGN_UP (WIDTH, 32);
  isp_output->format->es->video.height = VCOS_ALIGN_UP (HEIGHT, 16);
  
  isp_output->format->es->video.crop.width = WIDTH;	//set output size
  isp_output->format->es->video.crop.height = HEIGHT;
  
  while (isp_output->format->es->video.crop.width > 1920) {
    isp_output->format->es->video.crop.width >>= 1;
    isp_output->format->es->video.crop.height >>= 1;
  }
  //      port->format->es->video.crop.width = 640; //set output size
  //      port->format->es->video.crop.height = 360;
  isp_output->userdata = (struct MMAL_PORT_USERDATA_T *) dev;

  status = mmal_port_format_commit (isp_output);
  if (status != MMAL_SUCCESS) {
    print ("ISP o/p commit failed\n");
    return -1;
  }
   
  if (dev->render) {
    status = mmal_format_full_copy (dev->render->input[0]->format, isp_output->format);
    dev->render->input[0]->buffer_num = 3;
    if (status == MMAL_SUCCESS)
      status = mmal_port_format_commit (dev->render->input[0]);
  }

  //  Encoder setup
  if (dev->encoder) {
    status = mmal_format_full_copy (encoder_input->format, isp_output->format);
    encoder_input->buffer_num = 1;
    if (status == MMAL_SUCCESS)
      status = mmal_port_format_commit (encoder_input);

    // Only supporting H264 at the moment
    encoder_output->format->encoding = MMAL_ENCODING_MJPEG;
    //encoder_output->format->encoding = MMAL_ENCODING_H264;
    encoder_output->format->es->video.width = VCOS_ALIGN_UP (WIDTH, 32);
    encoder_output->format->es->video.height = VCOS_ALIGN_UP (HEIGHT, 16);

    encoder_output->format->es->video.crop.width = WIDTH;	//set output size
    encoder_output->format->es->video.crop.height = HEIGHT;

    encoder_output->format->bitrate = dev->bitrate;//1500000;	//-b?

    encoder_output->buffer_size = 256 << 10;//encoder_output->buffer_size_recommended; buffer->alloc_size

    if (encoder_output->buffer_size < encoder_output->buffer_size_min)
      encoder_output->buffer_size = encoder_output->buffer_size_min;

    encoder_output->buffer_num = 8;	//encoder_output->buffer_num_recommended;

    if (encoder_output->buffer_num < encoder_output->buffer_num_min)
      encoder_output->buffer_num = encoder_output->buffer_num_min;

    // We need to set the frame rate on output to 0, to ensure it gets
    // updated correctly from the input framerate when port connected
    encoder_output->format->es->video.frame_rate.num = 0;
    encoder_output->format->es->video.frame_rate.den = 1;

    // Commit the port changes to the output port
    status = mmal_port_format_commit (encoder_output);

    if (status != MMAL_SUCCESS) {
      print ("Unable to set format on encoder output port %d\n", dev->dst_mmal_enc);
      return -1;
    }
    {//bunch of H264 stuff that could be removed
      if(0){
        MMAL_PARAMETER_VIDEO_PROFILE_T param;
        param.hdr.id = MMAL_PARAMETER_PROFILE;
        param.hdr.size = sizeof (param);
        param.profile[0].profile = MMAL_VIDEO_PROFILE_H264_HIGH;  //state->profile;
        param.profile[0].level = MMAL_VIDEO_LEVEL_H264_4;
        status = mmal_port_parameter_set (encoder_output, &param.hdr);
        if (status != MMAL_SUCCESS) {
          print ("Unable to set H264 profile\n");
        }
      }
      
      status =mmal_port_parameter_set_boolean (encoder_input, MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, 1);
      if ( status != MMAL_SUCCESS) {
        print ("Unable to set immutable input flag\n");// Continue rather than abort..
      }
      
      //set INLINE HEADER flag to generate SPS and PPS for every IDR if requested
      status =mmal_port_parameter_set_boolean (encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, 0);
      if ( status != MMAL_SUCCESS) {
        print ("failed to set INLINE HEADER FLAG parameters\n");// Continue rather than abort..
      }
      
      //set INLINE VECTORS flag to request motion vector estimates H264
      status =mmal_port_parameter_set_boolean (encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_VECTORS, 0) ;
      if ( status != MMAL_SUCCESS) {
        print ("failed to set INLINE VECTORS parameters\n"); // Continue rather than abort..
      }
    }
    
    if (status != MMAL_SUCCESS) {
      print ("Unable to set format on video encoder input port\n");
    }
    
    print ("Enable encoder....\n");
    status = mmal_component_enable (dev->encoder);
    if (status != MMAL_SUCCESS) {
      print ("Failed to enable\n");
      return -1;
    }
    //status = mmal_port_parameter_set_boolean (encoder_output, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);//disabled
    status = mmal_port_parameter_set_boolean (encoder_input,  MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
    if (status != MMAL_SUCCESS) {
      print ("Failed to set zero copy\n");
      return -1;
    }
    encoder_input->userdata = (struct MMAL_PORT_USERDATA_T *) dev;
  }

  if(default_format==1){
    //only for MJPEG
    status = mmal_port_parameter_set_boolean (isp_output, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
  }
  

  if (dev->render) {
    status += mmal_port_parameter_set_boolean (dev->render->input[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
    if (status != MMAL_SUCCESS) {
      return -1;
    }

    dev->render->input[0]->userdata = (struct MMAL_PORT_USERDATA_T *) dev;

    status = mmal_port_enable (dev->render->input[0], render_encoder_input_callback);
    if (status != MMAL_SUCCESS)
      return -1;

    print ("Create pool of %d buffers of size %d for render\n",
        dev->render->input[0]->buffer_num, dev->render->input[0]->buffer_size);
    
    dev->render_pool = mmal_port_pool_create (dev->render->input[0], 
        dev->render->input[0]->buffer_num, dev->render->input[0]->buffer_size);
    
    if (!dev->render_pool) {
      print ("Failed to create render pool\n");
      return -1;
    }
  }

  if (dev->encoder){
    status = mmal_port_enable (encoder_input, render_encoder_input_callback);
    if (status != MMAL_SUCCESS)
      return -1;
    
    print ("Create pool of %d buffers of size %d for encode ip\n", encoder_input->buffer_num, isp_output->buffer_size);
    dev->encode_pool = mmal_port_pool_create (encoder_input, isp_output->buffer_num, isp_output->buffer_size);
    if (!dev->encode_pool) {
      print ("Failed to create encode ip pool\n");
      return -1;
    }
  }

  //isp_output: call back setup
  if (default_format == 0){
    status = mmal_port_enable (isp_output, encoder_buffer_cb);  //-f0
  }
  else
  {
    status = mmal_port_enable (isp_output, isp_output_callback); //-f1
  }
  if (status != MMAL_SUCCESS)
    return -1;
    
  //isp_ouput: buffer pool
  print ("Create pool of %d buffers of size %d for encode/render\n", isp_output->buffer_num, isp_output->buffer_size);
  dev->isp_output_pool = mmal_port_pool_create (isp_output, isp_output->buffer_num, isp_output->buffer_size);
  if (!dev->isp_output_pool) {
    print ("Failed to create pool\n");
    return -1;
  }

  buffers_to_isp (dev);

  // open h264 file and put the file handle in userdata for the encoder output port
  if (dev->encoder) {
    /*
     if (filename[0] == '-' && filename[1] == '\0') {
     dev->h264_fd = stdout;
     debug = 0;
     }*/
    {
      printf ("Writing data to %s\n", "/mnt/tmpfs/file.mjpg");
      dev->mjpeg_fd = fopen ("/mnt/tmpfs/file.mjpg", "wb");
    }
    /*
    dev->pts_fd = (void *) fopen ("file.pts", "wb");
    if (dev->pts_fd)    // save header for mkvmerge 
      fprintf (dev->pts_fd, "# timecode format v2\n");*/

    encoder_output->userdata = (void *) dev;

    //Create encoder output buffers
    
    print ("Create pool of %d buffers of size %d\n", encoder_output->buffer_num, encoder_output->buffer_size);
    dev->output_pool = mmal_port_pool_create (encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);
    if (!dev->output_pool) {
      print ("Failed to create pool\n");
      return -1;
    }
    
    status = mmal_port_enable (encoder_output, encoder_buffer_cb);
    if (status != MMAL_SUCCESS) {
      print ("Failed to enable port %s\n", encoder_output->name);
      return -1;
    }
  }
  
  // queue always needed
  dev->save_queue = mmal_queue_create ();
  if (!dev->save_queue) {
    print ("Failed to create queue\n");
    return -1;
  }
  
  // thread alway needed
  vcos_status = vcos_thread_create (&dev->save_thread, "save-thread", NULL, save_thread, dev);
  if (vcos_status != VCOS_SUCCESS) {
    print ("Failed to create save thread\n");
    return -1;
  }
 
  return 0;
}




/* ---------------------------------------------------------------------------
 * main
 */

static void
usage (const char *argv0)
{
  fprintf (stderr, "Usage: %s [options]\n", argv0);
  fprintf (stderr, "Available options are\n");
  fprintf (stderr, " -b	bitrate (1500000)\n");
//  fprintf (stderr, " -d               Do not use any real V4L2 capture device\n");
  fprintf (stderr, " -f <format>    Select frame format\n\t"
	   "0 = V4L2_PIX_FMT_YUYV\n\t" "1 = V4L2_PIX_FMT_MJPEG\n\t");
  fprintf (stderr, " -h		Print this help screen and exit\n");
//  fprintf (stderr, " -i image MJPEG image\n");
//  fprintf (stderr, " -m               Streaming mult for ISOC (b/w 0 and 2)\n");
//  fprintf (stderr, " -n               Number of Video buffers (b/w 2 and 32)\n");
//  fprintf (stderr, " -o <IO method> Select UVC IO method:\n\t"
//         "0 = MMAP\n\t" "1 = USER_PTR\n");
  fprintf (stderr, " -r <resolution> Select frame resolution:\n\t"
	   "0 = 480p, (720x480)\n\t" "1 = 720p, WXGA (1280x720)\n");
  fprintf (stderr, " -s <speed>	Select USB bus speed (b/w 0 and 2)\n\t"
	   "0 = Full Speed (FS)\n\t"
	   "1 = High Speed (HS)\n\t" "2 = Super Speed (SS)\n");
  fprintf (stderr, " -t		Streaming burst (b/w 0 and 15)\n");
  fprintf (stderr, " -u device	UVC Video Output device\n");
  fprintf (stderr, " -v device	V4L2 Video Capture device\n");

}


int
main (int argc, char *argv[])
{
  struct uvc_device *udev;
  struct v4l2_device *vdev;
  struct timeval tv;
  struct v4l2_format fmt;
  char *uvc_devname = "/dev/video1";
  char *v4l2_devname = "/dev/video0";
  //nchar *mjpeg_image = NULL;
  fd_set fdsv, fdsu;
  int ret, opt, nfds;
  int bulk_mode = 0;
  int bitrate= 1500000;
  //int dummy_data_gen_mode = 0;
  /* Frame format/resolution related params. */
  //int default_format = 0;     /* V4L2_PIX_FMT_YUYV */
  int default_resolution = 1;	/* 720p */
  int nbufs = 2;		/* was 2 Ping-Pong buffers */
  int vnbufs = 3;		//isp input and output buffers
  /* USB speed related params */
  int mult = 0;
  int burst = 0;
  enum usb_device_speed speed = USB_SPEED_HIGH;	/* High-Speed */
  enum v4l2_memory uvc_io_method = V4L2_MEMORY_USERPTR;
  int frames = 0;
  struct timespec start;
  struct timespec last;;
  //v4l2 setup parameters
  const struct v4l2_format_info *info;
  unsigned int pixelformat = V4L2_PIX_FMT_UYVY;
  
  int v4l2_width = 0;
  int v4l2_height = 0;
  
  char *endptr;

  while ((opt = getopt (argc, argv, "b:df:hi:m:n:o:r:s:t:u:v:z:")) != -1) {
    switch (opt) {
    case 'b':
      bitrate = atoi (optarg);
      break;

    case 'f':
      if (atoi (optarg) < 0 && atoi (optarg) > 1) {
        usage (argv[0]);
        return 1;
      }

      default_format = atoi (optarg);
      break;
    case 'g':
      //set v4l2 frame size
      //do_set_format = 1;
      v4l2_width = strtol(optarg, &endptr, 10);
      if (*endptr != 'x' || endptr == optarg) {
        print("Invalid size '%s'\n", optarg);
        return 1;
      }
      v4l2_height = strtol(endptr + 1, &endptr, 10);
      if (*endptr != 0) {
        print("Invalid size '%s'\n", optarg);
        return 1;
      }

      break;

    case 'h':
      usage (argv[0]);
      return 1;

    case 'm':
      if (atoi (optarg) < 0 && atoi (optarg) > 2) {
        usage (argv[0]);
        return 1;
      }

      mult = atoi (optarg);
      printf ("Requested Mult value = %d\n", mult);
      break;

    case 'n':
      if (atoi (optarg) < 2 && atoi (optarg) > 32) {
        usage (argv[0]);
        return 1;
      }

      nbufs = atoi (optarg);
      vnbufs = 3;		// number of input v4l2 buffers?
      //nbufs = 2;
      printf ("Number of buffers requested = %d\n", nbufs);
      break;

    case 'r':
      if (atoi (optarg) < 0 && atoi (optarg) > 1) {
        usage (argv[0]);
        return 1;
      }

      default_resolution = atoi (optarg);
      break;

    case 's':
      if (atoi (optarg) < 0 && atoi (optarg) > 2) {
        usage (argv[0]);
        return 1;
      }

      speed = atoi (optarg);
      break;

    case 't':
      if (atoi (optarg) < 0 && atoi (optarg) > 15) {
        usage (argv[0]);
        return 1;
      }

      burst = atoi (optarg);
      printf ("Requested Burst value = %d\n", burst);
      break;

    case 'u':
      uvc_devname = optarg;
      break;

    case 'v':
      v4l2_devname = optarg;
      printf ("v4l2_devname %s\n", v4l2_devname);
      break;
    
    case 'z':
      info = v4l2_format_by_name (optarg);
      if (info == NULL) {
        printf("Unsupported video format for V4L2 requested  %s\n", optarg);
        return 1;
      }
      pixelformat = info->fourcc; 
      break;

    default:
      printf ("Invalid option '-%c'\n", opt);
      usage (argv[0]);
      return 1;
    }
  }

  {
    // Try to set the default format at the V4L2 video capture device as //requested by the user. actually not setup here, use video_set_format//
    CLEAR (fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = v4l2_width;//(default_resolution == 0) ? 640 : 1280;
    fmt.fmt.pix.height = v4l2_height;//(default_resolution == 0) ? 360 : 720;
    printf ("width %d, height %d\n", fmt.fmt.pix.width, fmt.fmt.pix.height);
    switch (0) {
    case 1:
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_JPEG;
      fmt.fmt.pix.sizeimage = fmt.fmt.pix.width * fmt.fmt.pix.height * 1;
      break;
    case 0:
    default:
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      fmt.fmt.pix.sizeimage = fmt.fmt.pix.width * fmt.fmt.pix.height * 2;
      break;
    }
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
  }

  /* video_init */
  vdev = calloc (1, sizeof *vdev);
  vdev->v4l2_fd = -1;
  vdev->memtype = V4L2_MEMORY_MMAP;
  vdev->buffers = NULL;

  vdev->v4l2_devname = v4l2_devname;
  vdev->bitrate = bitrate;
  /* Open the V4L2 device. but if useing loopback this could fail */
  ret = video_open (vdev, v4l2_devname);
  //ret = v4l2_open (&vdev, v4l2_devname, &fmt);
  if (vdev == NULL || ret < 0)
    return 1;
  else
    print ("v4l2_open %s\n", v4l2_devname);
  vdev->v4l2_devname = v4l2_devname;

  /* Open the UVC device. */
  ret = uvc_video_open (&udev, uvc_devname);
  if (udev == NULL || ret < 0)
    return 1;
  print ("uvc fd %d\n", udev->uvc_fd);

  udev->uvc_streamon = 0;
  udev->uvc_devname = uvc_devname;

  //vdev->nbufs = 4;
  {
    /* Bind UVC and V4L2 devices. */
    udev->vdev = vdev;
    vdev->udev = udev;
  }

  /* Set parameters as passed by user udev width height uses default_resolution
  * settting, (vdev uses -g setting) */
  udev->width = (default_resolution == 0) ? 640 : 1280;
  udev->height = (default_resolution == 0) ? 360 : 720;

  switch (default_format) {
  case 1:
    udev->fcc = V4L2_PIX_FMT_MJPEG;
    udev->imgsize = udev->width * udev->height * 1;
    break;
  case 0:
  default:
    udev->fcc = V4L2_PIX_FMT_YUYV;
    udev->imgsize = udev->width * udev->height * 2;
    break;
  }
  
  udev->memtype = uvc_io_method;
  udev->bulk = bulk_mode;
  udev->nbufs = nbufs;
  udev->mult = mult;
  udev->burst = burst;
  udev->speed = speed;
  printf ("getting udev format\n");

  {
    /* UVC - V4L2 integrated path */
    /*IO methods used at UVC and V4L2 domains must be  complementary to avoid any memcpy from the CPU.*/
    switch (uvc_io_method) {
    case V4L2_MEMORY_MMAP:
      vdev->memtype = V4L2_MEMORY_USERPTR;
      break;
    case V4L2_MEMORY_USERPTR:
      print ("set vdev to V4L2_MEMORY_MMAP\n");
      vdev->memtype = V4L2_MEMORY_MMAP;
    default:
      break;
    }
  }

  switch (speed) {
  case USB_SPEED_FULL:
    udev->maxpkt = 1023;
    break;
  case USB_SPEED_HIGH:
    udev->maxpkt = 2048;
    break;
  case USB_SPEED_SUPER:
    udev->maxpkt = 1024;
    break;
  default:
    udev->maxpkt = 1024;
    break;
  }

  uvc_events_init (udev);

  unsigned int fmt_flags = 0;
  //fmt_flags |= V4L2_PIX_FMT_FLAG_PREMUL_ALPHA;
  unsigned int capabilities = V4L2_CAP_VIDEO_CAPTURE;
  unsigned int buftype = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  enum v4l2_memory memtype = V4L2_MEMORY_MMAP;
  int vret;
  enum v4l2_field field = V4L2_FIELD_ANY;
  //enum v4l2_field field = V4L2_FIELD_INTERLACED; // for eos m
  //field = v4l2_field_from_string
  bcm_host_init ();
  vdev->nbufs = vnbufs;

  //info = v4l2_format_by_name ("UYVY");
  //pixelformat = info->fourcc;
  vret = video_querycap (vdev, &capabilities);
  vret = cap_get_buf_type (capabilities);

  video_set_buf_type (vdev, vret);

  vdev->memtype = memtype;
  //make user selectable dimensions and field type
  ret = video_set_format (vdev, v4l2_width, v4l2_height, pixelformat, 0, 0, field, fmt_flags); // generic setup
  //ret = video_set_format (vdev, 1280, 720, V4L2_PIX_FMT_UYVY, 0, 0, field, fmt_flags);  // eos m5 ok
  //ret = video_set_format (vdev, 720 , 480, V4L2_PIX_FMT_RGB24, 0, 0, V4L2_FIELD_INTERLACED, fmt_flags); // eos m
// rGB24 is need for interlaced
  if ( ret < 0 )
    printf("Cannot set %s to %c%c%c%c\n", vdev->v4l2_devname, pixfmtstr(pixelformat));
  //video_set_format (vdev, 1280, 720, V4L2_PIX_FMT_RGB24, 0, 0, field, fmt_flags);//colours wrong

  video_set_dv_timings (vdev);
  video_get_format (vdev);
  setup_mmal_comp (vdev, vnbufs);
 // video_set_quality (&dev, -1)
  //videhhh_prepare_capture (vdev, vnbufs);//, 0, "", BUFFER_FILL_NONE); 
  video_alloc_buffers (vdev, vnbufs );
  //enable_mmal_input (vdev);
  //print ("before video q all buf\n");
  //video_queue_all_buffers(vdev, BUFFER_FILL_NONE);
  //printf("uvc; udev->width %d, udev->height %d\n", udev->width, udev->height);
  v4l2_dump ("process", vdev);
  vdev->dq_ubuf_ok = true;
  /* Init UVC events. */
  last = start;
  while (1) {
    FD_ZERO (&fdsv);
    FD_ZERO (&fdsu);
    /* We want both setup and data events on UVC interface.. */
    FD_SET (udev->uvc_fd, &fdsu);

    fd_set efds = fdsu;		//events
    fd_set dfds = fdsu;		//write
    /* Timeout. */
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    /* ..but only data events on V4L2 interface */
    FD_SET (vdev->v4l2_fd, &fdsv);

    //if (1) {
    nfds = max (vdev->v4l2_fd, udev->uvc_fd);
    ret = select (nfds + 1, &fdsv, &dfds, &efds, &tv);

    if (ret == -1) {
      printf ("select error %d, %s\n", errno, strerror (errno));
      continue;
      if (errno == EINTR)
        continue;
      break;
    }

    if (ret == 0) {
      printf ("select timeout\n");
      break;
    }

    if (FD_ISSET (udev->uvc_fd, &efds))
      uvc_events_process (udev);

    if (FD_ISSET (udev->uvc_fd, &dfds))	//write
      uvc_video_process (udev);

    if (FD_ISSET (vdev->v4l2_fd, &fdsv)) {	//reading
      ret = v4l2_process_data (vdev);
      if (ret == 1) {
        print ("v4l2_process-data failed\n");
        break;

      }
    }
    // add some delay when not streaming to reduce cpu in while loop
    if (!vdev->is_streaming && !udev->is_streaming)
      nanosleep ((const struct timespec[]) { {0, 5000000L} }, NULL);
  }
  printf ("closing");
  if (vdev->is_streaming) {
    /* Stop V4L2 streaming... */
    video_enable (vdev, 0);	//v4l2_stop_capturing (vdev);
    video_free_buffers (vdev);
    
    for (unsigned int i = 0; i < vdev->nbufs; ++i) {
      video_buffer_munmap (vdev, &vdev->buffers[i]);
    }
    vdev->is_streaming = 0;
  }
  
  if (udev->is_streaming) {
    /* ... and now UVC streaming.. */
    uvc_video_stream (udev, 0);
    //uvc_uninit_device (udev);
    uvc_video_reqbufs_userptr (udev, 0);
    udev->is_streaming = 0;
  }

  video_close (vdev);

  uvc_video_close (udev);

  return 0;
}

