//#ifndef YAVTA_RAW_H_INCLUDED
//#define YAVTA_RAW_H_INCLUDED
#define ARRAY_SIZE(a)   ((sizeof(a) / sizeof(a[0])))
#include <linux/videodev2.h>
extern int debug;
//#define print(...) do { if (debug) printf(__VA_ARGS__); }  while (0)

#define MMAL_ENCODING_UNUSED 0
struct v4l2_format_info
{
  const char *name;
  unsigned int fourcc;
  unsigned char n_planes;
  MMAL_FOURCC_T mmal_encoding;
};

const struct v4l2_format_info pixel_formats[] = {
  {"RGB332", V4L2_PIX_FMT_RGB332, 1, MMAL_ENCODING_UNUSED},
  {"RGB444", V4L2_PIX_FMT_RGB444, 1, MMAL_ENCODING_UNUSED},
  {"ARGB444", V4L2_PIX_FMT_ARGB444, 1, MMAL_ENCODING_UNUSED},
  {"XRGB444", V4L2_PIX_FMT_XRGB444, 1, MMAL_ENCODING_UNUSED},
  {"RGB555", V4L2_PIX_FMT_RGB555, 1, MMAL_ENCODING_UNUSED},
  {"ARGB555", V4L2_PIX_FMT_ARGB555, 1, MMAL_ENCODING_UNUSED},
  {"XRGB555", V4L2_PIX_FMT_XRGB555, 1, MMAL_ENCODING_UNUSED},
  {"RGB565", V4L2_PIX_FMT_RGB565, 1, MMAL_ENCODING_UNUSED},
  {"RGB555X", V4L2_PIX_FMT_RGB555X, 1, MMAL_ENCODING_UNUSED},
  {"RGB565X", V4L2_PIX_FMT_RGB565X, 1, MMAL_ENCODING_RGB16},
  {"BGR666", V4L2_PIX_FMT_BGR666, 1, MMAL_ENCODING_UNUSED},
  {"BGR24", V4L2_PIX_FMT_BGR24, 1, MMAL_ENCODING_RGB24},
  {"RGB24", V4L2_PIX_FMT_RGB24, 1, MMAL_ENCODING_BGR24},
  {"BGR32", V4L2_PIX_FMT_BGR32, 1, MMAL_ENCODING_BGR32},
  {"ABGR32", V4L2_PIX_FMT_ABGR32, 1, MMAL_ENCODING_BGRA},
  {"XBGR32", V4L2_PIX_FMT_XBGR32, 1, MMAL_ENCODING_BGR32},
  {"RGB32", V4L2_PIX_FMT_RGB32, 1, MMAL_ENCODING_RGB32},
  {"ARGB32", V4L2_PIX_FMT_ARGB32, 1, MMAL_ENCODING_ARGB},
  {"XRGB32", V4L2_PIX_FMT_XRGB32, 1, MMAL_ENCODING_UNUSED},
  {"HSV24", V4L2_PIX_FMT_HSV24, 1, MMAL_ENCODING_UNUSED},
  {"HSV32", V4L2_PIX_FMT_HSV32, 1, MMAL_ENCODING_UNUSED},
  {"Y8", V4L2_PIX_FMT_GREY, 1, MMAL_ENCODING_UNUSED},
  {"Y10", V4L2_PIX_FMT_Y10, 1, MMAL_ENCODING_UNUSED},
  {"Y12", V4L2_PIX_FMT_Y12, 1, MMAL_ENCODING_UNUSED},
  {"Y16", V4L2_PIX_FMT_Y16, 1, MMAL_ENCODING_UNUSED},
  {"UYVY", V4L2_PIX_FMT_UYVY, 1, MMAL_ENCODING_UYVY},
  {"VYUY", V4L2_PIX_FMT_VYUY, 1, MMAL_ENCODING_VYUY},
  {"YUYV", V4L2_PIX_FMT_YUYV, 1, MMAL_ENCODING_YUYV},
  {"YVYU", V4L2_PIX_FMT_YVYU, 1, MMAL_ENCODING_YVYU},
  {"NV12", V4L2_PIX_FMT_NV12, 1, MMAL_ENCODING_NV12},
  {"NV12M", V4L2_PIX_FMT_NV12M, 2, MMAL_ENCODING_UNUSED},
  {"NV21", V4L2_PIX_FMT_NV21, 1, MMAL_ENCODING_NV21},
  {"NV21M", V4L2_PIX_FMT_NV21M, 2, MMAL_ENCODING_UNUSED},
  {"NV16", V4L2_PIX_FMT_NV16, 1, MMAL_ENCODING_UNUSED},
  {"NV16M", V4L2_PIX_FMT_NV16M, 2, MMAL_ENCODING_UNUSED},
  {"NV61", V4L2_PIX_FMT_NV61, 1, MMAL_ENCODING_UNUSED},
  {"NV61M", V4L2_PIX_FMT_NV61M, 2, MMAL_ENCODING_UNUSED},
  {"NV24", V4L2_PIX_FMT_NV24, 1, MMAL_ENCODING_UNUSED},
  {"NV42", V4L2_PIX_FMT_NV42, 1, MMAL_ENCODING_UNUSED},
 {"YUV420M", V4L2_PIX_FMT_YUV420M, 3, MMAL_ENCODING_UNUSED},
  {"YUV422M", V4L2_PIX_FMT_YUV422M, 3, MMAL_ENCODING_UNUSED},
  {"YUV444M", V4L2_PIX_FMT_YUV444M, 3, MMAL_ENCODING_UNUSED},
  {"YVU420M", V4L2_PIX_FMT_YVU420M, 3, MMAL_ENCODING_UNUSED},
  {"YVU422M", V4L2_PIX_FMT_YVU422M, 3, MMAL_ENCODING_UNUSED},
  {"YVU444M", V4L2_PIX_FMT_YVU444M, 3, MMAL_ENCODING_UNUSED},
  {"SBGGR8", V4L2_PIX_FMT_SBGGR8, 1, MMAL_ENCODING_BAYER_SBGGR8},
  {"SGBRG8", V4L2_PIX_FMT_SGBRG8, 1, MMAL_ENCODING_BAYER_SGBRG8},
  {"SGRBG8", V4L2_PIX_FMT_SGRBG8, 1, MMAL_ENCODING_BAYER_SGRBG8},
  {"SRGGB8", V4L2_PIX_FMT_SRGGB8, 1, MMAL_ENCODING_BAYER_SRGGB8},
  {"SBGGR10_DPCM8", V4L2_PIX_FMT_SBGGR10DPCM8, 1, MMAL_ENCODING_UNUSED},
  {"SGBRG10_DPCM8", V4L2_PIX_FMT_SGBRG10DPCM8, 1, MMAL_ENCODING_UNUSED},
  {"SGRBG10_DPCM8", V4L2_PIX_FMT_SGRBG10DPCM8, 1, MMAL_ENCODING_UNUSED},
  {"SRGGB10_DPCM8", V4L2_PIX_FMT_SRGGB10DPCM8, 1, MMAL_ENCODING_UNUSED},
  {"SBGGR10", V4L2_PIX_FMT_SBGGR10, 1, MMAL_ENCODING_UNUSED},
  {"SGBRG10", V4L2_PIX_FMT_SGBRG10, 1, MMAL_ENCODING_UNUSED},
  {"SGRBG10", V4L2_PIX_FMT_SGRBG10, 1, MMAL_ENCODING_UNUSED},
  {"SRGGB10", V4L2_PIX_FMT_SRGGB10, 1, MMAL_ENCODING_UNUSED},
  {"SBGGR10P", V4L2_PIX_FMT_SBGGR10P, 1, MMAL_ENCODING_BAYER_SBGGR10P},
  {"SGBRG10P", V4L2_PIX_FMT_SGBRG10P, 1, MMAL_ENCODING_BAYER_SGBRG10P},
  {"SGRBG10P", V4L2_PIX_FMT_SGRBG10P, 1, MMAL_ENCODING_BAYER_SGRBG10P},
  {"SRGGB10P", V4L2_PIX_FMT_SRGGB10P, 1, MMAL_ENCODING_BAYER_SRGGB10P},
  {"SBGGR12", V4L2_PIX_FMT_SBGGR12, 1, MMAL_ENCODING_UNUSED},
  {"SGBRG12", V4L2_PIX_FMT_SGBRG12, 1, MMAL_ENCODING_UNUSED},
  {"SGRBG12", V4L2_PIX_FMT_SGRBG12, 1, MMAL_ENCODING_UNUSED},
  {"SRGGB12", V4L2_PIX_FMT_SRGGB12, 1, MMAL_ENCODING_UNUSED},
  {"DV", V4L2_PIX_FMT_DV, 1, MMAL_ENCODING_UNUSED},
  {"MJPEG", V4L2_PIX_FMT_MJPEG, 1, MMAL_ENCODING_UNUSED},
  {"MPEG", V4L2_PIX_FMT_MPEG, 1, MMAL_ENCODING_UNUSED},
};

enum buffer_fill_mode
{
  BUFFER_FILL_NONE = 0,
  BUFFER_FILL_FRAME = 1 << 0,
  BUFFER_FILL_PADDING = 1 << 1,
};



/* Buffer representing one video frame */

struct buffer
{
  struct v4l2_buffer buf;
  void *start;
  size_t length;
};

struct mmal_buffer
{
  unsigned int idx;
  unsigned int padding[VIDEO_MAX_PLANES];
  unsigned int size[VIDEO_MAX_PLANES];
  void *mem[VIDEO_MAX_PLANES];
  MMAL_BUFFER_HEADER_T *mmal;
  int dma_fd;
  unsigned int vcsm_handle;
};

/* Represents a UVC based video output device */
struct uvc_device
{
  // uvc device specific /
  int uvc_fd;
  int is_streaming;
  //nint run_standalone;
  char *uvc_devname;

  // uvc control request specific //
  struct uvc_streaming_control probe;
  struct uvc_streaming_control commit;
  int control;
  struct uvc_request_data request_error_code;
  unsigned int brightness_val;

  // uvc buffer specific //
  //enum io_method io;
  enum v4l2_memory memtype;
  struct buffer *mem;
  
  //struct buffer *dummy_buf;
  unsigned int nbufs;
  unsigned int fcc;
  unsigned int width;
  unsigned int height;

  unsigned int bulk;
  uint8_t color;
  unsigned int imgsize;
  void *imgdata;

  // USB speed specific //
  int mult;
  int burst;
  int maxpkt;
  enum usb_device_speed speed;

  // uvc specific flags //
  int first_buffer_queued;
  int uvc_shutdown_requested;
  int uvc_streamon;
  // uvc buffer queue and dequeue counters //
  unsigned long long int qbuf_count;
  unsigned long long int dqbuf_count;

  // v4l2 device hook //
  struct v4l2_device *vdev;
};

/* Represents a V4L2 based video capture device */
struct v4l2_device
{
  /* v4l2 device specific */
  int v4l2_fd;
  int opened;
  int fd2;
  int opened2;
  int is_streaming;
  char *v4l2_devname;
  int mjpeg_fd;
  int counter;
  /* v4l2 buffer specific */
  struct buffer *mem; //store /dev/video0 buffer info
  unsigned int nbufs;
  struct mmal_buffer *buffers;
  
  enum v4l2_buf_type type;
  enum v4l2_memory memtype;
  //enum io_method io;
  ///
  unsigned int width;
  unsigned int height;
  unsigned int fps;
  unsigned int frame_time_usec;
  uint32_t buffer_output_flags;
  uint32_t timestamp_type;
  struct timeval starttime;
  int64_t lastpts;
  struct timespec last ; //general timer tracking
  /* v4l2 buffer queue and dequeue counters */
  unsigned long long int qbuf_count;
  unsigned long long int dqbuf_count;
  int bitrate;

  /* uvc device hook */
  struct uvc_device *udev;

  MMAL_FOURCC_T dst_mmal_enc;
  //mmal setup
  
  VCOS_THREAD_T save_thread;
  MMAL_QUEUE_T *save_queue;
  int thread_quit;

  MMAL_COMPONENT_T *isp;
  MMAL_COMPONENT_T *render;
  MMAL_COMPONENT_T *encoder;
  MMAL_POOL_T *isp_output_pool;
  MMAL_POOL_T *render_pool;
  MMAL_POOL_T *encode_pool;

  MMAL_BOOL_T can_zero_copy;

  /* V4L2 to MMAL interface */
  MMAL_QUEUE_T *isp_queue;
  MMAL_POOL_T *mmal_pool;
  /* Encoded data */
  MMAL_POOL_T *output_pool;


  unsigned char num_planes;
  struct v4l2_plane_pix_format plane_fmt[VIDEO_MAX_PLANES];
  struct v4l2_buffer vbuf;
  void *pattern[VIDEO_MAX_PLANES];
  unsigned int patternsize[VIDEO_MAX_PLANES];
  bool write_data_prefix;
  
  bool dq_ubuf_ok;
};
//extern const char * v4l2_format_name (unsigned int fourcc);
//extern const struct v4l2_format_info * v4l2_format_by_name (const char *name);
//extern const struct v4l2_format_info * v4l2_format_by_fourcc (unsigned int );

