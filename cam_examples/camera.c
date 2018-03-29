#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <malloc.h>
#include <sys/mman.h>
#include <unistd.h>
//#define CAMERA_DEV "/dev/video7"
#define CLEAR(x) memset (&(x), 0, sizeof (x))

static int camera_fd;
struct buffer {
    void *      start;
    size_t      length;
};

typedef struct {
    char *data;
    unsigned long length;
} frame_t;

struct buffer *  buffers = NULL;
static unsigned int  n_buffers = 0;

int init_device (int fd, int width, int height, int fps)
{
    struct v4l2_requestbuffers req;
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    unsigned int min;
    int ret;
    struct v4l2_fmtdesc fmtdesc;

    /*1.查询设备属性VIDIOC_QUERYCAP*/
    if (-1 == ioctl (fd, VIDIOC_QUERYCAP, &cap)) {
        fprintf (stderr, "VIDIOC_QUERYCAP fail\n");
        goto err;
    }
        /*V4L2_CAP_VIDEO_CAPTURE:是否支持图像获取能力*/
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        fprintf (stderr, "VIDIOC_QUERYCAP fail\n");
        goto err;
    }
      /*其中域 capabilities 代表设备支持的操作模式，常见的值有
      * V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING 表示是一个
      * 视频捕捉设备并且具有数据流控制模式*/
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        fprintf (stderr, "device does not support streaming i/o\n");
        goto err;
    }
    /* Print capability infomations */
    printf("Capability Informations:\n");
    printf(" driver: %s\n", cap.driver);             /*驱动名字*/
    printf(" card: %s\n", cap.card);                 /*设备名字*/
    printf(" bus_info: %s\n", cap.bus_info);         /*设备在系统中的位置*/
    printf(" version: %08X\n", cap.version);         /*驱动版本号*/

    /* Select video input, video standard and tune here. */
    /*2.图像的缩放VIDIO_CROPCAP*/
    CLEAR (cropcap);
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;      /*指定buf的类型为capture，用于视频捕获设备*/
    if (0 == ioctl (fd, VIDIOC_CROPCAP, &cropcap)) {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default *//*窗口大小为默认值*/

        if (-1 == ioctl (fd, VIDIOC_S_CROP, &crop)) {/*设置缩放*/
            switch (errno) {
                case EINVAL:
                    /* Cropping not supported. */
                    break;
                default:
                    /* Errors ignored. */
                    break;
            }
        }
    }
    /* enum video formats. */
    CLEAR(fmtdesc);
    /*3.帧格式*/
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    printf("Enum format:\n");
    while ((ret = ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc)) == 0)/*查询所有支持的格式*/
    {
        fmtdesc.index++;/*index 要查询的格式序号*/
        printf(" <%d> pixelformat = \"%c%c%c%c\", description = %s\n",fmtdesc.index,
                fmtdesc.pixelformat & 0xFF,
                (fmtdesc.pixelformat >> 8) & 0xFF,
                (fmtdesc.pixelformat >> 16) & 0xFF,
                (fmtdesc.pixelformat >> 24) & 0xFF,
                fmtdesc.description);/*格式名称*/
    }

    /* set video formats. */
    CLEAR (fmt);
    char * p = (char *)(&fmt.fmt.pix.pixelformat);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl (fd, VIDIOC_G_FMT, &fmt) < 0) {
        /* Errors ignored. */
        printf("get fmt fail\n");
    }

    fmt.fmt.pix.width       = width;
    fmt.fmt.pix.height      = height;
    //    init_mem_repo();
    //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
        
    if (-1 == ioctl (fd, VIDIOC_S_FMT, &fmt)){/*先查询再设置*/
        printf("set format fail\n");
        return -1;
    }

    if (ioctl (fd, VIDIOC_G_FMT, &fmt) < 0) {/*检查是否支持*/
        /* Errors ignored. */
        printf("get fmt fail\n");
    }

    printf("fmt.type = %d\n", fmt.type);
    printf("fmt.width = %d\n", fmt.fmt.pix.width);
    printf("fmt.height = %d\n", fmt.fmt.pix.height);
    printf("fmt.format = %c%c%c%c\n", p[0], p[1], p[2], p[3]);
    printf("fmt.field = %d\n", fmt.fmt.pix.field);
    printf("fps = %d\n", fps);
    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;

    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;

    struct v4l2_streamparm* setfps;
    setfps=(struct v4l2_streamparm *) calloc(1, sizeof(struct v4l2_streamparm));
    memset(setfps, 0, sizeof(struct v4l2_streamparm));
    setfps->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    setfps->parm.capture.timeperframe.numerator = 1;
    setfps->parm.capture.timeperframe.denominator = fps;
    /*4.设置视频帧率*/
    if(ioctl(fd, VIDIOC_S_PARM, setfps) < 0){
        printf("set fps fail\n");
        return -1;
    }

    /*5.向设备申请缓冲区*/
    CLEAR (req);
    req.count   = 16;/*缓冲区内缓冲帧的数目*/
    req.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;/*帧缓冲数据格式*/
    req.memory  = V4L2_MEMORY_MMAP;/*区别是内存映射还是用户指针方式*/

    if (-1 == ioctl (fd, VIDIOC_REQBUFS, &req)) {//reqbufs
        fprintf (stderr, "VIDIOC_QUERYCAP fail\n");
        goto err;
    }

    if (req.count < 2) {
        fprintf (stderr, "Insufficient buffer memory\n");
        return -1;
    }

    buffers = calloc (req.count, sizeof (*buffers));
    if (!buffers) {
        fprintf (stderr, "Out of memory\n");
        return -1;
    }
    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
        struct v4l2_buffer buf;

        CLEAR (buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = n_buffers;

        if (-1 == ioctl (fd, VIDIOC_QUERYBUF, &buf)){/*获取缓冲区地址*/
            fprintf (stderr, "VIDIOC_QUERYCAP fail\n");
            goto err;
        }

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start = mmap (NULL /* start anywhere */,
                buf.length,
                PROT_READ | PROT_WRITE /* required */,
                MAP_SHARED /* recommended */,
                fd, buf.m.offset);/*被映射对象的起点*/

        if (MAP_FAILED/* (void *)-1 */ == buffers[n_buffers].start){
            fprintf (stderr, "mmap fail\n");
            goto err;
        }
    }

    return 0;

err:
    return -1;
}
static void errno_exit (const char *s)
{
    fprintf (stderr, "%s error %d, %s\n",s, errno, strerror (errno));
    exit (EXIT_FAILURE);
}

void start_capturing (int fd)
{
    unsigned int i,ret;
    enum v4l2_buf_type type;

    printf("n_buffers:%d\n",n_buffers);
    for (i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR (buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;
        /*5.将capture缓冲帧依次放入capture队列中准备开始获取图像*/
        ret = ioctl (fd, VIDIOC_QBUF, &buf);
        if (-1 == ret)
            errno_exit ("VIDIOC_QBUF");
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    /*6.打开视频流*/
    ret = ioctl (fd, VIDIOC_STREAMON, &type);
    if (-1 == ret)
        errno_exit ("VIDIOC_STREAMON");
}
/*
 * read a frame of image from video device
 */
int get_buffer(int fd, struct v4l2_buffer *v4l_buf)
{
    if (-1 == ioctl (fd, VIDIOC_DQBUF, v4l_buf))
        return 0;

    return 1;
}

/*
 * enqueue the frame again
 */
int put_buffer(int fd, struct v4l2_buffer *v4l_buf)
{
    return ioctl(fd, VIDIOC_QBUF, v4l_buf);
}

int main(int argc,char *argv[])
{
    int fd ,fd1 = 0;
    int ret,r,i;
    struct timeval tv;
    fd_set fds;
    char file[100];
    char dev_name[16];

    if (argc !=2) {
        printf("invalid param!\n");
        return argc;
    }   
    ret = snprintf(dev_name, 16, "/dev/video%d", atoi(argv[1]));
    if (ret < 0) {
        printf("Get dev name error!");
        return ret;
    }   

    frame_t *fm = malloc(sizeof(frame_t));
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    //fd =open (CAMERA_DEV, O_RDWR /* required */ | O_NONBLOCK, 0); 
    fd =open (dev_name, O_RDWR /* required */ | O_NONBLOCK, 0); 
    if(fd < 0){
        printf("open camera failed\n");    
        close(fd);
        return -1;
    }

    camera_fd = fd;

    printf("init device\n");
    init_device (fd, 1280, 800, 30);
    printf("start capturing\n");
    start_capturing (fd);
    for(i=0;i<1000;i++)
    {
        FD_ZERO (&fds);/*套接字清空*/
        FD_SET (fd, &fds);/*用于在文件描述符集合中增加一个新的文件描述符*/
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        r = select (fd + 1, &fds, NULL, NULL, &tv);
        printf("r1= %d\n",r);
        r = get_buffer(fd, &buf);
        printf("r2= %d\n",r);
        fm->data = buffers[buf.index].start;
        fm->length = buf.bytesused;
        //    if(i == 200 )
        //    {
        if(r!=0)
        {
            //sprintf(file,"/luo/pic%d.jpg",i);
            sprintf(file,"/flash/luo/pic%d.jpg",i);
            fd1 = open(file,O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
            if(fd1)
            {
                printf("open wch and write\n");
                write(fd1,fm->data,fm->length);
                close(fd1);
            }
        }
        //    }
        r = put_buffer(fd, &buf);
        printf("r3= %d\n",r);
    }
    return 0;    

} 
