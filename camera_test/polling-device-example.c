#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/un.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include "fault_lib.h"

#include <sys/types.h>  
#include <sys/stat.h>  
#include <fcntl.h>
#define IS_ANDROID
//#define IS_LINUX

#if defined(IS_ANDROID)
#include <android/log.h>

#define LOG_TAG "PollingDeviceExample"
//#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__))
//#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__))
//#define LOGE(...) ((void)__android_log_print(ANDROID_LOG_ERROR,LOG_TAG, __VA_ARGS__))
#define LOGI(...) printf(__VA_ARGS__)
#define LOGW(...) //printf(__VA_ARGS__)
#define LOGE(...) printf(__VA_ARGS__)
#elif defined(IS_LINUX)
#define LOGI(...) printf(__VA_ARGS__)
#define LOGW(...) //printf(__VA_ARGS__)
#define LOGE(...) printf(__VA_ARGS__)
#else
#define LOGI(...)
#define LOGW(...)
#define LOGE(...)
#endif

#if defined(IS_LINUX)
#define max20088_cmd_addr "/sys/devices/platform/11010000.i2c/i2c-3/3-0068/register_addr"
#define max20088_cmd_val "/sys/devices/platform/11010000.i2c/i2c-3/3-0068/linux_register_max20088"
#define max20086_cmd_addr "/sys/devices/platform/11009000.i2c/i2c-2/2-0048/register_addr"
#define max20086_cmd_val "/sys/devices/platform/11009000.i2c/i2c-2/2-0048/linux_register_max20086"
char staBuff[255][255]={0};
char max20088_linux_status()
{
    int i = 0,j = 0;
    int ret = 0;
    char buff[10]={0};
    char buff_t[10]={0};
    char val[20][20]={0};
    int addr[10] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};
    int fd,fd_t;
    memset(staBuff,0,255);
    fd = open(max20088_cmd_addr, O_RDWR /* required */ | O_NONBLOCK);
    if (fd < 0){
        LOGE("open failed\n");
        close(fd);
        return fd;
    }
    fd_t = open(max20088_cmd_val, O_RDWR /* required */ | O_NONBLOCK);
    if (fd_t < 0){
        LOGE("open failed\n");
        close(fd_t);
        return fd;
    }

    for (i = 0;i<10;i++){
        memset(buff,0,10);
        sprintf(buff,"0x%02x",addr[i]);
        LOGI("buff=%s,length=%lu\n",buff,strlen(buff));
        write(fd,buff,strlen(buff));
        for(j=0;j<3;j++){
            lseek(fd_t,0,SEEK_SET);
            ret = read(fd_t,buff_t,4);
            if(ret < 0){
                LOGE("read error1\n");
                return ret;
            }else if (ret == 0){
                printf("ret:0\n");
            }else{
                printf("ret = %d\n",ret);
                
            }
        }
        //printf("buff_t=%s\n",buff_t);
        sprintf(buff_t,"%s",buff_t);
        //printf("buff_t=%s\n",buff_t);
        strcpy(val[i],buff_t);
        printf("----------------\n");
    }

    LOGI("ADDR:  ");
    for (i=0;i<10;i++){
        LOGI("0x%02x\t",addr[i]);
    }
    LOGI("\n");
    LOGI("VALUE: ");
    for (i=0;i<10;i++){
        printf("%s\t",val[i]);
    }
    LOGI(" \nDiagnosis information:\n");
    if (strcmp(val[2],"0x11")==0){
        LOGI("MAX20088 ID matched!\n");
        strcpy(staBuff[0],"ID:MAX20088.");
    }else{
        LOGE("MAX20088 ID ERROR!\n");
    }

    if (strcmp(val[4],"0x05")==0 && strcmp(val[6],"0x00")==0){
        strcpy(staBuff[1],"Over-current present and Output voltage < UV threshold");
        return 1;
    }else if (strcmp(val[6],"0x00")==0){
        strcpy(staBuff[2],"camera not connected!");
        return 2;
    }else{
        LOGI("camera connected!\n");
    }

    close(fd);
    close(fd_t);
    return ret;
}
#endif
#if defined(IS_ANDROID)
#define cmd_addr "/sys/devices/platform/11009000.i2c/i2c-2/2-004a/register_addr"
#define max20088_cmd_val "/sys/devices/platform/11009000.i2c/i2c-2/2-004a/android_register_max20088"
#define tmp102_cmd_val "/sys/devices/platform/11009000.i2c/i2c-2/2-004a/android_register_temp102"
#define max9286_cmd_val "/sys/devices/platform/11009000.i2c/i2c-2/2-004a/android_register_max9286"
char max20088_android_status()
{
    return 0;
}
#include<math.h>
int convert(int a,char *p)//a为字符串长度，p为需要转换字符串指针
{ 
    int i,j,sum;
    sum=0;
    for(i=0;i<a;i++)
    {   
        if(*(p+i)<='f'&&*(p+i)>='a') 
            j=(int)(*(p+i))-87; 
        else if(*(p+i)<='F'&&*(p+i)>='A') 
            j=(int)(*(p+i))-55;
        else   
            j=(int)(*(p+i))-48; 
        sum=sum+pow(16.0,a-i-1)*j;
    }   
    return (sum);
} 

char tmp102(int flag)
{
    float tmp = 0;
    char data[10] = {0};
    int fd,fd_t,fd_t1;
    int ret = 0,i = 0,j = 0;
    char buff[10]={0};
    char buff_t[10]={0};
    char val[20][20]={0};
    int addr[4] = {0x00,0x01,0x10,0x11};
    fd = open(cmd_addr, O_RDWR /* required */ | O_NONBLOCK);
    if (fd < 0){
        LOGE("open failed\n");
        close(fd);
        return fd;
    }
    fd_t = open(tmp102_cmd_val, O_RDWR /* required */ | O_NONBLOCK);
    if (fd_t < 0){
        LOGE("open failed\n");
        close(fd_t);
        return fd;
    }
    fd_t1 = open(max9286_cmd_val, O_RDWR /* required */ | O_NONBLOCK);
    if (fd_t < 0){
        LOGE("open failed\n");
        close(fd_t1);
        return fd;
    }
    if (flag < 0 || flag > 2){
        LOGE("input flag error. (flag < 0 || flag > 2) \n");
        return ret;
    }
    if(flag == 1){
        write(fd,"0x0a",4);
        write(fd_t1,"0xf1",4);
    }else if (flag == 2){
        write(fd,"0x0a",4);
        write(fd_t1,"0xf2",4);
    }else{
        LOGE("flag:%d\n",flag);
        return ret;
    }

    for (i = 0;i<4;i++){
        memset(buff,0,10);
        sprintf(buff,"0x%02x",addr[i]);
        LOGI("buff=%s,length=%lu\n",buff,strlen(buff));
        write(fd,buff,strlen(buff));
        for(j=0;j<2;j++){
            lseek(fd_t,0,SEEK_SET);
            ret = read(fd_t,buff_t,6);
            if(ret < 0){
                LOGE("read error\n");
                return ret;
            }else if (ret == 0){
                printf("ret:0\n");
            }else{
                printf("ret = %d\n",ret);
            }
        }
        //printf("buff_t=%s\n",buff_t);
        sprintf(buff_t,"%s",buff_t);
        //printf("buff_t=%s\n",buff_t);
        strcpy(val[i],buff_t);
        printf("----------------\n");
    }
    LOGI("ADDR:  ");
    for (i=0;i<4;i++){
        LOGI("0x%02x\t",addr[i]);
    }
    LOGI("\n");
    LOGI("VALUE: ");
    for (i=0;i<4;i++){
        printf("%s\t",val[i]);
    }
    LOGI("\n");

    write(fd,&addr[1],4);
    write(fd_t,"0x70a0",6);

    write(fd,&addr[0],4);
    lseek(fd_t,0,SEEK_SET);
    ret = read(fd_t,buff_t,6);
    if(ret < 0){
        LOGE("read error\n");
        return ret;
    }else if (ret == 0){
        printf("ret:0\n");
    }else{
        printf("ret = %d\n",ret);
    }
    printf("buff_t=%s\n",buff_t);


    for(i = 0;i<6;i++){
        
        printf("buff_t[%d]=%c\t",i,buff_t[i]);
    }
    LOGI("\n");
    for(i = 0;i<4;i++){
        data[i]=buff_t[i+2];
        printf("data[%d]=%c\t",i,data[i]);
    }
    LOGI("\n");
    printf("data=%s\n",data);
    printf("data = %d\n",convert(3,data));
    tmp = convert(3,data)*0.0625;
    printf("tmp = %f\n",tmp);

    write(fd,"0x0a",4);
    write(fd_t1,"0xf3",4);

    close(fd);
    close(fd_t);
    return ret;
}
#endif
#if 1
int main(int argc,char * argv[])
{
#if 0
    int i = 0;
    char flag = 0;
    flag = max20088_linux_status();
    if(flag == 1){
        LOGI("...1\n");
    }else if (flag == 2){
        LOGI("...2\n");
    }else if (flag == 3){
        LOGI("...3\n");
    }else{
        LOGI("hello world!\n");
    }
    
    for(i=0;i<5;i++)
        LOGI("\n%d.%s\n",i,staBuff[i]);
#endif
    int flag;
    while(1){
        scanf("%d",&flag);
        tmp102(flag);
    }
    return 0;
}

#else
char max20086_linux_status()
{
    return 0;
}
char polling_callback(char *name)
{
	LOGI("polling_callback(name=%s) start.\r\n", name);

	if (strcmp(name, "max20088_linux") == 0) {
	    return max20088_linux_status();
	} else if (strcmp(name, "max20086_linux") == 0) {
		return max20086_linux_status();
	} else if (strcmp(name, "max20088_android") == 0) {
		return max20088_android_status();
	} else if (strcmp(name, "max20086_android") == 0) {
		return max20086_android_status();
	} else {
		LOGE("no %s device.\r\n", name);
		return -1;
	}
}

int main(void)
{
    pid_t pc;

    LOGI("polling-device-example start.\r\n");
    pc = fork();
    if (pc < 0) {
        LOGE("fork error(%s).\r\n", strerror(errno));
        exit(1);
    } else if (pc == 0) {
        //LOGI("child process.\r\n");
    } else {
        //LOGI("parent process: the child process id = %d\r\n",
        //getpid());
        exit(0);
    }

    if (setsid() < 0) {
        LOGE("setsid error(%s).\r\n", strerror(errno));
        exit(1);
    }

    if (fault_device_init() < 0) {
        LOGE("fault_device_init failed.\r\n");
        exit(1);
	}

    if (polling_device_register("max20088_linux", 1,
                (char*)polling_callback) < 0) {
        LOGE("register example6_name failed.\r\n");
        goto out;
    }

    if (polling_device_register("max20086_linux", 3,
                (char *)polling_callback) < 0) {
        LOGE("register example7_name failed.\r\n");
        goto out;
    }

    if (polling_device_register("max20088_android", 5,
                (char *)polling_callback) < 0) {
        LOGE("register example8_name failed.\r\n");
        goto out;
    }

    if (polling_device_register("max20086_android", 5,
                (char *)polling_callback) < 0) {
        LOGE("register example8_name failed.\r\n");
        goto out;
    }

	//while(1) {
	for (int i = 0; i < 3; i++) {
		sleep(10);
	}
	//};

    if (polling_device_unregister("example6_name") < 0) {
        LOGE("unregister example6_name failed.\r\n");
        goto out;
    }

    if (polling_device_unregister("example7_name") < 0) {
        LOGE("unregister example7_name failed.\r\n");
        goto out;
    }

    if (polling_device_unregister("example8_name") < 0) {
        LOGE("unregister example8_name failed.\r\n");
        goto out;
    }

out:
    if (fault_device_remove() < 0) {
        LOGE("fault_device_remove failed.\r\n");
        exit(1);
    }
	return 0;
}
#endif
