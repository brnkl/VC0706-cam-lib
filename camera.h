#ifndef CAMERA_H
#define CAMERA_H

#include "legato.h"
#include <termios.h>

#define VC0706_RESP_PREFIX 0x76
#define VC0706_PREFIX 0x56
#define VC0706_RESET 0x26
#define VC0706_GEN_VERSION 0x11
#define VC0706_SET_PORT 0x24
#define VC0706_READ_FBUF 0x32
#define VC0706_GET_FBUF_LEN 0x34
#define VC0706_FBUF_CTRL 0x36
#define VC0706_DOWNSIZE_CTRL 0x54
#define VC0706_DOWNSIZE_STATUS 0x55
#define VC0706_READ_DATA 0x30
#define VC0706_WRITE_DATA 0x31
#define VC0706_COMM_MOTION_CTRL 0x37
#define VC0706_COMM_MOTION_STATUS 0x38
#define VC0706_COMM_MOTION_DETECTED 0x39
#define VC0706_MOTION_CTRL 0x42
#define VC0706_MOTION_STATUS 0x43
#define VC0706_TVOUT_CTRL 0x44
#define VC0706_OSD_ADD_CHAR 0x45

#define VC0706_STOPCURRENTFRAME 0x0
#define VC0706_STOPNEXTFRAME 0x1
#define VC0706_RESUMEFRAME 0x3
#define VC0706_STEPFRAME 0x2

#define VC0706_640x480 0x00
#define VC0706_320x240 0x11
#define VC0706_160x120 0x22

#define VC0706_MOTIONCONTROL 0x0
#define VC0706_UARTMOTION 0x01
#define VC0706_ACTIVATEMOTION 0x01

#define VC0706_SET_ZOOM 0x52
#define VC0706_GET_ZOOM 0x53

#define CAM_BUFF_SIZE 200
#define CAM_BLOCK_SIZE 192
#define CAM_DELAY 10
#define CAM_SERIAL 0
#define CAM_BAUD_RATE 38400
#define TTY_TIMEOUT 5000
#define MAX_PATH_SIZE 256

typedef struct {
  char devPath[MAX_PATH_SIZE];
  int fd;                       // file descriptor for the serial port
  uint8_t serialNum;            // camera serial number
  uint8_t buff[CAM_BUFF_SIZE];  // uint8_t array to store camera data
  uint8_t bufferLen;            // current length of data in buffer
  uint16_t frameptr;
} Camera;

// File stream functions for reading photos
LE_SHARED bool cam_snapshotToFile(Camera* cam,
                                  const char* path,
                                  uint8_t imgSize,
                                  char* imgPath);
bool cam_readImageToFile(Camera* cam, const char* path, char* imgPath);
bool cam_readImageBlocks(Camera* cam, FILE* filePtr);
uint8_t cam_getImageBlockSize(int jpgLen);

// Higher level commands
LE_SHARED bool cam_setPTZ(Camera* cam,
                          uint16_t wz,
                          uint16_t hz,
                          uint16_t pan,
                          uint16_t tilt);
LE_SHARED bool cam_getPTZ(Camera* cam,
                          uint16_t* w,
                          uint16_t* h,
                          uint16_t* wz,
                          uint16_t* hz,
                          uint16_t* pan,
                          uint16_t* tilt);
LE_SHARED bool cam_setCompression(Camera* cam, uint8_t c);
LE_SHARED uint8_t cam_getCompression(Camera* cam);
LE_SHARED bool cam_setMotionStatus(Camera* cam,
                                   uint8_t x,
                                   uint8_t d1,
                                   uint8_t d2);
LE_SHARED bool cam_setMotionDetect(Camera* cam, bool flag);
LE_SHARED bool cam_motionDetected(Camera* cam);
LE_SHARED uint8_t cam_getMotionStatus(Camera* cam, uint8_t x);
LE_SHARED bool cam_getMotionDetect(Camera* cam);
LE_SHARED bool cam_setImageSize(Camera* cam, uint8_t x);
LE_SHARED uint8_t cam_getImageSize(Camera* cam);
LE_SHARED bool cam_setDownsize(Camera* cam, uint8_t newSize);
LE_SHARED uint8_t cam_getDownsize(Camera* cam);
LE_SHARED uint8_t cam_available(Camera* cam);
LE_SHARED char* cam_getVersion(Camera* cam);
LE_SHARED uint32_t cam_frameLength(Camera* cam);
LE_SHARED bool cam_resumeVideo(Camera* cam);
LE_SHARED uint8_t* cam_readPicture(Camera* cam, uint8_t n);
LE_SHARED bool cam_tvOff(Camera* cam);
LE_SHARED bool cam_tvOn(Camera* cam);
LE_SHARED bool cam_reset(Camera* cam);
LE_SHARED bool cam_takePicture(Camera* cam);
LE_SHARED bool cam_frameBuffCtrl(Camera* cam, uint8_t cmd);

// Low level camera commands
bool cam_runCommandFlush(Camera* cam,
                         uint8_t cmd,
                         uint8_t args[],
                         uint8_t nArgs,
                         uint8_t respLen);
bool cam_verifyResponse(Camera* cam, uint8_t cmd);
uint8_t cam_readResponse(Camera* cam, uint8_t nBytes, uint8_t timeout);
bool cam_runCommand(Camera* cam,
                    uint8_t cmd,
                    uint8_t args[],
                    uint8_t nArgs,
                    uint8_t respLen,
                    bool flushFlag);
void cam_sendCommand(Camera* cam, uint8_t cmd, uint8_t args[], uint8_t nArgs);

// Serial/file descriptor helpers
LE_SHARED int fd_closeCam(int fd);
LE_SHARED int fd_openCam(char* devPath);
ssize_t fd_getByte(int fd, uint8_t* data);
#endif
