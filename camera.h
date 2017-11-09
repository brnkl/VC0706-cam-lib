#include "legato.h"

#define VC0706_RESP_PREFIX 0x76
#define VC0706_PREFIX 0x56
#define VC0706_RESET  0x26
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

#define CAM_BAUD_RATE LE_TTY_SPEED_38400
#define BUFF_SIZE 100
#define DELAY 10
#define TIMEOUT 200
#define CAM_SERIAL 0

static const char SERIAL_PATH[] = "/dev/ttyHS0";

typedef struct {
  int fd; // file descriptor for the serial port
  uint8_t serialNum; // camera serial number
  uint8_t buff[BUFF_SIZE]; // uint8_t array to store camera data
  uint8_t bufferLen; // current length of data in buffer
  uint16_t frameptr;
} Camera;

void delay (unsigned int msecs);
int openCameraFd ();
void sendCommand (Camera *cam, uint8_t cmd, uint8_t args[]);
bool runCommand (Camera *cam, uint8_t cmd, uint8_t args[], int respLen);
uint8_t readResponse (Camera *cam, int nBytes, unsigned int timeout);
bool verifyResponse (Camera *cam, uint8_t cmd);
bool cameraFrameBuffCtrl (Camera *cam, uint8_t cmd);
bool takePicture (Camera *cam);
bool reset (Camera *cam);
bool TVon (Camera *cam);
bool TVOff (Camera *cam);
uint8_t *readPicture (Camera *cam, uint8_t n);
bool resumeVideo (Camera *cam);
uint32_t frameLength (Camera *cam);
char *getVersion (Camera *cam);
uint8_t available (Camera *cam);
uint8_t getDownsize (Camera *cam);
bool setDownsize(Camera *cam, uint8_t newSize);
uint8_t getImageSize (Camera *cam);
bool setImageSize (Camera *cam, uint8_t x);
bool getMotionDetect (Camera *cam);
uint8_t getMotionStatus(Camera *cam, uint8_t x);
bool motionDetected (Camera *cam);
bool setMotionDetect (Camera *cam, bool flag);
bool setMotionStatus (Camera *cam, uint8_t x, uint8_t d1, uint8_t d2);
uint8_t getCompression (Camera *cam);
bool setCompression(Camera *cam, uint8_t c);
bool getPTZ(Camera *cam, uint16_t *w, uint16_t *h, uint16_t *wz, uint16_t *hz, uint16_t *pan, uint16_t *tilt);
bool setPTZ(Camera *cam, uint16_t wz, uint16_t hz, uint16_t pan, uint16_t tilt);
bool snapshotToFile (Camera *cam, char *path, uint8_t imgSize);
