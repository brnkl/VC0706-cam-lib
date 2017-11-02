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
#define BUFF_SIZE 1000
#define DELAY 10
#define TIMEOUT 200
#define CAM_SERIAL 0

static const char SERIAL_PATH[] = "/dev/ttyHS0";

typedef uint8_t byte;

typedef struct {
  int fd; // file descriptor for the serial port
  byte serialNum; // camera serial number
  byte buff[BUFF_SIZE]; // byte array to store camera data
  int bufferLen; // current length of data in buffer
  uint16_t frameptr;
} Camera;

void delay (unsigned int secs);
int openCameraFd ();
void sendCommand (Camera *cam, byte cmd, byte args[]);
bool runCommand (Camera *cam, byte cmd, byte args[], int respLen);
int readResponse (Camera *cam, int nBytes, int timeout);
bool verifyResponse (Camera *cam, byte cmd);
bool cameraFrameBuffCtrl (Camera *cam, byte cmd);
bool takePicture (Camera *cam);
size_t bufferToFile (Camera *cam, char *path);
bool reset (Camera *cam);
bool TVon (Camera *cam);
bool TVOff (Camera *cam);
byte *readPicture (Camera *cam);
bool resumeVideo (Camera *cam);
uint32_t frameLength (Camera *cam);
char *getVersion (Camera *cam);
byte available (Camera *cam);
byte getDownsize (Camera *cam);
bool setDownsize(Camera *cam, int newSize);
int getImageSize (Camera *cam);
bool setImageSize (Camera *cam, int size);
bool getMotionDetect (Camera *cam);
byte getMotionStatus(Camera *cam, byte x);
bool motionDetected (Camera *cam);
bool setMotionDetect (bool flag);
bool setMotionStatus (byte x, byte d1, byte d2);
byte getCompression (Camera *cam);
bool setCompression(Camera *cam, byte c);
bool getPTZ(Camera *cam, uint16_t *w, uint16_t *h, uint16_t *wz, uint16_t *hz, uint16_t *pan, uint16_t *tilt);
bool setPTZ(Camera *cam, uint16_t wz, uint16_t hz, uint16_t pan, uint16_t tilt);
