#include "legato.h"
#include "interfaces.h"
#include "camera.h"
#include <time.h>

// Used to wait for the serial port
void delay (unsigned int msecs) {
  unsigned int secs = msecs / 1000;
  unsigned int retTime = time(0) + secs;
  while (time(0) < retTime);
}

// Returns a file descriptor that can be used to
// talk to the camera
int openCameraFd () {
  int fd = le_tty_Open(SERIAL_PATH, O_RDWR);
  le_tty_SetBaudRate(fd, CAM_BAUD_RATE);
  return fd;
}

// Send a command to the camera serial port
void sendCommand (Camera *cam, uint8_t cmd, uint8_t args[]) {
  uint8_t init = VC0706_PREFIX;
  write(cam->fd, &init, 1); // send the cmd prefix
  write(cam->fd, &(cam->serialNum), 1); // send the serial number
  write(cam->fd, &cmd, 1); // send the cmd
  for(int i = 0; i < NUM_ARRAY_MEMBERS(args); i++) { // send each arg
    write(cam->fd, &args[i], 1);
  }
}

// Run a command
bool runCommand (Camera *cam, uint8_t cmd, uint8_t args[], int respLen) {
  sendCommand(cam, cmd, args);
  if (readResponse(cam, respLen, TIMEOUT) != respLen)
    return false;
  if (verifyResponse(cam, cmd))
    return false;
  return true;
}

// Reads from the camera and returns how many bytes it read
uint8_t readResponse (Camera *cam, int nBytes, unsigned int timeout) {
  int counter = 0;
  cam->bufferLen = 0;
  // read while below timeout and while the buffer
  // is still smaller than expected
  while(timeout >= counter && cam->bufferLen <= nBytes) {
    ssize_t bytesRead = read(cam->fd, &(cam->buff[0]) + cam->bufferLen, 1); // read one uint8_t at a time
    cam->bufferLen++;
    // bytesRead will be 0 or -1 if no data was received
    if (bytesRead <= 0) {
      delay(1);
      counter++;
      continue;
    }
    counter = 0;
  }
  return cam->bufferLen;
}

bool verifyResponse (Camera *cam, uint8_t cmd) {
  // If any of these are not equal than
  // the command failed
  return
    !(cam->buff[0] != VC0706_RESP_PREFIX ||
    cam->buff[1] != cam->serialNum ||
    cam->buff[2] != cmd ||
    cam->buff[3] != 0x0);
}

bool cameraFrameBuffCtrl (Camera *cam, uint8_t cmd) {
  uint8_t args[] = { 0x1, cmd };
  return runCommand(cam, cmd, args, 5);
}

bool takePicture (Camera *cam) {
  cam->frameptr = 0;
  return cameraFrameBuffCtrl(cam, VC0706_STOPCURRENTFRAME);
}

bool reset (Camera *cam) {
  uint8_t args[] = { 0x0 };
  return runCommand(cam, VC0706_RESET, args, 5);
}

bool TVon (Camera *cam) {
  uint8_t args[] = { 0x1, 0x1 };
  return runCommand(cam, VC0706_TVOUT_CTRL, args, 5);
}

bool TVOff (Camera *cam) {
  uint8_t args[] = { 0x1, 0x0 };
  return runCommand(cam, VC0706_TVOUT_CTRL, args, 5);
}

uint8_t *readPicture (Camera *cam, uint8_t n) {
  uint8_t args[] = { 0x0C, 0x0, 0x0A,
    0, 0, cam->frameptr >> 8, cam->frameptr & 0xFF,
    0, 0, 0, n,
    DELAY >> 8, DELAY & 0xFF };
  if (!runCommand(cam, VC0706_READ_FBUF, args, 5))
    return 0;
  if (readResponse(cam, n + 5, DELAY) == 0)
    return 0;

  cam->frameptr += n;

  return &(cam->buff[0]);
}

bool resumeVideo (Camera *cam) {
  return cameraFrameBuffCtrl(cam, VC0706_RESUMEFRAME);
}

uint32_t frameLength (Camera *cam) {
  uint8_t args[] = { 0x01, 0x00 };
  if (!runCommand(cam, VC0706_GET_FBUF_LEN, args, 9))
    return 0;

  uint32_t len;
  len = cam->buff[5];
  len <<= 8;
  len |= cam->buff[6];
  len <<= 8;
  len |= cam->buff[7];
  len <<= 8;
  len |= cam->buff[8];

  return len;
}

char *getVersion (Camera *cam) {
  uint8_t args[] = { 0x01 };
  sendCommand(cam, VC0706_GEN_VERSION, args);
  if (!readResponse(cam, BUFF_SIZE, 200))
    return 0;
  cam->buff[cam->bufferLen] = 0;
  return (char*)&(cam->buff[0]);
}

uint8_t available (Camera *cam) {
  return cam->bufferLen;
}

uint8_t getDownsize (Camera *cam) {
  uint8_t args[] = { 0x0 };
  if (!runCommand(cam, VC0706_DOWNSIZE_STATUS, args, 6))
    return -1;
  return cam->buff[5];
}

bool setDownsize (Camera *cam, uint8_t newSize) {
  uint8_t args[] = { 0x01, newSize };
  return runCommand(cam, VC0706_DOWNSIZE_CTRL, args, 5);
}

uint8_t getImageSize (Camera *cam) {
  uint8_t args[] = { 0x4, 0x4, 0x1, 0x00, 0x19 };
  if (!runCommand(cam, VC0706_READ_DATA, args, 6))
    return -1;
  return cam->buff[5];
}

bool setImageSize (Camera *cam, uint8_t x) {
  uint8_t args[] = { 0x05, 0x04, 0x01, 0x00, 0x19, x };
  return runCommand(cam, VC0706_WRITE_DATA, args, 5);
}

bool getMotionDetect (Camera *cam) {
  uint8_t args[] = { 0x0 };
  if (!runCommand(cam, VC0706_COMM_MOTION_STATUS, args, 6))
    return false;
  return cam->buff[5];
}

uint8_t getMotionStatus(Camera *cam, uint8_t x) {
  uint8_t args[] = { 0x01, x };
  return runCommand(cam, VC0706_MOTION_STATUS, args, 5);
}

bool motionDetected (Camera *cam) {
  if (readResponse(cam, 4, 200) != 4)
    return false;
  if (!verifyResponse(cam, VC0706_COMM_MOTION_DETECTED))
    return false;
  return true;
}

bool setMotionDetect (Camera *cam, bool flag) {
  if (!setMotionStatus(cam, VC0706_MOTIONCONTROL, VC0706_UARTMOTION, VC0706_ACTIVATEMOTION))
    return false;
  uint8_t args[] = { 0x1, flag };
  return runCommand(cam, VC0706_MOTION_STATUS, args, 5);
}

bool setMotionStatus (Camera *cam, uint8_t x, uint8_t d1, uint8_t d2) {
  uint8_t args[] = { 0x03, x, d1, d2 };
  return runCommand(cam, VC0706_MOTION_CTRL, args, 5);
}

uint8_t getCompression (Camera *cam) {
  uint8_t args[] = { 0x4, 0x1, 0x1, 0x12, 0x04 };
  runCommand(cam, VC0706_READ_DATA, args, 6);
  return cam->buff[5];
}

bool setCompression (Camera *cam, uint8_t c) {
  uint8_t args[] = { 0x5, 0x1, 0x1, 0x12, 0x04, c };
  return runCommand(cam, VC0706_WRITE_DATA, args, 5);
}

bool getPTZ(Camera *cam, uint16_t *w, uint16_t *h, uint16_t *wz, uint16_t *hz, uint16_t *pan, uint16_t *tilt) {
  uint8_t args[] = {0x0};

  if (!runCommand(cam, VC0706_GET_ZOOM, args, 16))
    return false;
  *w = cam->buff[5];
  *w <<= 8;
  *w |= cam->buff[6];

  *h = cam->buff[7];
  *h <<= 8;
  *h |= cam->buff[8];

  *wz = cam->buff[9];
  *wz <<= 8;
  *wz |= cam->buff[10];

  *hz = cam->buff[11];
  *hz <<= 8;
  *hz |= cam->buff[12];

  *pan = cam->buff[13];
  *pan <<= 8;
  *pan |= cam->buff[14];

  *tilt = cam->buff[15];
  *tilt <<= 8;
  *tilt |= cam->buff[16];

  return true;
}

bool setPTZ(Camera *cam, uint16_t wz, uint16_t hz, uint16_t pan, uint16_t tilt) {
  uint8_t args[] = {
    0x08, wz >> 8, wz,
    hz >> 8, wz,
    pan>>8, pan,
    tilt>>8, tilt
  };
  return !runCommand(cam, VC0706_SET_ZOOM, args, 5);
}

bool snapshotToFile (Camera *cam, char *path, uint8_t imgSize) {
  setImageSize(cam, imgSize);
  LE_INFO("Taking photo...");
  bool photoTaken = takePicture(cam);
  if (photoTaken) {
    LE_INFO("Photo taken");
    char writePath[100];
    // e.g /mnt/sd/<timestamp>.jpg
    sprintf(writePath, "%s/%d.jpg", path, (int)time(0));
    LE_INFO("Opening file pointer for path %s", writePath);
    FILE *filePtr = fopen(writePath, "w");
    if (filePtr != NULL) {
      LE_INFO("Got valid file pointer");
      int jpgLen = frameLength(cam);
      while (jpgLen > 0) {
        uint8_t *buff;
        uint8_t bytesToRead = 32 < jpgLen ? jpgLen : 32;
        buff = readPicture(cam, bytesToRead);
        fwrite(buff, sizeof(*buff), bytesToRead, filePtr);
        jpgLen -= bytesToRead;
      }
      fclose(filePtr);
      return true;
    }
    else {
      LE_ERROR("Invalid file pointer for %s", writePath);
      return false;
    }
  }
  else {
    LE_ERROR("Failed to take photo");
    return false;
  }
}
