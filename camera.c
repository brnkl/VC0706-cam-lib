#include "camera.h"

COMPONENT_INIT { }

// File descriptor is returned in the fd pointer
le_result_t openCameraFd (const char *path, int *fd, tty_Speed_t baud, int nBytes, int timeout) {
  *fd = le_tty_Open(path, O_RDWR);
  le_result_t baudRes = le_tty_SetBaudRate(*fd, baud);
  le_result_t rawRes = le_tty_SetRaw(*fd, nBytes, timeout);
  return baudRes == LE_OK && rawRes == LE_OK ? LE_OK : LE_FAULT;
}

// tty_ functions mostly lifted from here
// https://github.com/oskarirauta/VC0706/blob/master/include/wiringSerial.c
ssize_t tty_putByte (int fd, uint8_t byte) {
  return write(fd, &byte, 1);
}

ssize_t tty_getByte (int fd, uint8_t *data) {
  return read(fd, data, 1);
}

int tty_dataAvail (int fd, int *data) {
  return ioctl(fd, FIONREAD, data);
}

void sendCommand (Camera *cam, uint8_t cmd, uint8_t args[], uint8_t nArgs) {
  tty_putByte(cam->fd, VC0706_PREFIX);
  tty_putByte(cam->fd, cam->serialNum);
  tty_putByte(cam->fd, cmd);
  for (int i = 0; i < nArgs; i++) {
    tty_putByte(cam->fd, args[i]);
  }
}

bool runCommand (Camera *cam, uint8_t cmd, uint8_t args[], uint8_t nArgs, uint8_t respLen, bool flushFlag) {
  // flush out the buffer
  if (flushFlag) {
    readResponse(cam, 100, 10);
  }
  sendCommand(cam, cmd, args, nArgs);
  if (readResponse(cam, respLen, 200) != respLen)
    return false;
  if (!verifyResponse(cam, cmd))
    return false;
  return true;
}

bool runCommandFlush (Camera *cam, uint8_t cmd, uint8_t args[], uint8_t nArgs, uint8_t respLen) {
  return runCommand (cam, cmd, args, nArgs, respLen, true);
}

// Reads from the camera and returns how many bytes it read
uint8_t readResponse (Camera *cam, uint8_t nBytes, uint8_t timeout) {
  uint8_t counter = 0;
  cam->bufferLen = 0;
  int avail;
  uint8_t data;
  while ((counter < timeout) && (cam->bufferLen < nBytes)) {
    LE_DEBUG("counter: %d, timeout: %d, nBytes: %d, bufferLen: %d", counter, timeout, nBytes, cam->bufferLen);
    // data is returned in avail pointer
    tty_dataAvail(cam->fd, &avail);
    // this case covers no data available
    if (avail <= 0) {
      usleep(1000);
      counter++;
      continue;
    }
    // this case is when we get data
    counter = 0;
    tty_getByte(cam->fd, &data);
    cam->buff[cam->bufferLen++] = data;
  }
  return cam->bufferLen;
}

void printBuffer (Camera *cam) {
  LE_DEBUG("Printing cam buffer");
  for(int i = 0; i < NUM_ARRAY_MEMBERS(cam->buff); i++) {
    LE_DEBUG("buff[%d]=%d", i, cam->buff[i]);
  }
}

bool verifyResponse (Camera *cam, uint8_t cmd) {
  // If any of these are not equal than
  // the command failed
  LE_DEBUG("buff[0] correct? %d", cam->buff[0] == VC0706_RESP_PREFIX);
  LE_DEBUG("buff[1] correct? %d", cam->buff[1] == cam->serialNum);
  LE_DEBUG("buff[2] correct? %d", cam->buff[2] == cmd);
  LE_DEBUG("buff[3] correct? %d", cam->buff[3] == 0x0);
  return
    cam->buff[0] == VC0706_RESP_PREFIX &&
    cam->buff[1] == cam->serialNum &&
    cam->buff[2] == cmd &&
    cam->buff[3] == 0x0;
}

bool cameraFrameBuffCtrl (Camera *cam, uint8_t cmd) {
  uint8_t args[] = { 0x1, cmd };
  return runCommandFlush(cam, VC0706_FBUF_CTRL, args, sizeof(args), 5);
}

bool takePicture (Camera *cam) {
  cam->frameptr = 0;
  return cameraFrameBuffCtrl(cam, VC0706_STOPCURRENTFRAME);
}

bool reset (Camera *cam) {
  uint8_t args[] = { 0x0 };
  return runCommandFlush(cam, VC0706_RESET, args, sizeof(args), 5);
}

bool TVon (Camera *cam) {
  uint8_t args[] = { 0x1, 0x1 };
  return runCommandFlush(cam, VC0706_TVOUT_CTRL, args, sizeof(args), 5);
}

bool TVOff (Camera *cam) {
  uint8_t args[] = { 0x1, 0x0 };
  return runCommandFlush(cam, VC0706_TVOUT_CTRL, args, sizeof(args), 5);
}

uint8_t* readPicture (Camera *cam, uint8_t n) {
  uint8_t args[] = { 0x0C, 0x0, 0x0A,
    0, 0, cam->frameptr >> 8, cam->frameptr & 0xFF,
    0, 0, 0, n,
    CAM_DELAY >> 8, CAM_DELAY & 0xFF };
  LE_DEBUG("frameptr: %d", cam->frameptr);
  if (!runCommand(cam, VC0706_READ_FBUF, args, sizeof(args), 5, false)) // don't flush
    return NULL;
  if (readResponse(cam, n + 5, CAM_DELAY) == 0)
    return NULL;

  cam->frameptr += n;

  return &(cam->buff[0]);
}

bool resumeVideo (Camera *cam) {
  return cameraFrameBuffCtrl(cam, VC0706_RESUMEFRAME);
}

uint32_t frameLength (Camera *cam) {
  uint8_t args[] = { 0x01, 0x00 };
  if (!runCommandFlush(cam, VC0706_GET_FBUF_LEN, args, sizeof(args), 9))
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

char* getVersion (Camera *cam) {
  uint8_t args[] = { 0x01 };
  sendCommand(cam, VC0706_GEN_VERSION, args, sizeof(args));
  if (!readResponse(cam, CAM_BUFF_SIZE, 200)) {
    LE_DEBUG("Failed to get version, returning empty string");
    return 0;
  }
  cam->buff[cam->bufferLen] = 0;
  return (char*)&(cam->buff[0]);
}

uint8_t available (Camera *cam) {
  return cam->bufferLen;
}

uint8_t getDownsize (Camera *cam) {
  uint8_t args[] = { 0x0 };
  if (!runCommandFlush(cam, VC0706_DOWNSIZE_STATUS, args, sizeof(args), 6))
    return -1;
  return cam->buff[5];
}

bool setDownsize (Camera *cam, uint8_t newSize) {
  uint8_t args[] = { 0x01, newSize };
  return runCommandFlush(cam, VC0706_DOWNSIZE_CTRL, args, sizeof(args), 5);
}

uint8_t getImageSize (Camera *cam) {
  uint8_t args[] = { 0x4, 0x4, 0x1, 0x00, 0x19 };
  if (!runCommandFlush(cam, VC0706_READ_DATA, args, sizeof(args), 6))
    return -1;
  return cam->buff[5];
}

bool setImageSize (Camera *cam, uint8_t x) {
  uint8_t args[] = { 0x05, 0x04, 0x01, 0x00, 0x19, x };
  return runCommandFlush(cam, VC0706_WRITE_DATA, args, sizeof(args), 5);
}

bool getMotionDetect (Camera *cam) {
  uint8_t args[] = { 0x0 };
  if (!runCommandFlush(cam, VC0706_COMM_MOTION_STATUS, args, sizeof(args), 6))
    return false;
  return cam->buff[5];
}

uint8_t getMotionStatus(Camera *cam, uint8_t x) {
  uint8_t args[] = { 0x01, x };
  return runCommandFlush(cam, VC0706_MOTION_STATUS, args, sizeof(args), 5);
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
  return runCommandFlush(cam, VC0706_MOTION_STATUS, args, sizeof(args), 5);
}

bool setMotionStatus (Camera *cam, uint8_t x, uint8_t d1, uint8_t d2) {
  uint8_t args[] = { 0x03, x, d1, d2 };
  return runCommandFlush(cam, VC0706_MOTION_CTRL, args, sizeof(args), 5);
}

uint8_t getCompression (Camera *cam) {
  uint8_t args[] = { 0x4, 0x1, 0x1, 0x12, 0x04 };
  runCommandFlush(cam, VC0706_READ_DATA, args, sizeof(args), 6);
  return cam->buff[5];
}

bool setCompression (Camera *cam, uint8_t c) {
  uint8_t args[] = { 0x5, 0x1, 0x1, 0x12, 0x04, c };
  return runCommandFlush(cam, VC0706_WRITE_DATA, args, sizeof(args), 5);
}

bool getPTZ(Camera *cam, uint16_t *w, uint16_t *h, uint16_t *wz, uint16_t *hz, uint16_t *pan, uint16_t *tilt) {
  uint8_t args[] = { 0x0 };

  if (!runCommandFlush(cam, VC0706_GET_ZOOM, args, sizeof(args), 16))
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

bool setPTZ (Camera *cam, uint16_t wz, uint16_t hz, uint16_t pan, uint16_t tilt) {
  uint8_t args[] = {
    0x08, wz >> 8, wz,
    hz >> 8, wz,
    pan >> 8, pan,
    tilt >> 8, tilt
  };
  return !runCommandFlush(cam, VC0706_SET_ZOOM, args, sizeof(args), 5);
}

bool snapshotToFile (Camera *cam, char *path, uint8_t imgSize) {
  setImageSize(cam, imgSize);
  LE_INFO("Taking photo...");
  bool success = true;
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
        uint8_t bytesToRead = CAM_BLOCK_SIZE < jpgLen ? CAM_BLOCK_SIZE : jpgLen;
        LE_DEBUG("jpgLen: %d, bytesToRead: %d", jpgLen, bytesToRead);
        buff = readPicture(cam, bytesToRead);
        if (buff == NULL) {
          LE_ERROR("Failed to read image data");
          success = false;
          break;
        }
        fwrite(buff, sizeof(*buff), bytesToRead, filePtr);
        jpgLen -= bytesToRead;
      }
      fclose(filePtr);
      return success;
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
