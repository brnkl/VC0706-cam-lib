#include "camera.h"
#include "legato.h"
#include <termios.h>
#include "util.h"

COMPONENT_INIT {}

/**
 * Open a serial connection to the camera
 */
int fd_openCam(char* devPath) {
  return fd_openSerial(devPath, CAM_BAUD_RATE);
}

/**
 * Close the serial connection to the camera
 */
int fd_closeCam(int fd) {
  return close(fd);
}

/**
 * Read a byte from fd into data
 *
 * TODO This sub-routine could be removed
 * (see cam_readResponse comment)
 */
ssize_t fd_getByte(int fd, uint8_t* data) {
  return read(fd, data, 1);
}

/**
 * Send a command sequence to cam
 * Should not be called directly (use cam_runCommand)
 */
void cam_sendCommand(Camera* cam, uint8_t cmd, uint8_t args[], uint8_t nArgs) {
  int start = 3;
  int end = nArgs + start;
  uint8_t* toWrite = malloc(end);
  toWrite[0] = VC0706_PREFIX;
  toWrite[1] = cam->serialNum;
  toWrite[2] = cmd;
  for (int i = start; i < end; i++) {
    toWrite[i] = args[i - start];
  };
  write(cam->fd, toWrite, end);
  free(toWrite);
}

/**
 * Run a command described by cmd and args on cam
 */
bool cam_runCommand(Camera* cam,
                    uint8_t cmd,
                    uint8_t args[],
                    uint8_t nArgs,
                    uint8_t respLen,
                    bool flushFlag) {
  if (flushFlag) {
    cam_readResponse(cam, 100, 10);
  }
  cam_sendCommand(cam, cmd, args, nArgs);
  if (cam_readResponse(cam, respLen, 200) != respLen)
    return false;
  if (!cam_verifyResponse(cam, cmd))
    return false;
  return true;
}

/**
 * Flush buffer before running command
 */
bool cam_runCommandFlush(Camera* cam,
                         uint8_t cmd,
                         uint8_t args[],
                         uint8_t nArgs,
                         uint8_t respLen) {
  return cam_runCommand(cam, cmd, args, nArgs, respLen, true);
}

/**
 * Read a response from cam
 *
 * TODO This sub-routine reads 1 byte at a time
 * (this was useful for debugging), but this is
 * not required. I performance could be
 * improved by reading `avail` bytes
 * at a time
 */
uint8_t cam_readResponse(Camera* cam, uint8_t nBytes, uint8_t timeout) {
  uint8_t counter = 0;
  cam->bufferLen = 0;
  int avail = 0;
  uint8_t data;
  while ((counter < timeout) && (cam->bufferLen < nBytes)) {
    // available data is returned in avail pointer
    bool availSuccess = fd_dataAvail(cam->fd, &avail) == 0;
    // this case covers no data available
    // or when fd_dataAvail fails
    if (avail <= 0 || !availSuccess) {
      usleep(1000);
      counter++;
      continue;
    }
    // this case is when we get data
    counter = 0;
    ssize_t bytesRead = fd_getByte(cam->fd, &data);
    if (bytesRead > 0)
      cam->buff[cam->bufferLen++] = data;
  }
  return cam->bufferLen;
}

/**
 * Verify that a given response is valid
 */
bool cam_verifyResponse(Camera* cam, uint8_t cmd) {
  // If any of these are not equal than
  // the command failed
  return cam->buff[0] == VC0706_RESP_PREFIX && cam->buff[1] == cam->serialNum &&
         cam->buff[2] == cmd && cam->buff[3] == 0x0;
}

/**
 * Helper for commands related
 * to controlling the frame buffer
 */
bool cam_frameBuffCtrl(Camera* cam, uint8_t cmd) {
  uint8_t args[] = {0x1, cmd};
  return cam_runCommandFlush(cam, VC0706_FBUF_CTRL, args, sizeof(args), 5);
}

/**
 * Stop the frame to take a photo
 */
bool cam_takePicture(Camera* cam) {
  cam->frameptr = 0;
  return cam_frameBuffCtrl(cam, VC0706_STOPCURRENTFRAME);
}

/**
 * Reset cam
 */
bool cam_reset(Camera* cam) {
  uint8_t args[] = {0x0};
  bool runRes = cam_runCommandFlush(cam, VC0706_RESET, args, sizeof(args), 5);
  sleep(2);  // wait for camera to come back online
  return runRes;
}

/**
 * Enable the analog TV out
 */
bool cam_tvOn(Camera* cam) {
  uint8_t args[] = {0x1, 0x1};
  return cam_runCommandFlush(cam, VC0706_TVOUT_CTRL, args, sizeof(args), 5);
}

/**
 * Disable the analog TV out
 */
bool cam_tvOff(Camera* cam) {
  uint8_t args[] = {0x1, 0x0};
  return cam_runCommandFlush(cam, VC0706_TVOUT_CTRL, args, sizeof(args), 5);
}

/**
 * Request a block of size n data from cam
 */
uint8_t* cam_readPicture(Camera* cam, uint8_t n) {
  uint8_t args[] = {0x0C,
                    0x0,
                    0x0A,
                    0,
                    0,
                    cam->frameptr >> 8,
                    cam->frameptr & 0xFF,
                    0,
                    0,
                    0,
                    n,
                    CAM_DELAY >> 8,
                    CAM_DELAY & 0xFF};
  if (!cam_runCommand(cam, VC0706_READ_FBUF, args, sizeof(args), 5,
                      false))  // don't flush
    return NULL;
  if (cam_readResponse(cam, n + 5, CAM_DELAY) == 0)
    return NULL;
  cam->frameptr += n;

  return &cam->buff[0];
}

/**
 * Resume frame capture
 */
bool cam_resumeVideo(Camera* cam) {
  return cam_frameBuffCtrl(cam, VC0706_RESUMEFRAME);
}

/**
 * Get the length of the frame
 * currently on cam
 */
uint32_t cam_frameLength(Camera* cam) {
  uint8_t args[] = {0x01, 0x00};
  if (!cam_runCommandFlush(cam, VC0706_GET_FBUF_LEN, args, sizeof(args), 9))
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

/**
 * Get the version from cam
 */
char* cam_getVersion(Camera* cam) {
  uint8_t args[] = {0x00};
  cam_sendCommand(cam, VC0706_GEN_VERSION, args, sizeof(args));
  if (!cam_readResponse(cam, CAM_BUFF_SIZE, 200))
    return NULL;
  cam->buff[cam->bufferLen] = 0;
  return (char*)&cam->buff[5];
}

/**
 * Get the length of cam's buffer
 */
uint8_t cam_available(Camera* cam) {
  return cam->bufferLen;
}

/**
 * Get cam's current downsize
 */
uint8_t cam_getDownsize(Camera* cam) {
  uint8_t args[] = {0x0};
  if (!cam_runCommandFlush(cam, VC0706_DOWNSIZE_STATUS, args, sizeof(args), 6))
    return -1;
  return cam->buff[5];
}

/**
 * Set cam's downsize
 */
bool cam_setDownsize(Camera* cam, uint8_t newSize) {
  uint8_t args[] = {0x01, newSize};
  return cam_runCommandFlush(cam, VC0706_DOWNSIZE_CTRL, args, sizeof(args), 5);
}

/**
 * Get cam's current image size
 */
uint8_t cam_getImageSize(Camera* cam) {
  uint8_t args[] = {0x4, 0x4, 0x1, 0x00, 0x19};
  if (!cam_runCommandFlush(cam, VC0706_READ_DATA, args, sizeof(args), 6))
    return -1;
  return cam->buff[5];
}

/**
 * Set cam's image size to x
 */
bool cam_setImageSize(Camera* cam, uint8_t x) {
  uint8_t args[] = {0x05, 0x04, 0x01, 0x00, 0x19, x};
  return cam_runCommandFlush(cam, VC0706_WRITE_DATA, args, sizeof(args), 5);
}

bool cam_getMotionDetect(Camera* cam) {
  uint8_t args[] = {0x0};
  if (!cam_runCommandFlush(cam, VC0706_COMM_MOTION_STATUS, args, sizeof(args),
                           6))
    return false;
  return cam->buff[5];
}

uint8_t cam_getMotionStatus(Camera* cam, uint8_t x) {
  uint8_t args[] = {0x01, x};
  return cam_runCommandFlush(cam, VC0706_MOTION_STATUS, args, sizeof(args), 5);
}

/**
 * Return true if cam detected motion
 */
bool cam_motionDetected(Camera* cam) {
  if (cam_readResponse(cam, 4, 200) != 4)
    return false;
  if (!cam_verifyResponse(cam, VC0706_COMM_MOTION_DETECTED))
    return false;
  return true;
}

/**
 * Enable/disable motion detection on cam
 */
bool cam_setMotionDetect(Camera* cam, bool flag) {
  if (!cam_setMotionStatus(cam, VC0706_MOTIONCONTROL, VC0706_UARTMOTION,
                           VC0706_ACTIVATEMOTION))
    return false;
  uint8_t args[] = {0x1, flag};
  return cam_runCommandFlush(cam, VC0706_MOTION_STATUS, args, sizeof(args), 5);
}

bool cam_setMotionStatus(Camera* cam, uint8_t x, uint8_t d1, uint8_t d2) {
  uint8_t args[] = {0x03, x, d1, d2};
  return cam_runCommandFlush(cam, VC0706_MOTION_CTRL, args, sizeof(args), 5);
}

/**
 * Get cam's current level of compression
 */
uint8_t cam_getCompression(Camera* cam) {
  uint8_t args[] = {0x4, 0x1, 0x1, 0x12, 0x04};
  cam_runCommandFlush(cam, VC0706_READ_DATA, args, sizeof(args), 6);
  return cam->buff[5];
}

/**
 * Set cam's level of compression to c
 */
bool cam_setCompression(Camera* cam, uint8_t c) {
  uint8_t args[] = {0x5, 0x1, 0x1, 0x12, 0x04, c};
  return cam_runCommandFlush(cam, VC0706_WRITE_DATA, args, sizeof(args), 5);
}

/**
 * Get cam's current PTZ values
 * All values returned in pointers
 */
bool cam_getPTZ(Camera* cam,
                uint16_t* w,
                uint16_t* h,
                uint16_t* wz,
                uint16_t* hz,
                uint16_t* pan,
                uint16_t* tilt) {
  uint8_t args[] = {0x0};

  if (!cam_runCommandFlush(cam, VC0706_GET_ZOOM, args, sizeof(args), 16))
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

/**
 * Set cam's PTZ settings
 */
bool cam_setPTZ(Camera* cam,
                uint16_t wz,
                uint16_t hz,
                uint16_t pan,
                uint16_t tilt) {
  uint8_t args[] = {0x08,     wz >> 8, wz,        hz >> 8, wz,
                    pan >> 8, pan,     tilt >> 8, tilt};
  return !cam_runCommandFlush(cam, VC0706_SET_ZOOM, args, sizeof(args), 5);
}

/**
 * Return how much data should be requested
 * from cam based on jpgLen
 */
uint8_t cam_getImageBlockSize(int jpgLen) {
  return CAM_BLOCK_SIZE < jpgLen ? CAM_BLOCK_SIZE : jpgLen;
}

/**
 * Read the entire frame buffer from cam
 * into filePtr
 */
bool cam_readImageBlocks(Camera* cam, FILE* filePtr) {
  int jpgLen = cam_frameLength(cam);
  int imgSize = jpgLen;
  int nWrites = 0;
  while (jpgLen > 0) {
    uint8_t bytesToRead = cam_getImageBlockSize(jpgLen);
    uint8_t* buff = cam_readPicture(cam, bytesToRead);
    if (buff == NULL) {
      LE_ERROR("Failed to read image data");
      return false;
    }
    fwrite(buff, sizeof(*buff), bytesToRead, filePtr);
    // give progress every 30 writes
    if (++nWrites % 30 == 0) {
      double percentComplete =
          ((double)imgSize - jpgLen) * 100.0 / (double)imgSize;
      LE_INFO("Image write %f%% complete", percentComplete);
    }
    jpgLen -= bytesToRead;
  }
  return true;
}

/**
 * Read the image from cam into a file
 * described by path
 */
bool cam_readImageToFile(Camera* cam, const char* path, char* imgPath) {
  bool success = false;
  // e.g /mnt/sd/<timestamp>.jpg
  sprintf(imgPath, "%s/%d.jpg", path, (int)time(0));
  LE_INFO("Opening file pointer for path %s", imgPath);
  FILE* filePtr = fopen(imgPath, "w");
  if (filePtr != NULL) {
    LE_INFO("File pointer valid");
    success = cam_readImageBlocks(cam, filePtr);
    if (success) {
      LE_INFO("Successfully wrote image to %s", imgPath);
    } else {
      LE_INFO("Failed to write photo data to %s", imgPath);
    }
    fclose(filePtr);
  } else {
    LE_ERROR("Invalid file pointer for %s", imgPath);
  }
  return success;
}

/**
 * Take a photo on cam and write it to a file
 * described by path
 *
 * The path to the new image is returned
 * in imgPath (assuming true is returned)
 */
bool cam_snapshotToFile(Camera* cam,
                        const char* path,
                        uint8_t imgSize,
                        char* imgPath) {
  cam_setImageSize(cam, imgSize);
  LE_INFO("Taking photo...");
  bool photoTaken = cam_takePicture(cam);
  if (photoTaken) {
    LE_INFO("Photo taken");
    bool res = cam_readImageToFile(cam, path, imgPath);
    cam_resumeVideo(cam);
    return res;
  } else {
    LE_ERROR("Failed to take photo");
    return false;
  }
}
