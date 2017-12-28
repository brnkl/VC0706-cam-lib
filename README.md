# VC0706-cam-lib-legato
Library for the Adafruit VC0706 camera used on Legato devices.

## Credit
The majority of this code is ported from Adafruit's [Adafruit-VC0706-Serial-Camera-Library](https://github.com/adafruit/Adafruit-VC0706-Serial-Camera-Library) repository with some C/Legato specific changes.

This is a library for the Adafruit TTL JPEG Camera (VC0706 chipset)

Pick one up today in the adafruit shop!
------> http://www.adafruit.com/products/397

These displays use Serial to communicate, 2 pins are required to interface

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Original code written by Limor Fried/Ladyada for Adafruit Industries.
BSD license, all text above must be included in any redistribution

## Releases

### 1.0.0
Released on December 16, 2017

View the full details here [here](https://github.com/brnkl/VC0706-cam-lib-legato/releases/tag/1.0.0)

## Example Usage
```c
#include "legato.h"
#include "interfaces.h"
#include "camera.h"

#define RETRY_WAIT_SEC 60

bool takePhoto (Camera *cam, char *dirPath) {
  cam->fd = fd_openCam();
  bool resetSucess = cam_reset(cam);
  LE_INFO("Camera reset %s", resetSucess ? "succeeded" : "failed");
  LE_DEBUG("Taking photo...");
  bool snapshotSuccess = cam_snapshotToFile(cam, dirPath, VC0706_640x480);
  LE_INFO("Snapshot %s", snapshotSuccess ? "succeeded" : "failed");
  fd_closeCam(cam->fd);
  return resetSucess && snapshotSuccess;
}

void photoLoop (Camera *cam, int intervalMintues, char *dirPath) {
  LE_INFO("Taking photos every %d minutes and storing them in %s", intervalMintues, dirPath);
  while (true) {
    bool success = takePhoto(cam, dirPath);
    int sleepDur = success ? intervalMintues * 60 : RETRY_WAIT_SEC;
    if (success) LE_INFO("Taking next photo in %d minutes", intervalMintues);
    else LE_INFO("Retrying after %d seconds", RETRY_WAIT_SEC);
    sleep(sleepDur);
  }
}

COMPONENT_INIT {
  Camera cam = {
    .serialNum = 0x00,
    .bufferLen = 0,
    .frameptr = 0,
  };
  photoLoop(&cam, 10, "/home/root/sd");
}
```

## API
```c
// File stream functions for reading photos
LE_SHARED bool cam_snapshotToFile (Camera *cam, const char *path, uint8_t imgSize, char *imgPath);
bool cam_readImageToFile (Camera *cam, const char *path, char *imgPath);
bool cam_readImageBlocks (Camera *cam, FILE *filePtr);
uint8_t cam_getImageBlockSize (int jpgLen);

// Higher level commands
LE_SHARED bool cam_setPTZ (Camera *cam, uint16_t wz, uint16_t hz, uint16_t pan, uint16_t tilt);
LE_SHARED bool cam_getPTZ (Camera *cam, uint16_t *w, uint16_t *h, uint16_t *wz, uint16_t *hz, uint16_t *pan, uint16_t *tilt);
LE_SHARED bool cam_setCompression (Camera *cam, uint8_t c);
LE_SHARED uint8_t cam_getCompression (Camera *cam);
LE_SHARED bool cam_setMotionStatus (Camera *cam, uint8_t x, uint8_t d1, uint8_t d2);
LE_SHARED bool cam_setMotionDetect (Camera *cam, bool flag);
LE_SHARED bool cam_motionDetected (Camera *cam);
LE_SHARED uint8_t cam_getMotionStatus (Camera *cam, uint8_t x);
LE_SHARED bool cam_getMotionDetect (Camera *cam);
LE_SHARED bool cam_setImageSize (Camera *cam,uint8_t x);
LE_SHARED uint8_t cam_getImageSize (Camera *cam);
LE_SHARED bool cam_setDownsize (Camera *cam, uint8_t newSize);
LE_SHARED uint8_t cam_getDownsize (Camera *cam);
LE_SHARED uint8_t cam_available (Camera *cam);
LE_SHARED char *cam_getVersion (Camera *cam);
LE_SHARED uint32_t cam_frameLength (Camera *cam);
LE_SHARED bool cam_resumeVideo (Camera *cam);
LE_SHARED uint8_t *cam_readPicture (Camera *cam, uint8_t n);
LE_SHARED bool cam_tvOff (Camera *cam);
LE_SHARED bool cam_tvOn(Camera *cam);
LE_SHARED bool cam_reset(Camera *cam);
LE_SHARED bool cam_takePicture(Camera *cam);
LE_SHARED bool cam_frameBuffCtrl(Camera *cam, uint8_t cmd);

// Low level camera commands
bool cam_runCommandFlush (Camera *cam, uint8_t cmd, uint8_t args[], uint8_t nArgs, uint8_t respLen);
bool cam_verifyResponse (Camera *cam, uint8_t cmd);
uint8_t cam_readResponse (Camera *cam, uint8_t nBytes, uint8_t timeout);
bool cam_runCommand (Camera *cam, uint8_t cmd, uint8_t args[], uint8_t nArgs, uint8_t respLen, bool flushFlag);
void cam_sendCommand (Camera *cam, uint8_t cmd, uint8_t args[], uint8_t nArgs);

// Serial/file descriptor helpers
int fd_dataAvail (int fd, int *data);
ssize_t fd_getByte (int fd, uint8_t *data);
LE_SHARED int fd_closeCam (int fd);
LE_SHARED int fd_openCam ();
int fd_openSerial (const char *device, int baud);
speed_t fd_convertBaud (int baud);
```

## Typedefs

### Camera
```c
typedef struct {
  int fd; // file descriptor for the serial port
  uint8_t serialNum; // camera serial number
  uint8_t buff[BUFF_SIZE]; // uint8_t array to store camera data
  uint8_t bufferLen; // current length of data in buffer
  uint16_t frameptr;
} Camera;
```

## Usage on Other Platforms
This code is highly decoupled from Legato with the exception of `LE_INFO` and `LE_DEBUG` log statements. In future releases this could be made more portable with pre-processor directives to redefine these macros.