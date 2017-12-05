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

## Project Status

Still incomplete and not functional. We are currently experiencing some issues reading a complete image off the camera after taking a photo. The code is currently riddled with debug log statements but these will be cleaned up prior to release `1.0.0`

## Usage on Other Platforms
The Legato serial API [`le_tty`](http://legato.io/legato-docs/latest/le__tty_8h.html) returns a file descriptor used to read and write. The file descriptor in the `Camera` struct can be replaced with any other file descriptor serial interface.

## API
```c
LE_SHARED le_result_t openCameraFd (const char *path, int *fd, tty_Speed_t baud, int nBytes, int timeout);
LE_SHARED void sendCommand (Camera *cam, uint8_t cmd, uint8_t args[], uint8_t nArgs);
LE_SHARED bool runCommand (Camera *cam, uint8_t cmd, uint8_t args[], uint8_t nArgs, uint8_t respLen, bool flushFlag);
LE_SHARED bool runCommandFlush (Camera *cam, uint8_t cmd, uint8_t args[], uint8_t nArgs, uint8_t respLen);
LE_SHARED uint8_t readResponse (Camera *cam, uint8_t nBytes, uint8_t timeout);
LE_SHARED void printBuffer (Camera *cam);
LE_SHARED bool verifyResponse (Camera *cam, uint8_t cmd);
LE_SHARED bool cameraFrameBuffCtrl (Camera *cam, uint8_t cmd);
LE_SHARED bool takePicture (Camera *cam);
LE_SHARED bool reset (Camera *cam);
LE_SHARED bool TVon (Camera *cam);
LE_SHARED bool TVOff (Camera *cam);
LE_SHARED uint8_t* readPicture (Camera *cam, uint8_t n);
LE_SHARED bool resumeVideo (Camera *cam);
LE_SHARED uint32_t frameLength (Camera *cam);
LE_SHARED char* getVersion (Camera *cam);
LE_SHARED uint8_t available (Camera *cam);
LE_SHARED uint8_t getDownsize (Camera *cam);
LE_SHARED bool setDownsize(Camera *cam, uint8_t newSize);
LE_SHARED uint8_t getImageSize (Camera *cam);
LE_SHARED bool setImageSize (Camera *cam, uint8_t x);
LE_SHARED bool getMotionDetect (Camera *cam);
LE_SHARED uint8_t getMotionStatus(Camera *cam, uint8_t x);
LE_SHARED bool motionDetected (Camera *cam);
LE_SHARED bool setMotionDetect (Camera *cam, bool flag);
LE_SHARED bool setMotionStatus (Camera *cam, uint8_t x, uint8_t d1, uint8_t d2);
LE_SHARED uint8_t getCompression (Camera *cam);
LE_SHARED bool setCompression(Camera *cam, uint8_t c);
LE_SHARED bool getPTZ(Camera *cam, uint16_t *w, uint16_t *h, uint16_t *wz, uint16_t *hz, uint16_t *pan, uint16_t *tilt);
LE_SHARED bool setPTZ(Camera *cam, uint16_t wz, uint16_t hz, uint16_t pan, uint16_t tilt);
LE_SHARED bool snapshotToFile (Camera *cam, char *path, uint8_t imgSize);
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
