# VC0706-cam-lib-legato
Library for the Adafruit VC0706 camera used on Legato devices. The majority of this code is ported from Adafruit's [Adafruit-VC0706-Serial-Camera-Library](https://github.com/adafruit/Adafruit-VC0706-Serial-Camera-Library) repository with some C/Legato specific changes.

## Project Status

Still incomplete and not functional. We are hoping to release 1.0.0 in the coming days.

## Usage on Other Platforms
The Legato serial API [`le_tty`](http://legato.io/legato-docs/latest/le__tty_8h.html) returns a file descriptor used to read and write. The file descriptor in the `Camera` struct can be replaced with any other file descriptor serial interface.

## API
```c
void delay (unsigned int msecs);
LE_SHARED int openCameraFd ();
LE_SHARED void sendCommand (Camera *cam, uint8_t cmd, uint8_t args[]);
LE_SHARED bool runCommand (Camera *cam, uint8_t cmd, uint8_t args[], int respLen, bool flushFlag);
LE_SHARED bool runCommandFlush (Camera *cam, uint8_t cmd, uint8_t args[], int respLen);
LE_SHARED uint8_t readResponse (Camera *cam, unsigned int nBytes, unsigned int timeout);
LE_SHARED void printBuffer (Camera *cam);
LE_SHARED bool verifyResponse (Camera *cam, uint8_t cmd);
LE_SHARED bool cameraFrameBuffCtrl (Camera *cam, uint8_t cmd);
LE_SHARED bool takePicture (Camera *cam);
LE_SHARED bool reset (Camera *cam);
LE_SHARED bool TVon (Camera *cam);
LE_SHARED bool TVOff (Camera *cam);
LE_SHARED uint8_t *readPicture (Camera *cam, uint8_t n);
LE_SHARED bool resumeVideo (Camera *cam);
LE_SHARED uint32_t frameLength (Camera *cam);
LE_SHARED char *getVersion (Camera *cam);
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
