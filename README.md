# VC0706-cam-lib-legato
Library for the Adafruit VC0706 camera used on Legato devices. The majority of this code is ported from Adafruit's [Adafruit-VC0706-Serial-Camera-Library](https://github.com/adafruit/Adafruit-VC0706-Serial-Camera-Library) repository with some C/Legato specific changes.

## Project Status

Still incomplete and not functional. We are hoping to release 1.0.0 in the coming days.

## Usage on Other Platforms
The Legato serial API [`le_tty`](http://legato.io/legato-docs/latest/le__tty_8h.html) returns a file descriptor used to read and write. The file descriptor in the `Camera` struct can be replaced with any other file descriptor serial interface.

## API
```c
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
