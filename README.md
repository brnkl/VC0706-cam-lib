# VC0706-cam-lib-legato
Library for the Adafruit VC0706 camera used on Legato devices. The majority of this code is ported from Adafruit's [Adafruit-VC0706-Serial-Camera-Library](https://github.com/adafruit/Adafruit-VC0706-Serial-Camera-Library) repository with some C/Legato specific changes.

## Project Status

Still incomplete and not functional. We are hoping to release 1.0.0 in the coming days.

## Usage on Other Platforms
The Legato serial API [`le_tty`](http://legato.io/legato-docs/latest/le__tty_8h.html) returns a file descriptor used to read and write. The file descriptor in the `Camera` struct can be replaced with any other file descriptor serial interface.

## API
```c
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
```

## Typedefs

### byte
```c
typedef uint8_t byte;
```

### Camera
```c
typedef struct {
  int fd; // file descriptor for the serial port
  byte serialNum; // camera serial number
  byte buff[BUFF_SIZE]; // byte array to store camera data
  int bufferLen; // current length of data in buffer
  uint16_t frameptr;
} Camera;

```
