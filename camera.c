#include "legato.h"
#include "interfaces.h"
#include "camera.h"
#include <time.h>

COMPONENT_INIT {
  Camera cam = {
    .fd = openCameraFd(),
    .serialNum = 0x0,
    .bufferLen = 0,
    .frameptr = 0
  };
  bool success = takePicture(&cam);
  LE_INFO("Number of bytes in buffer: %d", NUM_ARRAY_MEMBERS(cam.buff));
  LE_INFO("Did it work: %d", success);
}

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
void sendCommand (Camera *cam, byte cmd, byte args[]) {
  uint8_t init = VC0706_PREFIX;
  write(cam->fd, &init, 1); // send the cmd prefix
  write(cam->fd, &(cam->serialNum), 1); // send the serial number
  write(cam->fd, &cmd, 1); // send the cmd
  for(int i = 0; i < NUM_ARRAY_MEMBERS(args); i++) { // send each arg
    write(cam->fd, &args[i], 1);
  }
}

// Run a command
bool runCommand (Camera *cam, byte cmd, byte args[], int respLen) {
  sendCommand(cam, cmd, args);
  if (readResponse(cam, respLen, TIMEOUT) != respLen)
    return false;
  if (verifyResponse(cam, cmd))
    return false;
  else return true;
}

// Reads from the camera and returns how many bytes it read
int readResponse (Camera *cam, int nBytes, int timeout) {
  int counter = 0;
  int bufferLen = 0;
  // read while below timeout and while the buffer
  // is still smaller than expected
  while(timeout >= counter && cam->bufferLen <= nBytes) {
    ssize_t bytesRead = read(cam->fd, &(cam->buff) + cam->bufferLen, 1); // read one byte at a time
    bufferLen++;
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

bool verifyResponse (Camera *cam, byte cmd) {
  // If any of these are not equal than
  // the command failed
  return
    !(cam->buff[0] != VC0706_RESP_PREFIX ||
    cam->buff[1] != cam->serialNum ||
    cam->buff[2] != cmd ||
    cam->buff[3] != 0x0);
}

bool cameraFrameBuffCtrl (Camera *cam, byte cmd) {
  byte args[] = { 0x1, cmd };
  return runCommand(cam, cmd, args, 5);
}

bool takePicture (Camera *cam) {
  return cameraFrameBuffCtrl(cam, VC0706_STOPCURRENTFRAME);
}

bool reset (Camera *cam) {
  byte args[] = { 0x0 };
  return runCommand(cam, VC0706_RESET, args, 5);
}

size_t bufferToFile (Camera *cam, char *path) {
  FILE *photo = fopen(path, "w");
  if (photo != NULL) {
    LE_INFO("File pointer for %s is valid", path);
    size_t bytesWritten = fwrite(&(cam->buff), sizeof(cam->buff[0]), cam->bufferLen, photo);
    fclose(photo);
    return bytesWritten;
  }
  else {
    LE_INFO("File pointer for %s is invalid", path);
    return -1;
  }
}

// TODO implement these
bool TVon (Camera *cam);
bool TVOff (Camera *cam);
byte *readPicture (Camera *cam);
bool resumeVideo (Camera *cam);
uint32_t frameLength (Camera *cam);
char *getVersion (Camera *cam);
uint8_t available();
uint8_t getDownsize(void);
bool setDownsize(uint8_t);
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
