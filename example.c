#include "camera.h"
#include "interfaces.h"
#include "legato.h"

#define RETRY_WAIT_SEC 60

bool takePhoto(Camera* cam, char* dirPath) {
  cam->fd = fd_openCam();
  bool resetSucess = cam_reset(cam);
  LE_INFO("Camera reset %s", resetSucess ? "succeeded" : "failed");
  LE_DEBUG("Taking photo...");
  bool snapshotSuccess = cam_snapshotToFile(cam, dirPath, VC0706_640x480);
  LE_INFO("Snapshot %s", snapshotSuccess ? "succeeded" : "failed");
  fd_closeCam(cam->fd);
  return resetSucess && snapshotSuccess;
}

void photoLoop(Camera* cam, int intervalMintues, char* dirPath) {
  LE_INFO("Taking photos every %d minutes and storing them in %s",
          intervalMintues, dirPath);
  while (true) {
    bool success = takePhoto(cam, dirPath);
    int sleepDur = success ? intervalMintues * 60 : RETRY_WAIT_SEC;
    if (success)
      LE_INFO("Taking next photo in %d minutes", intervalMintues);
    else
      LE_INFO("Retrying after %d seconds", RETRY_WAIT_SEC);
    sleep(sleepDur);
  }
}

COMPONENT_INIT {
  Camera cam = {
      .serialNum = 0x00, .bufferLen = 0, .frameptr = 0,
  };
  photoLoop(&cam, 10, "/home/root/sd");
}