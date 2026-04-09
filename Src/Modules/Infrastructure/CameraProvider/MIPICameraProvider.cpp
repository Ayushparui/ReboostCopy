/**
 * @file MIPICameraProvider.cpp
 *
 * This file implements a module that grabs images from the MIPI camera sensor.
 *
 * @author Thomas Röfer
 */

#include "MIPICameraProvider.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"

MAKE_MODULE(MIPICameraProvider);

MIPICameraProvider::MIPICameraProvider()
  : MIPIProvider(CameraInfo::color, imageTopic, cameraInfoTopic, useSystemTimeFlag) {}

void MIPICameraProvider::update(CameraImage& theCameraImage)
{
#ifdef TARGET_BOOSTER
  SYNC;
  theCameraImage.setResolution(cameraInfo.width / 2, cameraInfo.height);

  if(imageBuffer.empty())
    return;

  const int width = cameraInfo.width;
  const unsigned char* yBase = imageBuffer.data();
  const unsigned char* uvBase = yBase + theCameraImage.height * width;
  for(int y = 0; y < cameraInfo.height; ++y)
  {
    const unsigned char* yChannel = yBase + y * width;
    const unsigned char* uvChannel = uvBase + y / 2 * width;
    for(PixelTypes::YUYVPixel* p = theCameraImage[y], * pEnd = p + theCameraImage.width; p < pEnd; ++p)
    {
      p->y0 = *yChannel++;
      p->u = *uvChannel++;
      p->y1 = *yChannel++;
      p->v = *uvChannel++;
    }
  }

  theCameraImage.timestamp = lastImageTimestamp;
#else
  theCameraImage.setResolution(cameraInfo.width / 2, cameraInfo.height);
  theCameraImage.timestamp = Time::getCurrentSystemTime();
#endif
}

void MIPICameraProvider::update(JPEGImage& theJPEGImage)
{
  theJPEGImage.fromCameraImage(theCameraImage, jpegQuality);
}

bool MIPICameraProvider::isFrameDataComplete2() const
{
  return theCameraImage.timestamp != lastImageTimestamp;
}

void MIPICameraProvider::waitForFrameData2()
{
  const bool noImageYet = !theCameraImage.timestamp;
  if(waitForFrameData(noImageYet ? maxWaitForImage : minWaitForImage) && noImageYet)
    SystemCall::say("Camera ready");
}
