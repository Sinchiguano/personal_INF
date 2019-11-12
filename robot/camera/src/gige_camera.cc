#include <bitset>
#include <iomanip>
#include <iostream>
#include <limits.h>
#include <stdlib.h>

#include "Common.h"
#include "smcs_cpp/IImageBitmap.h"
//#include <gige_cpp/IImageBitmap.h>

#include <opencv2/opencv.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "gige_camera.h"

namespace gige_camera {

smcs::ICameraAPI initGigeApi() {
  smcs::InitCameraAPI();
  smcs::ICameraAPI gigeApi = smcs::GetCameraAPI();
  if (!gigeApi->IsUsingKernelDriver()) {
    std::cout << "Warning: Smartek Filter Driver not loaded." << std::endl;
  }
  return gigeApi;
}

smcs::IDevice getZeroDevice(smcs::ICameraAPI &gigeApi) {
  gigeApi->FindAllDevices(3.0);
  smcs::DevicesList devices = gigeApi->GetAllDevices();
  if (devices.empty()) {
    std::cout << "Can not find any camera " << std::endl;
    return 0;
  }
  smcs::IDevice device = devices[0];
  if (device != NULL && device->Connect()) {
    std::cout << "Connected to " << device->GetModelName()
              << " camera with IP number "
              << Common::IpAddrToString(device->GetIpAddress()) << std::endl;
    INT64 newPacketSize = 1500;
    newPacketSize = newPacketSize & 0xFFFF;
    device->SetIntegerNodeValue("GevSCPSPacketSize", newPacketSize);
    INT64 packetSize = 0;
    device->GetIntegerNodeValue("GevSCPSPacketSize", packetSize);
    packetSize = packetSize & 0xFFFF;
    std::cout << "PacketSize: " << packetSize << std::endl;

    // enable trigger mode
    bool status = device->SetStringNodeValue("TriggerMode", "On");
    // set software trigger mode
    status = device->SetStringNodeValue("TriggerSource", "Software");
    // set continuous acquisition mode
    status = device->SetStringNodeValue("AcquisitionMode", "Continuous");
    // set trigger selector to frame start
    status = device->SetStringNodeValue("TriggerSelector", "FrameStart");
    // start acquisition
    status = device->SetIntegerNodeValue("TLParamsLocked", 1);
    status = device->CommandNodeExecute("AcquisitionStart");

    std::string value;
    status = device->GetStringNodeValue("AcquisitionMode", value);
    std::cout << "AcquisitionMode set to " << value << std::endl;
    status = device->GetStringNodeValue("TriggerMode", value);
    std::cout << "TriggerMode set to " << value << std::endl;
    status = device->GetStringNodeValue("TriggerSource", value);
    std::cout << "TriggerSource set to " << value << std::endl;

    //lower the resolution
    status = device->SetIntegerNodeValue("BinningHorizontal", 2);
    status = device->SetIntegerNodeValue("BinningVertical", 2);

    status = device->SetFloatNodeValue("ExposureTime",    15000.00);
    // status = device->SetFloatNodeValue("ExposureTime", 1300000.00);
  } else {
    std::cout << "Can not connect to " << device->GetModelName()
              << " camera with IP number "
              << Common::IpAddrToString(device->GetIpAddress())
              << " press any key to exit." << std::endl;
  }
  return device;
}

void Camera::set_exposure_time(double val) {
  device->SetFloatNodeValue("ExposureTime", val);
}

Camera::Camera() : gigeApi(initGigeApi()) {
  if (gigeApi) {
    device = getZeroDevice(gigeApi);
  }
}

Camera::~Camera() {
  device->Disconnect();
  smcs::ExitCameraAPI();
}

void getImageWait() {}

cv::Mat Camera::getImage() {
  cv::Mat img;

  bool hot_fix = true;

  if (hot_fix){
    //TEMPORAL FIX
    //Clear image buffer
    while (!device->IsBufferEmpty()) {
      smcs::IImageInfo imageInfo = NULL;
      device->GetImageInfo(&imageInfo);
      //img = copyToImg((smcs::IImageBitmap)imageInfo);
      // remove (pop) image from image buffer
      device->PopImage(imageInfo);
      std::cout << "REMOVED OBSOLETE IMAGE FROM BUFFER" << std::endl;
    }
    
    std::cout << "GIGA - trigger" << std::endl;
    // trigger and wait for image
    device->CommandNodeExecute("TriggerSoftware");
    int repeat = 1;
    while (device->IsBufferEmpty()) {
      if(repeat %20000 == 0){
        std::cout << "GIGA - trigger repeat" << std::endl;
        device->CommandNodeExecute("TriggerSoftware");
      }
      repeat++;
      usleep(5);
    }
  }
  else{
  std::cout << "GIGA - trigger" << std::endl;
  while (device->IsBufferEmpty()) {
    usleep(5);
  }
  }

  if (!device->IsBufferEmpty()) {
    std::cout << "GIGA - taking_image" << std::endl;
    smcs::IImageInfo imageInfo = NULL;
    device->GetImageInfo(&imageInfo);
    std::cout << "GIGA 1" << std::endl;
    img = copyToImg((smcs::IImageBitmap)imageInfo);
    std::cout << "GIGA 2" << std::endl;
    // remove (pop) image from image buffer
    device->PopImage(imageInfo);
  }

  std::cout << "GIGA - img_taken" << std::endl;

  return img;
}

cv::Mat Camera::copyToImg(const smcs::IImageBitmapInterface src) {
  // if (&dest == NULL)
  // return;

  UINT32 srcPixelType;
  UINT32 srcWidth, srcHeight;

  src.GetPixelType(srcPixelType);
  src.GetSize(srcWidth, srcHeight);

  cv::Mat img(cv::Size(srcHeight, srcWidth), CV_8UC1);

  for (unsigned int i = 0; i < (unsigned int)srcHeight; i++) {
    const UINT8 *lineData = src.GetRawData(i);
    std::bitset<8> pixbit(*(lineData));
    // std::cout << pixbit << std::endl;
    // UINT32 lineSize = src.GetLineSize();
    // std::cout << "line size " << lineSize << std::endl;
    for (unsigned int j = 0; j < (unsigned int)srcWidth; j++) {
      img.at<uchar>(cv::Point(i, j)) = *(lineData + j);
    }
  }

  return img;
}

} // namespace gige_camera
