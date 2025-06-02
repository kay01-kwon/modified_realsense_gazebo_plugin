/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include "realsense_gazebo_plugin/RealSensePlugin.h"
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>

#define DEPTH_SCALE_M 0.001

#define DEPTH_CAMERA_TOPIC "depth"
#define COLOR_CAMERA_TOPIC "color"
#define IRED1_CAMERA_TOPIC "infrared"
#define IRED2_CAMERA_TOPIC "infrared2"

using namespace gazebo;

/////////////////////////////////////////////////
RealSensePlugin::RealSensePlugin() {
  this->depthCam = nullptr;
  this->ired1Cam = nullptr;
  this->ired2Cam = nullptr;
  this->colorCam = nullptr;
  this->prefix = "";
  this->pointCloudCutOffMax_ = 5.0;
}

/////////////////////////////////////////////////
RealSensePlugin::~RealSensePlugin() {}

/////////////////////////////////////////////////
void RealSensePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Output the name of the model
  std::cout
      << std::endl
      << "RealSensePlugin: The realsense_camera plugin is attach to model "
      << _model->GetName() << std::endl;

  cameraParamsMap_.insert(std::make_pair(COLOR_CAMERA_NAME, CameraParams()));
  cameraParamsMap_.insert(std::make_pair(DEPTH_CAMERA_NAME, CameraParams()));
  cameraParamsMap_.insert(std::make_pair(IRED1_CAMERA_NAME, CameraParams()));
  cameraParamsMap_.insert(std::make_pair(IRED2_CAMERA_NAME, CameraParams()));

  if(!_sdf->HasElement("depthUpdateRate"))
  {
    this->depthUpdateRate_ = 30.0;
    printf("Depth Update Rate not set, using default: %f\n", this->depthUpdateRate_);
  }
  else{
    _sdf->GetElement("depthUpdateRate")->GetValue()->Get(this->depthUpdateRate_);
    printf("Depth Update Rate: %f\n", this->depthUpdateRate_);
  }

  if(!_sdf->HasElement("colorUpdateRate"))
  {
    this->colorUpdateRate_ = 30.0;
    printf("Color Update Rate not set, using default: %f\n", this->colorUpdateRate_);
  }
  else{
    _sdf->GetElement("colorUpdateRate")->GetValue()->Get(this->colorUpdateRate_);
    printf("Color Update Rate: %f\n", this->colorUpdateRate_);
  }

  if(!_sdf->HasElement("infraredUpdateRate"))
  {
    this->infraredUpdateRate_ = 30.0;
    printf("Infrared Update Rate not set, using default: %f\n", this->infraredUpdateRate_);
  }
  else{
    _sdf->GetElement("infraredUpdateRate")->GetValue()->Get(this->infraredUpdateRate_);
    printf("Infrared Update Rate: %f\n", this->infraredUpdateRate_);
  }

  if(!_sdf->HasElement("depthTopicName"))
  {
    cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name = "/camera/depth/image_rect_raw";
    printf("Depth Topic Name not set, using default: %s\n",
           cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name.c_str());
  }
  else{
    _sdf->GetElement("depthTopicName")->GetValue()->Get(
        cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name);
    printf("Depth Topic Name: %s\n",
           cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name.c_str());
  }

  if(!_sdf->HasElement("depthCameraInfoTopicName"))
  {
    cameraParamsMap_[DEPTH_CAMERA_NAME].camera_info_topic_name =
        "/camera/depth/camera_info";
    printf("Depth Camera Info Topic Name not set, using default: %s\n",
           cameraParamsMap_[DEPTH_CAMERA_NAME].camera_info_topic_name.c_str());
  }
  else{
    _sdf->GetElement("depthCameraInfoTopicName")->GetValue()->Get(
        cameraParamsMap_[DEPTH_CAMERA_NAME].camera_info_topic_name);
    printf("Depth Camera Info Topic Name: %s\n",
           cameraParamsMap_[DEPTH_CAMERA_NAME].camera_info_topic_name.c_str());
  }

  if(!_sdf->HasElement("colorTopicName"))
  {
    cameraParamsMap_[COLOR_CAMERA_NAME].topic_name = "/camera/color/image_rect_raw";
    printf("Color Topic Name not set, using default: %s\n",
           cameraParamsMap_[COLOR_CAMERA_NAME].topic_name.c_str());
  }
  else{
    _sdf->GetElement("colorTopicName")->GetValue()->Get(
        cameraParamsMap_[COLOR_CAMERA_NAME].topic_name);
    printf("Color Topic Name: %s\n",
           cameraParamsMap_[COLOR_CAMERA_NAME].topic_name.c_str());
  }

  if(!_sdf->HasElement("colorCameraInfoTopicName"))
  {
    cameraParamsMap_[COLOR_CAMERA_NAME].camera_info_topic_name =
        "/camera/color/camera_info";
    printf("Color Camera Info Topic Name not set, using default: %s\n",
           cameraParamsMap_[COLOR_CAMERA_NAME].camera_info_topic_name.c_str());
  }
  else{
    _sdf->GetElement("colorCameraInfoTopicName")->GetValue()->Get(
        cameraParamsMap_[COLOR_CAMERA_NAME].camera_info_topic_name);
    printf("Color Camera Info Topic Name: %s\n",
           cameraParamsMap_[COLOR_CAMERA_NAME].camera_info_topic_name.c_str());
  }

  if(!_sdf->HasElement("infrared1TopicName"))
  {
    cameraParamsMap_[IRED1_CAMERA_NAME].topic_name = "/camera/infra1/image_rect_raw";
    printf("Infrared1 Topic Name not set, using default: %s\n",
           cameraParamsMap_[IRED1_CAMERA_NAME].topic_name.c_str());
  }
  else{
    _sdf->GetElement("infrared1TopicName")->GetValue()->Get(
        cameraParamsMap_[IRED1_CAMERA_NAME].topic_name);
    printf("Infrared1 Topic Name: %s\n",
           cameraParamsMap_[IRED1_CAMERA_NAME].topic_name.c_str());
  }

  if(!_sdf->HasElement("infrared1CameraInfoTopicName"))
  {
    cameraParamsMap_[IRED1_CAMERA_NAME].camera_info_topic_name =
        "/camera/infra1/camera_info";
    printf("Infrared1 Camera Info Topic Name not set, using default: %s\n",
           cameraParamsMap_[IRED1_CAMERA_NAME].camera_info_topic_name.c_str());
  }
  else{
    _sdf->GetElement("infrared1CameraInfoTopicName")->GetValue()->Get(
        cameraParamsMap_[IRED1_CAMERA_NAME].camera_info_topic_name);
    printf("Infrared1 Camera Info Topic Name: %s\n",
           cameraParamsMap_[IRED1_CAMERA_NAME].camera_info_topic_name.c_str());
  }

  if(!_sdf->HasElement("infrared2TopicName"))
  {
    cameraParamsMap_[IRED2_CAMERA_NAME].topic_name = "/camera/infra2/image_rect_raw";
    printf("Infrared2 Topic Name not set, using default: %s\n",
           cameraParamsMap_[IRED2_CAMERA_NAME].topic_name.c_str());
  }
  else{
    _sdf->GetElement("infrared2TopicName")->GetValue()->Get(
        cameraParamsMap_[IRED2_CAMERA_NAME].topic_name);
    printf("Infrared2 Topic Name: %s\n",
           cameraParamsMap_[IRED2_CAMERA_NAME].topic_name.c_str());
  }

  if(!_sdf->HasElement("infrared2CameraInfoTopicName"))
  {
    cameraParamsMap_[IRED2_CAMERA_NAME].camera_info_topic_name =
        "/camera/infra2/camera_info";
    printf("Infrared2 Camera Info Topic Name not set, using default: %s\n",
           cameraParamsMap_[IRED2_CAMERA_NAME].camera_info_topic_name.c_str());
  }
  else{
    _sdf->GetElement("infrared2CameraInfoTopicName")->GetValue()->Get(
        cameraParamsMap_[IRED2_CAMERA_NAME].camera_info_topic_name);
    printf("Infrared2 Camera Info Topic Name: %s\n",
           cameraParamsMap_[IRED2_CAMERA_NAME].camera_info_topic_name.c_str());
  }

  if(!_sdf->HasElement("colorOpticalframeName"))
  {
    cameraParamsMap_[COLOR_CAMERA_NAME].optical_frame = "camera_color_optical_frame";
    printf("Color Optical Frame not set, using default: %s\n",
           cameraParamsMap_[COLOR_CAMERA_NAME].optical_frame.c_str());
  }
  else{
    _sdf->GetElement("colorOpticalframeName")->GetValue()->Get(
        cameraParamsMap_[COLOR_CAMERA_NAME].optical_frame);
    printf("Color Optical Frame: %s\n",
           cameraParamsMap_[COLOR_CAMERA_NAME].optical_frame.c_str());
  }

  if(!_sdf->HasElement("depthOpticalframeName"))
  {
    cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame = "camera_depth_optical_frame";
    printf("Depth Optical Frame not set, using default: %s\n",
           cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame.c_str());
  }
  else{
    _sdf->GetElement("depthOpticalframeName")->GetValue()->Get(
        cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame);
    printf("Depth Optical Frame: %s\n",
           cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame.c_str());
  }

  if(!_sdf->HasElement("infrared1OpticalframeName"))
  {
    cameraParamsMap_[IRED1_CAMERA_NAME].optical_frame = "camera_infra1_optical_frame";
    printf("Infrared1 Optical Frame not set, using default: %s\n",
           cameraParamsMap_[IRED1_CAMERA_NAME].optical_frame.c_str());
  }
  else{
    _sdf->GetElement("infrared1OpticalframeName")->GetValue()->Get(
        cameraParamsMap_[IRED1_CAMERA_NAME].optical_frame);
    printf("Infrared1 Optical Frame: %s\n",
           cameraParamsMap_[IRED1_CAMERA_NAME].optical_frame.c_str());
  }

  if(!_sdf->HasElement("infrared2OpticalframeName"))
  {
    cameraParamsMap_[IRED2_CAMERA_NAME].optical_frame = "camera_infra2_optical_frame";
    printf("Infrared2 Optical Frame not set, using default: %s\n",
           cameraParamsMap_[IRED2_CAMERA_NAME].optical_frame.c_str());
  }
  else{
    _sdf->GetElement("infrared2OpticalframeName")->GetValue()->Get(
        cameraParamsMap_[IRED2_CAMERA_NAME].optical_frame);
    printf("Infrared2 Optical Frame: %s\n",
           cameraParamsMap_[IRED2_CAMERA_NAME].optical_frame.c_str());
  }

  if(!_sdf->HasElement("rangeMinDepth"))
  {
    this->rangeMinDepth_ = 0.020;
    printf("Range Min Depth not set, using default: %f\n", this->rangeMinDepth_);
  }
  else{
    _sdf->GetElement("rangeMinDepth")->GetValue()->Get(this->rangeMinDepth_);
    printf("Range Min Depth: %f\n", this->rangeMinDepth_);
  }

  if (!_sdf->HasElement("rangeMaxDepth"))
  {
    this->rangeMaxDepth_ = 10.0;
    printf("Range Max Depth not set, using default: %f\n", this->rangeMaxDepth_);
  }
  else{
    _sdf->GetElement("rangeMaxDepth")->GetValue()->Get(this->rangeMaxDepth_);
    printf("Range Max Depth: %f\n", this->rangeMaxDepth_);
  }

  if(!_sdf->HasElement("pointCloud"))
  {
    this->pointCloud_ = false;
    printf("Point Cloud not set, using default: %d\n", this->pointCloud_);
  }
  else{
    _sdf->GetElement("pointCloud")->GetValue()->Get(this->pointCloud_);
    printf("Point Cloud: %d\n", this->pointCloud_);
  }

  if(!_sdf->HasElement("pointCloudTopicName"))
  {
    this->pointCloudTopic_ = "/camera/color/points";
    printf("Point Cloud Topic Name not set, using default: %s\n",
           this->pointCloudTopic_.c_str());
  }
  else{
    _sdf->GetElement("pointCloudTopicName")->GetValue()->Get(
        this->pointCloudTopic_);
    printf("Point Cloud Topic Name: %s\n",
           this->pointCloudTopic_.c_str());
  }

  if(!_sdf->HasElement("pointCloudCutoff"))
  {
    this->pointCloudCutOff_ = 0.5;
    printf("Point Cloud Cutoff not set, using default: %f\n", this->pointCloudCutOff_);
  }
  else{
    _sdf->GetElement("pointCloudCutoff")->GetValue()->Get(this->pointCloudCutOff_);
    printf("Point Cloud Cutoff: %f\n", this->pointCloudCutOff_);
  }

  if(!_sdf->HasElement("pointCloudCutoffMax"))
  {
    this->pointCloudCutOffMax_ = 5.0;
    printf("Point Cloud Cutoff Max not set, using default: %f\n",
           this->pointCloudCutOffMax_);
  }
  else{
    _sdf->GetElement("pointCloudCutoffMax")->GetValue()->Get(
        this->pointCloudCutOffMax_);
    printf("Point Cloud Cutoff Max: %f\n",
           this->pointCloudCutOffMax_);
  }

  if(!_sdf->HasElement("noiseBase"))
  {
    this->noiseBase_ = 0.003;
    printf("Noise Base not set, using default: %f\n", this->noiseBase_);
  }
  else{
    _sdf->GetElement("noiseBase")->GetValue()->Get(this->noiseBase_);
    printf("Noise Base: %f\n", this->noiseBase_);
  }

  if(!_sdf->HasElement("noiseScale"))
  {
    this->noiseScale_ = 0.001;
    printf("Noise Scale not set, using default: %f\n", this->noiseScale_);
  }
  else{
    _sdf->GetElement("noiseScale")->GetValue()->Get(this->noiseScale_);
    printf("Noise Scale: %f\n", this->noiseScale_);
  }

  if(!_sdf->HasElement("prefix"))
  {
    this->prefix = _model->GetName() + "_";
    printf("Prefix not set, using default: %s\n", this->prefix.c_str());
  }
  else{
    _sdf->GetElement("prefix")->GetValue()->Get(this->prefix);
    printf("Prefix: %s\n", this->prefix.c_str());
  }

  // Store a pointer to the this model
  this->rsModel = _model;

  // Store a pointer to the world
  this->world = this->rsModel->GetWorld();

  // Sensors Manager
  sensors::SensorManager *smanager = sensors::SensorManager::Instance();

  // Get Cameras Renderers
  this->depthCam = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(
                       smanager->GetSensor(prefix + DEPTH_CAMERA_NAME))
                       ->DepthCamera();

  this->ired1Cam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                       smanager->GetSensor(prefix + IRED1_CAMERA_NAME))
                       ->Camera();
  this->ired2Cam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                       smanager->GetSensor(prefix + IRED2_CAMERA_NAME))
                       ->Camera();
  this->colorCam = std::dynamic_pointer_cast<sensors::CameraSensor>(
                       smanager->GetSensor(prefix + COLOR_CAMERA_NAME))
                       ->Camera();

  // Check if camera renderers have been found successfuly
  if (!this->depthCam) {
    std::cerr << "RealSensePlugin: Depth Camera has not been found"
              << std::endl;
    return;
  }
  if (!this->ired1Cam) {
    std::cerr << "RealSensePlugin: InfraRed Camera 1 has not been found"
              << std::endl;
    return;
  }
  if (!this->ired2Cam) {
    std::cerr << "RealSensePlugin: InfraRed Camera 2 has not been found"
              << std::endl;
    return;
  }
  if (!this->colorCam) {
    std::cerr << "RealSensePlugin: Color Camera has not been found"
              << std::endl;
    return;
  }

  // Resize Depth Map dimensions
  try {
    this->depthMap.resize(this->depthCam->ImageWidth() *
                          this->depthCam->ImageHeight());
  } catch (std::bad_alloc &e) {
    std::cerr << "RealSensePlugin: depthMap allocation failed: " << e.what()
              << std::endl;
    return;
  }

  // Setup Transport Node
  this->transportNode = transport::NodePtr(new transport::Node());
  this->transportNode->Init(this->world->Name());

  // Setup Publishers
  std::string rsTopicRoot = "~/" + this->rsModel->GetName();

  this->depthPub = this->transportNode->Advertise<msgs::ImageStamped>(
      rsTopicRoot + DEPTH_CAMERA_TOPIC, 1, depthUpdateRate_);
  this->ired1Pub = this->transportNode->Advertise<msgs::ImageStamped>(
      rsTopicRoot + IRED1_CAMERA_TOPIC, 1, infraredUpdateRate_);
  this->ired2Pub = this->transportNode->Advertise<msgs::ImageStamped>(
      rsTopicRoot + IRED2_CAMERA_TOPIC, 1, infraredUpdateRate_);
  this->colorPub = this->transportNode->Advertise<msgs::ImageStamped>(
      rsTopicRoot + COLOR_CAMERA_TOPIC, 1, colorUpdateRate_);

  // Listen to depth camera new frame event
  this->newDepthFrameConn = this->depthCam->ConnectNewDepthFrame(
      std::bind(&RealSensePlugin::OnNewDepthFrame, this));

  this->newIred1FrameConn = this->ired1Cam->ConnectNewImageFrame(std::bind(
      &RealSensePlugin::OnNewFrame, this, this->ired1Cam, this->ired1Pub));

  this->newIred2FrameConn = this->ired2Cam->ConnectNewImageFrame(std::bind(
      &RealSensePlugin::OnNewFrame, this, this->ired2Cam, this->ired2Pub));

  this->newColorFrameConn = this->colorCam->ConnectNewImageFrame(std::bind(
      &RealSensePlugin::OnNewFrame, this, this->colorCam, this->colorPub));

  // Listen to the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&RealSensePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void RealSensePlugin::OnNewFrame(const rendering::CameraPtr cam,
                                 const transport::PublisherPtr pub) {
  msgs::ImageStamped msg;

  // Set Simulation Time
  msgs::Set(msg.mutable_time(), this->world->SimTime());

  // Set Image Dimensions
  msg.mutable_image()->set_width(cam->ImageWidth());
  msg.mutable_image()->set_height(cam->ImageHeight());

  // Set Image Pixel Format
  msg.mutable_image()->set_pixel_format(
      common::Image::ConvertPixelFormat(cam->ImageFormat()));

  // Set Image Data
  msg.mutable_image()->set_step(cam->ImageWidth() * cam->ImageDepth());
  msg.mutable_image()->set_data(cam->ImageData(),
                                cam->ImageDepth() * cam->ImageWidth() *
                                    cam->ImageHeight());

  // Publish realsense infrared stream
  pub->Publish(msg);
}

/////////////////////////////////////////////////
void RealSensePlugin::OnNewDepthFrame() {
  // Get Depth Map dimensions
  unsigned int imageSize =
      this->depthCam->ImageWidth() * this->depthCam->ImageHeight();

  // Instantiate message
  msgs::ImageStamped msg;

  // Convert Float depth data to RealSense depth data
  const float *depthDataFloat = this->depthCam->DepthData();
  for (unsigned int i = 0; i < imageSize; ++i) {
    // Check clipping and overflow
    if (depthDataFloat[i] < rangeMinDepth_ ||
        depthDataFloat[i] > rangeMaxDepth_ ||
        depthDataFloat[i] > DEPTH_SCALE_M * UINT16_MAX ||
        depthDataFloat[i] < 0) {
      this->depthMap[i] = 0;
    } else {
      // Add noise to depth data
      float std_dev = noiseBase_ + noiseScale_*(depthDataFloat[i] * depthDataFloat[i]);
      
      noise_ = noiseDist_(gen_) * std_dev;
      this->depthMap[i] = (uint16_t)((depthDataFloat[i] + noise_)/ DEPTH_SCALE_M);
    }
  }

  // Pack realsense scaled depth map
  msgs::Set(msg.mutable_time(), this->world->SimTime());
  msg.mutable_image()->set_width(this->depthCam->ImageWidth());
  msg.mutable_image()->set_height(this->depthCam->ImageHeight());
  msg.mutable_image()->set_pixel_format(common::Image::L_INT16);
  msg.mutable_image()->set_step(this->depthCam->ImageWidth() *
                                this->depthCam->ImageDepth());
  msg.mutable_image()->set_data(this->depthMap.data(),
                                sizeof(*this->depthMap.data()) * imageSize);

  // Publish realsense scaled depth map
  this->depthPub->Publish(msg);
}

/////////////////////////////////////////////////
void RealSensePlugin::OnUpdate() {}
