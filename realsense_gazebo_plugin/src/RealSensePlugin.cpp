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
std::ostream &print_param(std::ostream &os,
const std::string &param_name,
const double param_value,
const bool is_default)
{
  if(is_default)
  {
    os << "[RealsensePlugin] " << param_name << " not set, using default: "
       << param_value << std::endl;
  }
  else
  {
    os << "[RealsensePlugin] " << param_name << ": " << param_value << std::endl;
  }

  return os;
}

std::ostream &print_param(std::ostream &os,
const std::string &param_name,
const float param_value,
const bool is_default)
{
  if(is_default)
  {
    os << "[RealsensePlugin] " << param_name << " not set, using default: "
       << param_value << std::endl;
  }
  else
  {
    os << "[RealsensePlugin] " << param_name << ": " << param_value << std::endl;
  }

  return os;
}

std::ostream &print_param(std::ostream &os,
const std::string &param_name,
const bool param_value,
const bool is_default)
{
  if(is_default)
  {
    os << "[RealsensePlugin] " << param_name << " not set, using default: "
       << std::boolalpha << param_value << std::endl;
  }
  else
  {
    os << "[RealsensePlugin] " << param_name << ": " << std::boolalpha
       << param_value << std::endl;
  }
  return os;
}

std::ostream &print_param(std::ostream &os,
const std::string &param_name,
const std::string &param_value,
const bool is_default)
{
  if(is_default)
  {
    os << "[RealsensePlugin] " << param_name << " not set, using default: "
       << param_value << std::endl;
  }
  else
  {
    os << "[RealsensePlugin] " << param_name << ": " << param_value << std::endl;
  }
  return os;
}


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

  printf("************************************************\n");
  printf("RealSensePlugin: Loading parameters from SDF\n");

  if(!_sdf->HasElement("depthUpdateRate"))
  {
    this->depthUpdateRate_ = 30.0;
    print_param(std::cout, "Depth Update Rate", this->depthUpdateRate_, true);
  }
  else{
    this->depthUpdateRate_ =
     _sdf->Get<double>("depthUpdateRate");
    print_param(std::cout, "Depth Update Rate", this->depthUpdateRate_, false);
  }

  if(!_sdf->HasElement("colorUpdateRate"))
  {
    this->colorUpdateRate_ = 30.0;
    print_param(std::cout, "Color Update Rate", this->colorUpdateRate_, true);
  }
  else{
    this->colorUpdateRate_ = 
    _sdf->Get<double>("colorUpdateRate");
    print_param(std::cout, "Color Update Rate", this->colorUpdateRate_, false);
  }

  if(!_sdf->HasElement("infraredUpdateRate"))
  {
    this->infraredUpdateRate_ = 30.0;
    print_param(std::cout, "Infrared Update Rate", this->infraredUpdateRate_, true);
  }
  else{
    this->infraredUpdateRate_ = 
    _sdf->Get<double>("infraredUpdateRate");
    print_param(std::cout, "Infrared Update Rate", this->infraredUpdateRate_, false);
  }

  if(!_sdf->HasElement("depthTopicName"))
  {
    cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name = "/camera/depth/image_rect_raw";
    print_param(std::cout, "Depth Topic Name",
                cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name, true);
  }
  else{
    _sdf->GetElement("depthTopicName")->GetValue()->Get<std::string>(
        cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name);
    print_param(std::cout, "Depth Topic Name",
                cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name, false);
  }

  if(!_sdf->HasElement("depthCameraInfoTopicName"))
  {
    cameraParamsMap_[DEPTH_CAMERA_NAME].camera_info_topic_name =
        "/camera/depth/camera_info";
    print_param(std::cout, "Depth Camera Info Topic Name",
                cameraParamsMap_[DEPTH_CAMERA_NAME].camera_info_topic_name, true);
  }
  else{
    _sdf->GetElement("depthCameraInfoTopicName")->GetValue()->Get<std::string>(
        cameraParamsMap_[DEPTH_CAMERA_NAME].camera_info_topic_name);
    print_param(std::cout, "Depth Camera Info Topic Name",
                cameraParamsMap_[DEPTH_CAMERA_NAME].camera_info_topic_name, false);
  }

  if(!_sdf->HasElement("colorTopicName"))
  {
    cameraParamsMap_[COLOR_CAMERA_NAME].topic_name = "/camera/color/image_rect_raw";
    print_param(std::cout, "Color Topic Name",
                cameraParamsMap_[COLOR_CAMERA_NAME].topic_name, true);
  }
  else{
    _sdf->GetElement("colorTopicName")->GetValue()->Get<std::string>(
        cameraParamsMap_[COLOR_CAMERA_NAME].topic_name);
    print_param(std::cout, "Color Topic Name",
                cameraParamsMap_[COLOR_CAMERA_NAME].topic_name, false);
  }

  if(!_sdf->HasElement("colorCameraInfoTopicName"))
  {
    cameraParamsMap_[COLOR_CAMERA_NAME].camera_info_topic_name =
        "/camera/color/camera_info";
    print_param(std::cout, "Color Camera Info Topic Name",
                cameraParamsMap_[COLOR_CAMERA_NAME].camera_info_topic_name, true);
  }
  else{
    _sdf->GetElement("colorCameraInfoTopicName")->GetValue()->Get<std::string>(
        cameraParamsMap_[COLOR_CAMERA_NAME].camera_info_topic_name);
    print_param(std::cout, "Color Camera Info Topic Name",
                cameraParamsMap_[COLOR_CAMERA_NAME].camera_info_topic_name, false);
  }

  if(!_sdf->HasElement("infrared1TopicName"))
  {
    cameraParamsMap_[IRED1_CAMERA_NAME].topic_name = "/camera/infra1/image_rect_raw";
    print_param(std::cout, "Infrared1 Topic Name",
                cameraParamsMap_[IRED1_CAMERA_NAME].topic_name, true);
  }
  else{
    _sdf->GetElement("infrared1TopicName")->GetValue()->Get<std::string>(
        cameraParamsMap_[IRED1_CAMERA_NAME].topic_name);
    print_param(std::cout, "Infrared1 Topic Name",
                cameraParamsMap_[IRED1_CAMERA_NAME].topic_name, false);
  }

  if(!_sdf->HasElement("infrared1CameraInfoTopicName"))
  {
    cameraParamsMap_[IRED1_CAMERA_NAME].camera_info_topic_name =
        "/camera/infra1/camera_info";
    print_param(std::cout, "Infrared1 Camera Info Topic Name",
                cameraParamsMap_[IRED1_CAMERA_NAME].camera_info_topic_name, true);
  }
  else{
    _sdf->GetElement("infrared1CameraInfoTopicName")->GetValue()->Get<std::string>(
        cameraParamsMap_[IRED1_CAMERA_NAME].camera_info_topic_name);
    print_param(std::cout, "Infrared1 Camera Info Topic Name",
                cameraParamsMap_[IRED1_CAMERA_NAME].camera_info_topic_name, false);
  }

  if(!_sdf->HasElement("infrared2TopicName"))
  {
    cameraParamsMap_[IRED2_CAMERA_NAME].topic_name = "/camera/infra2/image_rect_raw";
    print_param(std::cout, "Infrared2 Topic Name",
                cameraParamsMap_[IRED2_CAMERA_NAME].topic_name, true);
  }
  else{
    _sdf->GetElement("infrared2TopicName")->GetValue()->Get<std::string>(
        cameraParamsMap_[IRED2_CAMERA_NAME].topic_name);
    print_param(std::cout, "Infrared2 Topic Name",
                cameraParamsMap_[IRED2_CAMERA_NAME].topic_name, false);
  }

  if(!_sdf->HasElement("infrared2CameraInfoTopicName"))
  {
    cameraParamsMap_[IRED2_CAMERA_NAME].camera_info_topic_name =
        "/camera/infra2/camera_info";
    print_param(std::cout, "Infrared2 Camera Info Topic Name",
                cameraParamsMap_[IRED2_CAMERA_NAME].camera_info_topic_name, true);
  }
  else{
    _sdf->GetElement("infrared2CameraInfoTopicName")->GetValue()->Get<std::string>(
        cameraParamsMap_[IRED2_CAMERA_NAME].camera_info_topic_name);
    print_param(std::cout, "Infrared2 Camera Info Topic Name",
                cameraParamsMap_[IRED2_CAMERA_NAME].camera_info_topic_name, false);
  }

  if(!_sdf->HasElement("colorOpticalframeName"))
  {
    cameraParamsMap_[COLOR_CAMERA_NAME].optical_frame = "camera_color_optical_frame";
    print_param(std::cout, "Color Optical Frame",
                cameraParamsMap_[COLOR_CAMERA_NAME].optical_frame, true);
  }
  else{
    _sdf->GetElement("colorOpticalframeName")->GetValue()->Get<std::string>(
        cameraParamsMap_[COLOR_CAMERA_NAME].optical_frame);
    print_param(std::cout, "Color Optical Frame",
                cameraParamsMap_[COLOR_CAMERA_NAME].optical_frame, false);
  }

  if(!_sdf->HasElement("depthOpticalframeName"))
  {
    cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame = "camera_depth_optical_frame";
    print_param(std::cout, "Depth Optical Frame",
                cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame, true);
  }
  else{
    _sdf->GetElement("depthOpticalframeName")->GetValue()->Get<std::string>(
        cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame);
    print_param(std::cout, "Depth Optical Frame",
                cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame, false);
  }

  if(!_sdf->HasElement("infrared1OpticalframeName"))
  {
    cameraParamsMap_[IRED1_CAMERA_NAME].optical_frame = "camera_infra1_optical_frame";
    print_param(std::cout, "Infrared1 Optical Frame",
                cameraParamsMap_[IRED1_CAMERA_NAME].optical_frame, true);
  }
  else{
    _sdf->GetElement("infrared1OpticalframeName")->GetValue()->Get(
        cameraParamsMap_[IRED1_CAMERA_NAME].optical_frame);
    print_param(std::cout, "Infrared1 Optical Frame",
                cameraParamsMap_[IRED1_CAMERA_NAME].optical_frame, false);
  }

  if(!_sdf->HasElement("infrared2OpticalframeName"))
  {
    cameraParamsMap_[IRED2_CAMERA_NAME].optical_frame = "camera_infra2_optical_frame";
    print_param(std::cout, "Infrared2 Optical Frame",
                cameraParamsMap_[IRED2_CAMERA_NAME].optical_frame, true);
  }
  else{
    _sdf->GetElement("infrared2OpticalframeName")->GetValue()->Get(
        cameraParamsMap_[IRED2_CAMERA_NAME].optical_frame);
    print_param(std::cout, "Infrared2 Optical Frame",
                cameraParamsMap_[IRED2_CAMERA_NAME].optical_frame, false);
  }

  if(!_sdf->HasElement("rangeMinDepth"))
  {
    this->rangeMinDepth_ = 0.020f;
    print_param(std::cout, "Range Min Depth", this->rangeMinDepth_, true);
  }
  else{
    _sdf->GetElement("rangeMinDepth")->GetValue()->Get<float>(this->rangeMinDepth_);
    print_param(std::cout, "Range Min Depth", this->rangeMinDepth_, false);
  }

  if (!_sdf->HasElement("rangeMaxDepth"))
  {
    this->rangeMaxDepth_ = 10.0f;
    print_param(std::cout, "Range Max Depth", this->rangeMaxDepth_, true);
  }
  else{
    _sdf->GetElement("rangeMaxDepth")->GetValue()->Get<float>(this->rangeMaxDepth_);
    print_param(std::cout, "Range Max Depth", this->rangeMaxDepth_, false);
  }

  if(!_sdf->HasElement("pointCloud"))
  {
    this->pointCloud_ = false;
    print_param(std::cout, "Point Cloud", this->pointCloud_, true);
  }
  else{
    _sdf->GetElement("pointCloud")->GetValue()->Get(this->pointCloud_);
    print_param(std::cout, "Point Cloud", this->pointCloud_, false);
  }

  if(!_sdf->HasElement("pointCloudTopicName"))
  {
    this->pointCloudTopic_ = "/camera/color/points";
    print_param(std::cout, "Point Cloud Topic Name",
                this->pointCloudTopic_, true);
  }
  else{
    _sdf->GetElement("pointCloudTopicName")->GetValue()->Get(
        this->pointCloudTopic_);
    print_param(std::cout, "Point Cloud Topic Name",
                this->pointCloudTopic_, false);
  }

  if(!_sdf->HasElement("pointCloudCutoff"))
  {
    this->pointCloudCutOff_ = 0.5;
    print_param(std::cout, "Point Cloud Cutoff",
                this->pointCloudCutOff_, true);
  }
  else{
    _sdf->GetElement("pointCloudCutoff")->GetValue()->Get<double>(this->pointCloudCutOff_);
    print_param(std::cout, "Point Cloud Cutoff",
                this->pointCloudCutOff_, false);
  }

  if(!_sdf->HasElement("pointCloudCutoffMax"))
  {
    this->pointCloudCutOffMax_ = 5.0;
    print_param(std::cout, "Point Cloud Cutoff Max",
                this->pointCloudCutOffMax_, true);
  }
  else{
    _sdf->GetElement("pointCloudCutoffMax")->GetValue()->Get<double>(
        this->pointCloudCutOffMax_);
    print_param(std::cout, "Point Cloud Cutoff Max",
                this->pointCloudCutOffMax_, false);
  }

  if(!_sdf->HasElement("noiseBase"))
  {
    this->noiseBase_ = 0.003;
    print_param(std::cout, "Noise Base", this->noiseBase_, true);
  }
  else{
    _sdf->GetElement("noiseBase")->GetValue()->Get<float>(this->noiseBase_);
    print_param(std::cout, "Noise Base", this->noiseBase_, false);
  }

  if(!_sdf->HasElement("noiseScale"))
  {
    this->noiseScale_ = 0.001;
    print_param(std::cout, "Noise Scale", this->noiseScale_, true);
  }
  else{
    _sdf->GetElement("noiseScale")->GetValue()->Get<float>(this->noiseScale_);
    print_param(std::cout, "Noise Scale", this->noiseScale_, false);
  }

  if(!_sdf->HasElement("prefix"))
  {
    this->prefix = _model->GetName() + "_";
    print_param(std::cout, "Prefix", this->prefix, true);
  }
  else{
    _sdf->GetElement("prefix")->GetValue()->Get(this->prefix);
    print_param(std::cout, "Prefix", this->prefix, false);
  }

  // Store a pointer to the this model
  this->rsModel = _model;

  // Store a pointer to the world
  this->world = this->rsModel->GetWorld();

  std::cout
      << "RealSensePlugin: Model name: " << this->rsModel->GetName() 
      << std::endl;

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
