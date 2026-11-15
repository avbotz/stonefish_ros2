/*    
    This file is a part of stonefish_ros2.

    stonefish_ros2 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    stonefish_ros2 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  ROS2SimulationManager.cpp
//  stonefish_ros2
//
//  Created by Patryk Cieslak on 02/10/23.
//  Copyright (c) 2023-2026 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros2/ROS2SimulationManager.h"
#include <algorithm>
#include "stonefish_ros2/ROS2ScenarioParser.h"
#include "stonefish_ros2/ROS2Interface.h"
#include "stonefish_ros2/ROS2SensorPluginInterface.h"
#include "stonefish_ros2/ROS2ActuatorPluginInterface.h"
#include "stonefish_ros2/ROS2CommPluginInterface.h"

#include "stonefish_ros2/msg/thruster_state.hpp"
#include "stonefish_ros2/msg/debug_physics.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "vision_msgs/msg/label_info.hpp"

#include <Stonefish/entities/animation/ManualTrajectory.h>
#include <Stonefish/entities/forcefields/Uniform.h>
#include <Stonefish/entities/forcefields/Jet.h>
#include <Stonefish/joints/FixedJoint.h>
#include <Stonefish/sensors/scalar/Pressure.h>
#include <Stonefish/sensors/scalar/DVL.h>
#include <Stonefish/sensors/scalar/Accelerometer.h>
#include <Stonefish/sensors/scalar/Gyroscope.h>
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/sensors/scalar/GPS.h>
#include <Stonefish/sensors/scalar/ForceTorque.h>
#include <Stonefish/sensors/scalar/RotaryEncoder.h>
#include <Stonefish/sensors/scalar/Odometry.h>
#include <Stonefish/sensors/vision/ColorCamera.h>
#include <Stonefish/sensors/vision/DepthCamera.h>
#include <Stonefish/sensors/vision/ThermalCamera.h>
#include <Stonefish/sensors/vision/OpticalFlowCamera.h>
#include <Stonefish/sensors/vision/SegmentationCamera.h>
#include <Stonefish/sensors/vision/EventBasedCamera.h>
#include <Stonefish/sensors/scalar/Multibeam.h>
#include <Stonefish/sensors/vision/Lidar.h>
#include <Stonefish/sensors/vision/FLS.h>
#include <Stonefish/sensors/vision/SSS.h>
#include <Stonefish/sensors/vision/MSIS.h>
#include <Stonefish/sensors/Contact.h>
#include <Stonefish/comms/USBL.h>
#include <Stonefish/comms/OpticalModem.h>
#include <Stonefish/actuators/Push.h>
#include <Stonefish/actuators/SimpleThruster.h>
#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/actuators/Propeller.h>
#include <Stonefish/actuators/Rudder.h>
#include <Stonefish/actuators/SuctionCup.h>
#include <Stonefish/actuators/Motor.h>
#include <Stonefish/actuators/DCMotor.h>
#include <Stonefish/actuators/Servo.h>
#include <Stonefish/actuators/VariableBuoyancy.h>
#include <Stonefish/actuators/Light.h>
#include <Stonefish/core/Robot.h>

using namespace std::placeholders;

namespace sf
{

static bool IsLinkActuator(Actuator* act, LinkActuatorType type)
{
    return act->GetType() == ActuatorType::LINK && static_cast<LinkActuator*>(act)->GetLinkActuatorType() == type;
}

static bool IsJointActuator(Actuator* act, JointActuatorType type)
{
    return act->GetType() == ActuatorType::JOINT && static_cast<JointActuator*>(act)->GetJointActuatorType() == type;
}

ROS2SimulationManager::ROS2SimulationManager(Scalar stepsPerSecond, std::string scenarioFilePath, const std::shared_ptr<rclcpp::Node>& nh)
	: SimulationManager(stepsPerSecond, Solver::SI, CollisionFilter::EXCLUSIVE), scenarioPath_(scenarioFilePath), nh_(nh),
      stateDivider_(1), stepCounter_(0), stopImageThread_(false)
{
    //Simulation clock (similar to Gazebo): physics is paced by the wall clock scaled by the realtime factor,
    //and optionally the simulation time is published on /clock and used to stamp all messages
    double rtf = nh_->declare_parameter("realtime_factor", 1.0);
    if(rtf > 0.0)
        SetRealtimeFactor(rtf);
    else
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Parameter 'realtime_factor' has to be positive! Using 1.0.");

    if(nh_->declare_parameter("publish_clock", false))
    {
        clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        if(rcl_enable_ros_time_override(clock_->get_clock_handle()) != RCL_RET_OK
           || rcl_set_ros_time_override(clock_->get_clock_handle(), 0) != RCL_RET_OK)
            throw std::runtime_error("Failed to initialize the simulation clock!");
        clockPub_ = nh_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    }
    else
        clock_ = nh_->get_clock();

    //Repeatable simulation: the same random numbers in every run and every frame of the cameras rendered at the
    //step in which it is due and published, however slow the computer is (the simulation waits for the renderer)
    int64_t seed = nh_->declare_parameter("seed", (int64_t)-1);
    if(seed >= 0)
        SimulationManager::SetRandomSeed((unsigned int)seed);
    lockStep_ = nh_->declare_parameter("lock_step", false);
    if(nh_->declare_parameter("start_paused", false))
        SetPaused(true);

    //Robot state (TF, joints, actuators) and /clock are published from the physics thread, and each message
    //costs per subscriber. Publishing them every step (0) is rarely needed: the controllers run far slower.
    //The rate is rounded to a whole number of steps. Message stamps stay exact, as the clock advances every step.
    double stateRate = nh_->declare_parameter("state_publish_rate", 0.0);
    if(stateRate > 0.0)
        stateDivider_ = (unsigned int)std::max(1L, std::lround(stepsPerSecond / stateRate));

    paramCallbackHandle_ = nh_->add_on_set_parameters_callback(std::bind(&ROS2SimulationManager::ParametersCallback, this, _1));

    it_ = std::make_shared<image_transport::ImageTransport>(nh_);
    interface_ = std::make_shared<ROS2Interface>(nh_, clock_);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_);

    //Actuator setpoints are handled on their own thread: the main thread renders, and a frame can take
    //tens of milliseconds on a slow GPU, which would otherwise delay every command by as much
    setpointCallbackGroup_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    setpointExecutor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    setpointExecutor_->add_callback_group(setpointCallbackGroup_, nh_->get_node_base_interface());
}

ROS2SimulationManager::~ROS2SimulationManager()
{
    StopSetpointThread();
    StopImageThread();
}

void ROS2SimulationManager::StopSetpointThread()
{
    setpointExecutor_->cancel();
    if(setpointThread_.joinable())
        setpointThread_.join();
}

void ROS2SimulationManager::StartImageThread()
{
    stopImageThread_ = false;
    imageThread_ = std::thread([this]()
    {
        std::unique_lock<std::mutex> lock(imageMutex_);
        while(true)
        {
            imageCondition_.wait(lock, [this]{ return stopImageThread_ || !pendingImages_.empty(); });
            if(stopImageThread_)
                return;

            std::deque<std::pair<std::string, std::function<void()>>> images;
            images.swap(pendingImages_);
            lock.unlock();
            imageCondition_.notify_all(); //There is room for new frames
            for(auto& image : images)
            {
                try
                {
                    image.second();
                }
                catch(const std::exception& e)
                {
                    RCLCPP_ERROR_STREAM(nh_->get_logger(), "Publishing image of '" << image.first << "' failed: " << e.what());
                }
            }
            lock.lock();
        }
    });
}

void ROS2SimulationManager::StopImageThread()
{
    {
        std::lock_guard<std::mutex> lock(imageMutex_);
        stopImageThread_ = true;
        pendingImages_.clear();
    }
    imageCondition_.notify_all();
    if(imageThread_.joinable())
        imageThread_.join();
}

void ROS2SimulationManager::PublishImageAsync(const std::string& camera, std::function<void()> publish)
{
    {
        std::unique_lock<std::mutex> lock(imageMutex_);
        if(lockStep_)
        {
            //Every frame is published: the renderer, and with it the simulation, waits when publishing cannot keep up
            imageCondition_.wait(lock, [this]{ return stopImageThread_ || pendingImages_.size() < 8; });
            if(stopImageThread_)
                return;
            pendingImages_.emplace_back(camera, std::move(publish));
        }
        else
        {
            auto pending = std::find_if(pendingImages_.begin(), pendingImages_.end(), 
                [&camera](const std::pair<std::string, std::function<void()>>& image) { return image.first == camera; });
            if(pending != pendingImages_.end())
                pending->second = std::move(publish); //Replaces a frame not yet published, which is stale by now
            else
                pendingImages_.emplace_back(camera, std::move(publish));
        }
    }
    imageCondition_.notify_all();
}

sensor_msgs::msg::Image::SharedPtr ROS2SimulationManager::CopyImage(const sensor_msgs::msg::Image& prototype, const void* data)
{
    //A new message per frame, as the previous one may still be being published
    auto img = std::make_shared<sensor_msgs::msg::Image>();
    img->header = prototype.header;
    img->height = prototype.height;
    img->width = prototype.width;
    img->encoding = prototype.encoding;
    img->is_bigendian = prototype.is_bigendian;
    img->step = prototype.step;
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    img->data.assign(bytes, bytes + (size_t)prototype.step * prototype.height);
    return img;
}

rclcpp::Time ROS2SimulationManager::CaptureStamp(Camera* cam)
{
    //On the simulation clock, images are stamped with the time their frame was captured, shared by all cameras
    //rendered in the same frame, rather than when they are read back from the GPU (a frame or more later).
    //The capture time is on the system time base, so it is shifted to the simulation time published on /clock.
    if(clockPub_ != nullptr)
    {
        Scalar t = std::max(cam->GetLastCaptureTime() - GetSimulationTimeOffset(), Scalar(0)); //Frames from before the start are stamped 0
        return rclcpp::Time((rcl_time_point_value_t)std::llround(t * Scalar(1e9)), RCL_ROS_TIME);
    }
    return clock_->now();
}

void ROS2SimulationManager::PublishSegmentationLabels(const std::string& camera, const std_msgs::msg::Header& header)
{
    //Only when they change, which is in the first frames and when objects are added. The revision is read
    //before the labels, so labels changing in between are published again with the next image.
    unsigned int revision = GetSegmentationLabelsRevision();
    auto published = segmentationLabelsRevisions_.find(camera);
    if(published != segmentationLabelsRevisions_.end() && published->second == revision)
        return;
    segmentationLabelsRevisions_[camera] = revision;

    vision_msgs::msg::LabelInfo msg;
    msg.header = header;
    std::vector<std::string> labels = GetSegmentationLabels();
    for(size_t i=1; i<labels.size(); ++i) //Pixel value 0 is the background
    {
        vision_msgs::msg::VisionClass c;
        c.class_id = (uint16_t)i;
        c.class_name = labels[i];
        msg.class_map.push_back(c);
    }
    vision_msgs::msg::VisionClass particles;
    particles.class_id = SegmentationCamera::OCEAN_PARTICLES_ID;
    particles.class_name = "ocean_particles";
    msg.class_map.push_back(particles);

    //Published right away, so that it reaches subscribers before the image, which is queued
    std::static_pointer_cast<rclcpp::Publisher<vision_msgs::msg::LabelInfo>>(pubs_.at(camera + "/labels"))->publish(msg);
}

rclcpp::SubscriptionOptions ROS2SimulationManager::GetSetpointSubscriptionOptions() const
{
    rclcpp::SubscriptionOptions options;
    options.callback_group = setpointCallbackGroup_;
    return options;
}

std::unordered_map<std::string, rclcpp::ServiceBase::SharedPtr>& ROS2SimulationManager::GetServices()
{
    return srvs_;
}

std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>& ROS2SimulationManager::GetPublishers()
{
    return pubs_;
}
    
std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr>& ROS2SimulationManager::GetSubscribers()
{
    return subs_;
}

std::unordered_map<std::string, image_transport::Publisher>& ROS2SimulationManager::GetImagePublishers()
{
    return imgPubs_;
}

std::shared_ptr<image_transport::ImageTransport> ROS2SimulationManager::GetImageTransportHandle()
{
    return it_;
}

std::unordered_map<std::string, std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr>>& ROS2SimulationManager::GetCameraMsgPrototypes()
{
    return cameraMsgPrototypes_;
}

std::unordered_map<std::string, std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr, sensor_msgs::msg::Image::SharedPtr>>& ROS2SimulationManager::GetDualImageCameraMsgPrototypes()
{
    return dualImageCameraMsgPrototypes_;
}

std::unordered_map<std::string, std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::Image::SharedPtr>>& ROS2SimulationManager::GetSonarMsgPrototypes()
{
    return sonarMsgPrototypes_;
}

void ROS2SimulationManager::AddROS2Robot(std::unique_ptr<ROS2Robot> robot)
{
    rosRobots_.push_back(std::move(robot));
}

bool ROS2SimulationManager::RespawnROS2Robot(const std::string& robotName, const Transform& origin)
{
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(rosRobots_[i]->robot_->GetName() == robotName)
        {
            rosRobots_[i]->respawnOrigin_ = origin;
            rosRobots_[i]->respawnRequested_ = true;
            return true;
        }
    }
    return false;
}

void ROS2SimulationManager::BuildScenario()
{
    // Run parser
    ROS2ScenarioParser parser(this, nh_);
    bool success = parser.Parse(scenarioPath_);

    // Save log
    std::string logPath = rclcpp::get_logging_directory().string() + "/stonefish_ros2_parser.log";
    bool success2 = parser.SaveLog(logPath);

    if(!success)
    {
        RCLCPP_ERROR(nh_->get_logger(), "Parsing of scenario file '%s' failed!", scenarioPath_.c_str());
        if(success2)
            RCLCPP_ERROR(nh_->get_logger(), "For more information check the parser log file '%s'.", logPath.c_str());
    }

    if(!success2)
        RCLCPP_ERROR(nh_->get_logger(), "Parser log file '%s' could not be saved!", logPath.c_str());

    // Standard services
    srvs_["enable_currents"] = nh_->create_service<std_srvs::srv::Trigger>("enable_currents", std::bind(&ROS2SimulationManager::EnableCurrentsService, this, _1, _2));
    srvs_["disable_currents"] = nh_->create_service<std_srvs::srv::Trigger>("disable_currents", std::bind(&ROS2SimulationManager::DisableCurrentsService, this, _1, _2));
    srvs_["respawn_robot"] = nh_->create_service<stonefish_ros2::srv::Respawn>("respawn_robot", std::bind(&ROS2SimulationManager::RespawnRobotService, this, _1, _2));
    srvs_["pause"] = nh_->create_service<std_srvs::srv::SetBool>("pause", std::bind(&ROS2SimulationManager::PauseService, this, _1, _2));
    srvs_["step"] = nh_->create_service<stonefish_ros2::srv::Step>("step", std::bind(&ROS2SimulationManager::StepService, this, _1, _2));

    // Cameras nobody subscribes to are not rendered
    cameraUseTimer_ = nh_->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ROS2SimulationManager::UpdateCameraUse, this));

    // Setpoint callbacks and queued images reference the scenario, so they only run while it exists
    if(!setpointThread_.joinable())
        setpointThread_ = std::thread([this]() { setpointExecutor_->spin(); });
    if(!imageThread_.joinable())
        StartImageThread();
}

void ROS2SimulationManager::DestroyScenario()
{
    cameraUseTimer_.reset();
    StopSetpointThread();
    StopImageThread();

    for(auto it = imgPubs_.begin(); it != imgPubs_.end(); ++it)
        it->second.shutdown();

    pubs_.clear();
    imgPubs_.clear();
    subs_.clear();
    srvs_.clear();
    cameraMsgPrototypes_.clear();
    sonarMsgPrototypes_.clear();
    segmentationLabelsRevisions_.clear();
    rosRobots_.clear();

    SimulationManager::DestroyScenario();
}
	
void ROS2SimulationManager::SimulationStepCompleted(Scalar timeStep)
{
    (void)timeStep; // Suppress warning

    //Setpoints are applied every step, robot state is only published every stateDivider_ steps
    //and after the last of the steps requested from a paused simulation, so that its end can be observed
    bool publishState = stepCounter_++ % stateDivider_ == 0 || (IsPaused() && GetRequestedSteps() == 0);

    ////////////////////////////////////////CLOCK////////////////////////////////////////////////
    if(clockPub_ != nullptr)
    {
        rcl_time_point_value_t t = (rcl_time_point_value_t)std::llround(GetSimulationTime() * Scalar(1e9));
        if(rcl_set_ros_time_override(clock_->get_clock_handle(), t) != RCL_RET_OK)
            RCLCPP_ERROR_STREAM_ONCE(nh_->get_logger(), "Failed to update the simulation clock!");
        if(publishState)
        {
            rosgraph_msgs::msg::Clock msg;
            msg.clock = rclcpp::Time(t, RCL_ROS_TIME);
            clockPub_->publish(msg);
        }
    }

    ////////////////////////////////////////SENSORS//////////////////////////////////////////////
    unsigned int id = 0;
    Sensor* sensor;
    while((sensor = GetSensor(id++)) != nullptr)
    {
        if(sensor->GetType() == SensorType::VISION && !sensor->IsEnabled())
        {
            PublishIdleCameraInfo(sensor, timeStep);
            continue;
        }

        if(!sensor->IsNewDataAvailable())
            continue;

        if(sensor->GetType() != SensorType::VISION)
        {
            if(pubs_.find(sensor->GetName()) == pubs_.end())
                continue;

            switch(static_cast<ScalarSensor*>(sensor)->GetScalarSensorType())
            {
                case ScalarSensorType::ACC:
                    interface_->PublishAccelerometer(pubs_.at(sensor->GetName()), (Accelerometer*)sensor);
                    break;
                case ScalarSensorType::GYRO:
                    interface_->PublishGyroscope(pubs_.at(sensor->GetName()), (Gyroscope*)sensor);
                    break;
                case ScalarSensorType::IMU:
                    interface_->PublishIMU(pubs_.at(sensor->GetName()), (IMU*)sensor);
                    break;
                case ScalarSensorType::ODOM:
                    interface_->PublishOdometry(pubs_.at(sensor->GetName()), (Odometry*)sensor);
                    break;
                case ScalarSensorType::DVL:
                {
                    interface_->PublishDVL(pubs_.at(sensor->GetName()), (DVL*)sensor);
                    if(pubs_.find(sensor->GetName() + "/altitude") != pubs_.end())
                        interface_->PublishDVLAltitude(pubs_.at(sensor->GetName() + "/altitude"), (DVL*)sensor);
                }
                    break;
                case ScalarSensorType::INS:
                {
                    interface_->PublishINS(pubs_.at(sensor->GetName()), (INS*)sensor);
                    if(pubs_.find(sensor->GetName() + "/odometry") != pubs_.end())
                        interface_->PublishINSOdometry(pubs_.at(sensor->GetName() + "/odometry"), (INS*)sensor);
                }
                    break;
                case ScalarSensorType::GPS:
                    interface_->PublishGPS(pubs_.at(sensor->GetName()), (GPS*)sensor);
                    break;
                case ScalarSensorType::PRESSURE:
                    interface_->PublishPressure(pubs_.at(sensor->GetName()), (Pressure*)sensor);
                    break;
                case ScalarSensorType::FT:
                    interface_->PublishForceTorque(pubs_.at(sensor->GetName()), (ForceTorque*)sensor);
                    break;
                case ScalarSensorType::ENCODER:
                    interface_->PublishEncoder(pubs_.at(sensor->GetName()), (RotaryEncoder*)sensor);
                    break;
                case ScalarSensorType::MULTIBEAM:
                {
                    interface_->PublishMultibeam(pubs_.at(sensor->GetName()), (Multibeam*)sensor);
                    if(pubs_.find(sensor->GetName() + "/pcl") != pubs_.end())
                        interface_->PublishMultibeamPCL(pubs_.at(sensor->GetName() + "/pcl"), (Multibeam*)sensor);
                }
                    break;
                case ScalarSensorType::PROFILER:
                    interface_->PublishProfiler(pubs_.at(sensor->GetName()), (Profiler*)sensor);
                    break;
                case ScalarSensorType::PLUGIN:
                {
                    ROS2SensorPluginInterface* iface = dynamic_cast<ROS2SensorPluginInterface*>(sensor);
                    if (iface != nullptr)
                        iface->Publish(nh_, pubs_);
                }
                    break;
                default:
                    break;
            }
        }

        sensor->MarkDataOld();
    }

    ///////////////////////////////////////COMMS///////////////////////////////////////////////////
    id = 0;
    Comm* comm;
    while((comm = GetComm(id++)) != nullptr)
    {
        if(pubs_.find(comm->GetName()) == pubs_.end())
            continue;

        switch(comm->GetType())
        {
            case CommType::ACOUSTIC:
            {
                std_msgs::msg::String msg;
                std::shared_ptr<CommDataFrame> message;
                while ((message = comm->ReadMessage()) != nullptr)
                {
                    msg.data = std::string(message->data.begin(), message->data.end());
                    std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::String>>(
                        pubs_.at(comm->GetName())
                    )->publish(msg);
                }
            }
                break;
            case CommType::USBL:
            {
                if(comm->IsNewDataAvailable())
                {
                    interface_->PublishUSBL(pubs_.at(comm->GetName()), pubs_.at(comm->GetName() + "/beacon_info"), (USBL*)comm);
                    comm->MarkDataOld();
                }

                std_msgs::msg::String msg;
                std::shared_ptr<CommDataFrame> message;
                while ((message = comm->ReadMessage()) != nullptr)
                {
                    msg.data = std::string(message->data.begin(), message->data.end());
                    std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::String>>(
                        pubs_.at(comm->GetName() + "/received_data")
                    )->publish(msg);
                }
            }
                break;
            case CommType::OPTICAL:
            {
                std_msgs::msg::Float64 msg;
                msg.data = ((OpticalModem*)comm)->GetReceptionQuality();
                std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float64>>(
                    pubs_.at(comm->GetName())
                )->publish(msg);

                std_msgs::msg::String msg2;
                std::shared_ptr<CommDataFrame> message;
                while ((message = comm->ReadMessage()) != nullptr)
                {
                    msg2.data = std::string(message->data.begin(), message->data.end());
                    std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::String>>(
                        pubs_.at(comm->GetName() + "/received_data")
                    )->publish(msg2);
                }
            }
                break;
            case CommType::PLUGIN:
            {
                ROS2CommPluginInterface* iface = dynamic_cast<ROS2CommPluginInterface*>(comm);
                if (iface != nullptr)
                    iface->Publish(nh_, pubs_);
            }
                break;
            default:
                break;
        }
    }

    //////////////////////////////////////TRAJECTORIES/////////////////////////////////////////////
    id = 0;
    Entity* ent;
    while(publishState && (ent = GetEntity(id++)) != nullptr)
    {
        if(ent->GetType() == EntityType::ANIMATED)
        {
            if(pubs_.find(ent->GetName() + "/odometry") == pubs_.end())
                continue;

            interface_->PublishTrajectoryState(pubs_.at(ent->GetName() + "/odometry"), pubs_.at(ent->GetName() + "/iteration"), (AnimatedEntity*)ent);
        }
    }

    //////////////////////////////////////CONTACTS/////////////////////////////////////////////////
    id = 0;
    Contact* cnt;
    while((cnt = GetContact(id++)) != nullptr)
    {
        if(!cnt->IsNewDataAvailable())
            continue;

        if(pubs_.find(cnt->GetName()) != pubs_.end())
        {
            interface_->PublishContact(pubs_[cnt->GetName()], cnt);
            cnt->MarkDataOld();
        }
    }

    //////////////////////////////////////WORLD TRANSFORMS/////////////////////////////////////////
    for(size_t i=0; publishState && i<rosRobots_.size(); ++i)
    {
        if(rosRobots_[i]->publishBaseLinkTransform_)
            interface_->PublishTF(tf_, rosRobots_[i]->robot_->GetTransform(), clock_->now(), "world_ned", rosRobots_[i]->robot_->GetName() + "/base_link");
    }
    
    //////////////////////////////////////SERVOS(JOINTS)/////////////////////////////////////////
    for(size_t i=0; publishState && i<rosRobots_.size(); ++i)
    {
        if(pubs_.find(rosRobots_[i]->robot_->GetName() + "/motors") != pubs_.end())
        {
            unsigned int aID = 0;
            Actuator* actuator;
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = clock_->now();
            msg.header.frame_id = rosRobots_[i]->robot_->GetName();
            
            while((actuator = rosRobots_[i]->robot_->GetActuator(aID++)) != nullptr)
            {
                if(IsJointActuator(actuator, JointActuatorType::MOTOR))
                {
                    Motor* mtr = static_cast<Motor*>(actuator);
                    msg.name.push_back(mtr->GetJointName());
                    msg.position.push_back(mtr->GetAngle());
                    msg.velocity.push_back(mtr->GetAngularVelocity());
                    msg.effort.push_back(mtr->GetTorque());
                }
                else if(IsJointActuator(actuator, JointActuatorType::DCMOTOR))
                {
                    DCMotor* mtr = static_cast<DCMotor*>(actuator);
                    msg.name.push_back(mtr->GetJointName());
                    msg.position.push_back(mtr->GetAngle());
                    msg.velocity.push_back(mtr->GetAngularVelocity());
                    msg.effort.push_back(mtr->GetTorque());
                }
            }
            if(msg.name.size() > 0)
                std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(
                    pubs_.at(rosRobots_[i]->robot_->GetName() + "/motors")
                )->publish(msg);
        }

        if(pubs_.find(rosRobots_[i]->robot_->GetName() + "/servos") != pubs_.end())
        {
            unsigned int aID = 0;
            Actuator* actuator;
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = clock_->now();
            msg.header.frame_id = rosRobots_[i]->robot_->GetName();
            
            while((actuator = rosRobots_[i]->robot_->GetActuator(aID++)) != nullptr)
            {
                if(IsJointActuator(actuator, JointActuatorType::SERVO))
                {
                    Servo* srv = static_cast<Servo*>(actuator);
                    msg.name.push_back(srv->GetJointName());
                    msg.position.push_back(srv->GetPosition());
                    msg.velocity.push_back(srv->GetVelocity());
                    msg.effort.push_back(srv->GetEffort());
                }
            }
            if(msg.name.size() > 0)
                std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(
                    pubs_.at(rosRobots_[i]->robot_->GetName() + "/servos")
                )->publish(msg);
        }

        if(pubs_.find(rosRobots_[i]->robot_->GetName() + "/rudders") != pubs_.end())
        {
            unsigned int aID = 0;
            Actuator* actuator;
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = clock_->now();
            msg.header.frame_id = rosRobots_[i]->robot_->GetName();
            
            while((actuator = rosRobots_[i]->robot_->GetActuator(aID++)) != nullptr)
            {
                if(IsLinkActuator(actuator, LinkActuatorType::RUDDER))
                {
                    Rudder* rudder = static_cast<Rudder*>(actuator);
                    msg.name.push_back(rudder->GetName());
                    msg.position.push_back(rudder->GetSetpoint());
                    msg.velocity.push_back(0.0);
                    msg.effort.push_back(0.0);
                }
            }
            if(msg.name.size() > 0)
                std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(
                    pubs_.at(rosRobots_[i]->robot_->GetName() + "/rudders")
                )->publish(msg);
        }

        if(rosRobots_[i]->thrusterSetpoints_.size() != 0
           && pubs_.find(rosRobots_[i]->robot_->GetName() + "/thrusters") != pubs_.end())
        {
            unsigned int aID = 0;
            unsigned int thID = 0;
            Actuator* actuator;
            stonefish_ros2::msg::ThrusterState msg;
            msg.header.stamp = clock_->now();
            msg.header.frame_id = rosRobots_[i]->robot_->GetName();
            msg.setpoint.resize(rosRobots_[i]->thrusterSetpoints_.size());
            msg.rpm.resize(rosRobots_[i]->thrusterSetpoints_.size());
            msg.thrust.resize(rosRobots_[i]->thrusterSetpoints_.size());
            msg.torque.resize(rosRobots_[i]->thrusterSetpoints_.size());

            while((actuator = rosRobots_[i]->robot_->GetActuator(aID++)) != nullptr)
            {
                if(IsLinkActuator(actuator, LinkActuatorType::THRUSTER))
                {
                    Thruster* th = static_cast<Thruster*>(actuator);
                    msg.setpoint[thID] = th->GetSetpoint();
                    msg.rpm[thID] = th->GetOmega()/(Scalar(2)*M_PI)*Scalar(60);
                    msg.thrust[thID] = th->GetThrust();
                    msg.torque[thID] = th->GetTorque();
                    ++thID;
                }
                else if(IsLinkActuator(actuator, LinkActuatorType::SIMPLE_THRUSTER))
                {
                    SimpleThruster* th = static_cast<SimpleThruster*>(actuator);
                    msg.setpoint[thID] = th->GetThrustSetpoint();
                    msg.rpm[thID] = Scalar(0);
                    msg.thrust[thID] = th->GetThrust();
                    msg.torque[thID] = th->GetTorque();
                    ++thID;
                }

                if(thID == rosRobots_[i]->thrusterSetpoints_.size())
                    break;
            }
            std::static_pointer_cast<rclcpp::Publisher<stonefish_ros2::msg::ThrusterState>>(
                    pubs_.at(rosRobots_[i]->robot_->GetName() + "/thrusters")
                )->publish(msg);
        }
    }

    //////////////////////////////////////////////ACTUATORS//////////////////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        unsigned int aID = 0;
        Actuator* actuator;
        unsigned int thID = 0;
        unsigned int propID = 0;
        unsigned int rudderID = 0;

        while((actuator = rosRobots_[i]->robot_->GetActuator(aID++)) != nullptr)
        {
            if (actuator->GetType() == ActuatorType::LINK)
            {
                switch(static_cast<LinkActuator*>(actuator)->GetLinkActuatorType())
                {
                    case LinkActuatorType::PUSH:
                    {
                        Push* push = static_cast<Push*>(actuator); //Commanded through its own topic

                        auto it = pubs_.find(actuator->GetName());
                        if(publishState && it != pubs_.end())
                        {
                            geometry_msgs::msg::WrenchStamped msg;
                            msg.header.stamp = clock_->now();
                            msg.header.frame_id = push->GetName();
                            msg.wrench.force.x = push->GetForce();
                            std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(it->second)->publish(msg);
                        }
                    }
                        break;
                    case LinkActuatorType::SIMPLE_THRUSTER:
                    {
                        SimpleThruster* th = static_cast<SimpleThruster*>(actuator);
                        if(rosRobots_[i]->thrusterSetpointsChanged_)
                        {
                            th->SetSetpoint(rosRobots_[i]->thrusterSetpoints_[thID++], Scalar(0));
                        }
                            
                        auto it = pubs_.find(actuator->GetName()+"/wrench");
                        if(publishState && it != pubs_.end())
                        {
                            geometry_msgs::msg::WrenchStamped msg;
                            msg.header.stamp = clock_->now();
                            msg.header.frame_id = th->GetName();
                            msg.wrench.force.x = th->GetThrust();
                            msg.wrench.torque.x = th->GetTorque();
                            std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(it->second)->publish(msg);
                        }            

                        it = pubs_.find(actuator->GetName()+"/joint_state");
                        if(publishState && it != pubs_.end())
                        {
                            //Publish propeller rotation for visualization
                            sensor_msgs::msg::JointState msg;
                            msg.header.stamp = clock_->now();
                            msg.header.frame_id = th->GetName();
                            msg.name.push_back(th->GetName()+"/propeller");
                            msg.position.push_back(th->GetAngle());  
                            std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(it->second)->publish(msg);
                        }
                    }
                        break;
                    case LinkActuatorType::THRUSTER:
                    {
                        Thruster* th = static_cast<Thruster*>(actuator);
                        if(rosRobots_[i]->thrusterSetpointsChanged_)
                        {
                            th->SetSetpoint(rosRobots_[i]->thrusterSetpoints_[thID++]);
                        }
                        if(rosRobots_[i]->supplyVoltageChanged_)
                            th->SetSupplyVoltage(rosRobots_[i]->supplyVoltage_);
                            
                        auto it = pubs_.find(actuator->GetName()+"/wrench");
                        if(publishState && it != pubs_.end())
                        {
                            geometry_msgs::msg::WrenchStamped msg;
                            msg.header.stamp = clock_->now();
                            msg.header.frame_id = th->GetName();
                            msg.wrench.force.x = th->GetThrust();
                            msg.wrench.torque.x = th->GetTorque();
                            std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(it->second)->publish(msg);
                        }            

                        it = pubs_.find(actuator->GetName()+"/joint_state");
                        if(publishState && it != pubs_.end())
                        {
                            //Publish propeller rotation for visualization
                            sensor_msgs::msg::JointState msg;
                            msg.header.stamp = clock_->now();
                            msg.header.frame_id = th->GetName();
                            msg.name.push_back(th->GetName()+"/propeller");
                            msg.position.push_back(th->GetAngle()/Scalar(100)); // Scaled for visualisation 
                            msg.velocity.push_back(th->GetOmega());
                            msg.effort.push_back(th->GetThrust());
                            std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(it->second)->publish(msg);
                        }
                    }
                        break;
                    case LinkActuatorType::PROPELLER:
                    {
                        Propeller* prop = static_cast<Propeller*>(actuator);
                        if(rosRobots_[i]->propellerSetpointsChanged_)
                        {
                            prop->SetSetpoint(rosRobots_[i]->propellerSetpoints_[propID++]);
                        }

                        auto it = pubs_.find(actuator->GetName());
                        if(publishState && it != pubs_.end())
                        {
                            geometry_msgs::msg::WrenchStamped msg;
                            msg.header.stamp = clock_->now();
                            msg.header.frame_id = prop->GetName();
                            msg.wrench.force.x = prop->GetThrust();
                            msg.wrench.torque.x = prop->GetTorque();
                            std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(it->second)->publish(msg);
                        }
                    }
                        break;
                    case LinkActuatorType::RUDDER:
                    {
                        if(rosRobots_[i]->rudderSetpointsChanged_)
                        {
                            (static_cast<Rudder*>(actuator))->SetSetpoint(rosRobots_[i]->rudderSetpoints_[rudderID++]);
                        }
                    }
                        break;
                    case LinkActuatorType::VBS:
                    {
                        auto it = pubs_.find(actuator->GetName());
                        if(publishState && it != pubs_.end())
                        {
                            std_msgs::msg::Float64 msg;
                            msg.data = static_cast<VariableBuoyancy*>(actuator)->GetLiquidVolume();
                            std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float64>>(it->second)->publish(msg);
                        }
                    }
                        break;
                    case LinkActuatorType::SUCTION_CUP:
                    {
                        auto it = pubs_.find(actuator->GetName());
                        if(publishState && it != pubs_.end())
                        {
                            std_msgs::msg::Bool msg;
                            msg.data = static_cast<SuctionCup*>(actuator)->GetPump();
                            std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::Bool>>(it->second)->publish(msg);
                        }
                    }
                        break;
                    case LinkActuatorType::PLUGIN:
                    {
                        ROS2ActuatorPluginInterface* iface = dynamic_cast<ROS2ActuatorPluginInterface*>(actuator);
                        if (publishState && iface != nullptr)
                            iface->Publish(nh_, pubs_);
                    }
                        break;
                    default:
                        break;
                }
            }
            else if (actuator->GetType() == ActuatorType::JOINT)
            {
                switch (static_cast<JointActuator*>(actuator)->GetJointActuatorType())
                {
                    case JointActuatorType::MOTOR:
                    {
                        if(rosRobots_[i]->servoSetpoints_.size() == 0)
                            continue;

                        Motor* mtr = static_cast<Motor*>(actuator);
                        auto it = rosRobots_[i]->servoSetpoints_.find(mtr->GetJointName());
                        if(it != rosRobots_[i]->servoSetpoints_.end())
                        {
                            if(it->second.first == ServoControlMode::TORQUE)
                            {
                                mtr->SetTorque(it->second.second);
                            }
                        }
                    }
                        break;

                    case JointActuatorType::DCMOTOR:
                    {
                        if(rosRobots_[i]->servoSetpoints_.size() == 0)
                            continue;

                        DCMotor* mtr = static_cast<DCMotor*>(actuator);
                        auto it = rosRobots_[i]->servoSetpoints_.find(mtr->GetJointName());
                        if(it != rosRobots_[i]->servoSetpoints_.end())
                        {
                            if(it->second.first == ServoControlMode::TORQUE) //DC motors are driven by voltage
                            {
                                mtr->SetVoltage(it->second.second);
                            }
                        }
                    }
                        break;

                    case JointActuatorType::SERVO:
                    {
                        if(rosRobots_[i]->servoSetpoints_.size() == 0)
                            continue;

                        Servo* srv = static_cast<Servo*>(actuator);
                        auto it = rosRobots_[i]->servoSetpoints_.find(srv->GetJointName());
                        if(it != rosRobots_[i]->servoSetpoints_.end())
                        {
                            if(it->second.first == ServoControlMode::VELOCITY)
                            {
                                srv->SetControlMode(ServoControlMode::VELOCITY);
                                srv->SetDesiredVelocity(it->second.second);
                            }
                            else if(it->second.first == ServoControlMode::POSITION)
                            {
                                srv->SetControlMode(ServoControlMode::POSITION);
                                srv->SetDesiredPosition(it->second.second);
                            }
                        }
                    }
                        break;
                    case JointActuatorType::PLUGIN:
                    {
                        ROS2ActuatorPluginInterface* iface = dynamic_cast<ROS2ActuatorPluginInterface*>(actuator);
                        if (publishState && iface != nullptr)
                            iface->Publish(nh_, pubs_);
                    }
                        break;
                    default:
                        break;
                }
            }
        }

        //Reset change flags
        rosRobots_[i]->thrusterSetpointsChanged_ = false;
        rosRobots_[i]->supplyVoltageChanged_ = false;
        rosRobots_[i]->propellerSetpointsChanged_ = false;
        rosRobots_[i]->rudderSetpointsChanged_ = false;
    }

    /////////////////////////////////// RESPAWN REQUESTS ///////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(rosRobots_[i]->respawnRequested_)
        {
            rosRobots_[i]->robot_->Respawn(this, rosRobots_[i]->respawnOrigin_);
            rosRobots_[i]->respawnRequested_ = false;
        }
    }    

    /////////////////////////////////// DEBUG //////////////////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(publishState && pubs_.find(rosRobots_[i]->robot_->GetName() + "/debug/physics") != pubs_.end())
        {
            Robot* r = rosRobots_[i]->robot_;
            size_t lID = 0;
            SolidEntity* link;
            stonefish_ros2::msg::DebugPhysics msg;
            msg.header.stamp = clock_->now();
            
            auto debugPub = std::static_pointer_cast<rclcpp::Publisher<stonefish_ros2::msg::DebugPhysics>>(
                pubs_.at(r->GetName() + "/debug/physics")
            );

            while((link = r->GetLink(lID++)) != nullptr)
            {
                msg.header.frame_id = link->GetName();
                
                msg.mass = link->GetMass();
                msg.volume = link->GetVolume();
                msg.surface = link->GetSurface();
                Vector3 inertia = link->GetInertia();
                msg.inertia.x = inertia.getX();
                msg.inertia.y = inertia.getY();
                msg.inertia.z = inertia.getZ();

                Vector3 cog = -link->GetCG2OTransform().getOrigin();
                msg.cog.x = cog.getX();
                msg.cog.y = cog.getY();
                msg.cog.z = cog.getZ();
                
                Vector3 cob = link->GetCG2OTransform() * link->GetCb();
                msg.cob.x = cob.getX();
                msg.cob.y = cob.getY();
                msg.cob.z = cob.getZ();
                
                Matrix3 toOrigin = link->GetOTransform().getBasis().inverse();

                Vector3 vel = toOrigin * link->GetLinearVelocity();
                Vector3 avel = toOrigin * link->GetAngularVelocity();
                msg.velocity.linear.x = vel.getX();
                msg.velocity.linear.y = vel.getY();
                msg.velocity.linear.z = vel.getZ();
                msg.velocity.angular.x = avel.getX();
                msg.velocity.angular.y = avel.getY();
                msg.velocity.angular.z = avel.getZ();

                Vector3 Fb, Tb, Fd, Td, Ff, Tf;
                link->GetHydrodynamicForces(Fb, Tb, Fd, Td, Ff, Tf);            
                Vector3 Cd, Cf;
                link->GetHydrodynamicCoefficients(Cd, Cf);
                msg.damping_coeff.x = Cd.getX();
                msg.damping_coeff.y = Cd.getY();
                msg.damping_coeff.z = Cd.getZ();
                msg.skin_friction_coeff.x = Cf.getX();
                msg.skin_friction_coeff.y = Cf.getY();
                msg.skin_friction_coeff.z = Cf.getZ();

                // Fb = toOrigin * Fb;
                // Tb = toOrigin * Tb;
                msg.buoyancy.force.x = Fb.getX();
                msg.buoyancy.force.y = Fb.getY();
                msg.buoyancy.force.z = Fb.getZ();
                msg.buoyancy.torque.x = Tb.getX();
                msg.buoyancy.torque.y = Tb.getY();
                msg.buoyancy.torque.z = Tb.getZ();

                Fd = toOrigin * Fd;
                Td = toOrigin * Td;
                msg.damping.force.x = Fd.getX();
                msg.damping.force.y = Fd.getY();
                msg.damping.force.z = Fd.getZ();
                msg.damping.torque.x = Td.getX();
                msg.damping.torque.y = Td.getY();
                msg.damping.torque.z = Td.getZ();

                Ff = toOrigin * Ff;
                Tf = toOrigin * Tf;
                msg.skin_friction.force.x = Ff.getX();
                msg.skin_friction.force.y = Ff.getY();
                msg.skin_friction.force.z = Ff.getZ();
                msg.skin_friction.torque.x = Tf.getX();
                msg.skin_friction.torque.y = Tf.getY();
                msg.skin_friction.torque.z = Tf.getZ();

                msg.wetted_surface = link->GetWettedSurface();
                msg.submerged_volume = link->GetSubmergedVolume();

                debugPub->publish(msg);
            }
        }
    }
}

void ROS2SimulationManager::UpdateCameraUse()
{
    //A camera whose images nobody subscribes to, in any transport, is turned off, so that it is not rendered, and only its
    //camera_info is published. A segmentation camera is rendered together with the camera it is paired with, so while one
    //is used all cameras are on.
    auto used = [this](const std::string& name)
    {
        size_t subscribers = 0;
        for(const std::string& key : {name, name + "/display"})
        {
            auto it = imgPubs_.find(key);
            if(it != imgPubs_.end())
                subscribers += it->second.getNumSubscribers();
        }
        return subscribers > 0;
    };

    std::vector<VisionSensor*> cameras;
    bool segmentationUsed = false;
    unsigned int id = 0;
    Sensor* sensor;
    while((sensor = GetSensor(id++)) != nullptr)
    {
        if(sensor->GetType() != SensorType::VISION)
            continue;
        VisionSensor* cam = static_cast<VisionSensor*>(sensor);
        VisionSensorType type = cam->GetVisionSensorType();
        if(type != VisionSensorType::COLOR_CAMERA && type != VisionSensorType::DEPTH_CAMERA 
           && type != VisionSensorType::SEGMENTATION_CAMERA)
            continue;
        cameras.push_back(cam);
        segmentationUsed |= type == VisionSensorType::SEGMENTATION_CAMERA && used(cam->GetName());
    }

    std::lock_guard<std::mutex> lock(cameraUseMutex_);
    for(VisionSensor* cam : cameras)
    {
        bool enabled = disabledSensors_.count(cam->GetName()) == 0 && (used(cam->GetName()) 
            || (segmentationUsed && cam->GetVisionSensorType() != VisionSensorType::SEGMENTATION_CAMERA));
        if(enabled != cam->IsEnabled())
            cam->SetEnabled(enabled);
    }
}

void ROS2SimulationManager::PublishIdleCameraInfo(Sensor* sens, Scalar timeStep)
{
    //A camera turned off for lack of subscribers keeps publishing its camera_info at its rate
    const std::string& name = sens->GetName();
    auto pub = pubs_.find(name + "/info");
    if(pub == pubs_.end() || sens->GetUpdateFrequency() <= Scalar(0))
        return;
    {
        std::lock_guard<std::mutex> lock(cameraUseMutex_);
        if(disabledSensors_.count(name) > 0)
            return;
    }
    Scalar& elapsed = idleCameraInfoTime_[name];
    elapsed += timeStep;
    Scalar period = Scalar(1)/sens->GetUpdateFrequency();
    if(elapsed < period)
        return;
    elapsed -= period;

    sensor_msgs::msg::CameraInfo::SharedPtr prototype;
    if(cameraMsgPrototypes_.count(name) > 0)
        prototype = cameraMsgPrototypes_.at(name).second;
    else if(dualImageCameraMsgPrototypes_.count(name) > 0)
        prototype = std::get<1>(dualImageCameraMsgPrototypes_.at(name));
    if(prototype == nullptr)
        return;
    sensor_msgs::msg::CameraInfo info = *prototype;
    info.header.stamp = clock_->now();
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(pub->second)->publish(info);
}

void ROS2SimulationManager::ColorCameraImageReady(ColorCamera* cam)
{
    //Copy the image out of the GPU buffer, which is only mapped during this call
    const std::string name = cam->GetName();
    sensor_msgs::msg::Image::SharedPtr img = CopyImage(*cameraMsgPrototypes_[name].first, cam->GetImageDataPointer());
    img->header.stamp = CaptureStamp(cam);

    auto info = std::make_shared<sensor_msgs::msg::CameraInfo>(*cameraMsgPrototypes_[name].second);
    info->header.stamp = img->header.stamp;

    //Publishing (and compressing, for image_transport subscribers) is left to the image thread
    PublishImageAsync(name, [this, name, img, info]()
    {
        imgPubs_.at(name).publish(img);
        std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(pubs_.at(name + "/info"))->publish(*info);
    });
}

void ROS2SimulationManager::DepthCameraImageReady(DepthCamera* cam)
{
    //As for colour cameras
    const std::string name = cam->GetName();
    sensor_msgs::msg::Image::SharedPtr img = CopyImage(*cameraMsgPrototypes_[name].first, cam->GetImageDataPointer());
    img->header.stamp = CaptureStamp(cam);

    auto info = std::make_shared<sensor_msgs::msg::CameraInfo>(*cameraMsgPrototypes_[name].second);
    info->header.stamp = img->header.stamp;

    PublishImageAsync(name, [this, name, img, info]()
    {
        imgPubs_.at(name).publish(img);
        std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(pubs_.at(name + "/info"))->publish(*info);
    });
}

void ROS2SimulationManager::ThermalCameraImageReady(ThermalCamera* cam)
{
    //Fill in the image message
    sensor_msgs::msg::Image::SharedPtr img = std::get<0>(dualImageCameraMsgPrototypes_[cam->GetName()]);
    img->header.stamp = CaptureStamp(cam);
    memcpy(img->data.data(), (float*)cam->GetImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::msg::CameraInfo::SharedPtr info = std::get<1>(dualImageCameraMsgPrototypes_[cam->GetName()]);
    info->header.stamp = img->header.stamp;

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr img2 = std::get<2>(dualImageCameraMsgPrototypes_[cam->GetName()]);
    img2->header.stamp = img->header.stamp;
    memcpy(img2->data.data(), (uint8_t*)cam->GetDisplayDataPointer(), img2->step * img2->height);

    //Publish messages
    imgPubs_.at(cam->GetName()).publish(img);
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(pubs_.at(cam->GetName() + "/info"))->publish(*info);
    imgPubs_.at(cam->GetName()+"/display").publish(img2);
}

void ROS2SimulationManager::OpticalFlowCameraImageReady(OpticalFlowCamera* cam)
{
    //Fill in the image message
    sensor_msgs::msg::Image::SharedPtr img = std::get<0>(dualImageCameraMsgPrototypes_[cam->GetName()]);
    img->header.stamp = CaptureStamp(cam);
    memcpy(img->data.data(), (float*)cam->GetImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::msg::CameraInfo::SharedPtr info = std::get<1>(dualImageCameraMsgPrototypes_[cam->GetName()]);
    info->header.stamp = img->header.stamp;

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr img2 = std::get<2>(dualImageCameraMsgPrototypes_[cam->GetName()]);
    img2->header.stamp = img->header.stamp;
    memcpy(img2->data.data(), (uint8_t*)cam->GetDisplayDataPointer(), img2->step * img2->height);

    //Publish messages
    imgPubs_.at(cam->GetName()).publish(img);
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(pubs_.at(cam->GetName() + "/info"))->publish(*info);
    imgPubs_.at(cam->GetName()+"/display").publish(img2);
}

void ROS2SimulationManager::SegmentationCameraImageReady(SegmentationCamera* cam)
{
    //As for colour cameras, with the labels and their display image
    const std::string name = cam->GetName();
    auto& prototypes = dualImageCameraMsgPrototypes_[name];
    sensor_msgs::msg::Image::SharedPtr img = CopyImage(*std::get<0>(prototypes), cam->GetImageDataPointer());
    img->header.stamp = CaptureStamp(cam);
    PublishSegmentationLabels(name, img->header);

    auto info = std::make_shared<sensor_msgs::msg::CameraInfo>(*std::get<1>(prototypes));
    info->header.stamp = img->header.stamp;

    sensor_msgs::msg::Image::SharedPtr img2 = CopyImage(*std::get<2>(prototypes), cam->GetDisplayDataPointer());
    img2->header.stamp = img->header.stamp;

    PublishImageAsync(name, [this, name, img, info, img2]()
    {
        imgPubs_.at(name).publish(img);
        std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(pubs_.at(name + "/info"))->publish(*info);
        imgPubs_.at(name + "/display").publish(img2);
    });
}

void ROS2SimulationManager::EventBasedCameraOutputReady(EventBasedCamera* cam)
{
    interface_->PublishEventBasedCamera(pubs_.at(cam->GetName()), cam);
}

void ROS2SimulationManager::LidarScanReady(Lidar* lidar)
{
    interface_->PublishLidar(pubs_.at(lidar->GetName()), lidar);
}

void ROS2SimulationManager::FLSScanReady(FLS* fls)
{
    //Fill in the data message
    sensor_msgs::msg::Image::SharedPtr img = sonarMsgPrototypes_[fls->GetName()].first;
    img->header.stamp = clock_->now();
    memcpy(img->data.data(), fls->GetImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr disp = sonarMsgPrototypes_[fls->GetName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), fls->GetDisplayDataPointer(), disp->step * disp->height);

    //Publish messages
    imgPubs_.at(fls->GetName()).publish(img);
    imgPubs_.at(fls->GetName() + "/display").publish(disp);

}

void ROS2SimulationManager::SSSScanReady(SSS* sss)
{
    //Fill in the data message
    sensor_msgs::msg::Image::SharedPtr img = sonarMsgPrototypes_[sss->GetName()].first;
    img->header.stamp = clock_->now();
    memcpy(img->data.data(), sss->GetImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr disp = sonarMsgPrototypes_[sss->GetName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), sss->GetDisplayDataPointer(), disp->step * disp->height);

    //Publish messages
    imgPubs_.at(sss->GetName()).publish(img);
    imgPubs_.at(sss->GetName() + "/display").publish(disp);
}

void ROS2SimulationManager::MSISScanReady(MSIS* msis)
{
    //Fill in the data message
    sensor_msgs::msg::Image::SharedPtr img = sonarMsgPrototypes_[msis->GetName()].first;
    img->header.stamp = clock_->now();
    memcpy(img->data.data(), msis->GetImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr disp = sonarMsgPrototypes_[msis->GetName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), msis->GetDisplayDataPointer(), disp->step * disp->height);

    //Fill in the laser scan message
    Scalar currentAngle = (msis->GetCurrentRotationStep() * msis->GetRotationStepAngle()) * M_PI / 180.0;
    unsigned int currentBeamIndex = msis->GetCurrentBeamIndex();

    sensor_msgs::msg::LaserScan laserscan;
    laserscan.header.stamp = img->header.stamp;
    laserscan.header.frame_id = msis->GetName();
    laserscan.angle_min = currentAngle;
    laserscan.angle_max = currentAngle;
    laserscan.angle_increment = 0.0;
    laserscan.time_increment = 0.0;
    laserscan.range_min = msis->GetRangeMin();
    laserscan.range_max = msis->GetRangeMax();
    laserscan.ranges.resize(img->height);
    laserscan.intensities.resize(img->height);

    for(unsigned int i=0; i<img->height; ++i)
    {
        laserscan.ranges[i] = (laserscan.range_max - laserscan.range_min) * (img->height-1-i)/Scalar(img->height-1) + laserscan.range_min;
        laserscan.intensities[i] = img->data[img->step * i + currentBeamIndex];
    }

    //Publish messages
    imgPubs_.at(msis->GetName()).publish(img);
    imgPubs_.at(msis->GetName() + "/display").publish(disp);
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::LaserScan>>(pubs_.at(msis->GetName() + "/beam"))->publish(laserscan);
}

void ROS2SimulationManager::EnableCurrentsService(const std_srvs::srv::Trigger::Request::SharedPtr req, 
                                           std_srvs::srv::Trigger::Response::SharedPtr res)
{
    (void)req;
    GetOcean()->EnableVelocityFields();
    res->message = "Ocean current simulation enabled.";
    res->success = true;
}

void ROS2SimulationManager::DisableCurrentsService(const std_srvs::srv::Trigger::Request::SharedPtr req, 
                                            std_srvs::srv::Trigger::Response::SharedPtr res)
{   
    (void)req;
    GetOcean()->DisableVelocityFields();
    res->message = "Ocean current simulation disabled.";
    res->success = true;
}

void ROS2SimulationManager::PauseService(const std_srvs::srv::SetBool::Request::SharedPtr req, 
                                           std_srvs::srv::SetBool::Response::SharedPtr res)
{
    SetPaused(req->data);
    res->message = req->data ? "Simulation paused." : "Simulation running.";
    res->success = true;
}

void ROS2SimulationManager::StepService(const stonefish_ros2::srv::Step::Request::SharedPtr req, 
                                           stonefish_ros2::srv::Step::Response::SharedPtr res)
{
    if(!IsPaused())
    {
        res->time = GetSimulationTime();
        res->message = "Only a paused simulation can be stepped.";
        res->success = false;
        return;
    }
    //The steps are simulated by the simulation thread, as fast as possible
    res->time = GetSimulationTime() + (Scalar)(GetRequestedSteps() + req->steps)/GetStepsPerSecond();
    RequestSteps(req->steps);
    res->message = "Stepping the simulation.";
    res->success = true;
}

void ROS2SimulationManager::RespawnRobotService(const stonefish_ros2::srv::Respawn::Request::SharedPtr req, 
                             stonefish_ros2::srv::Respawn::Response::SharedPtr res)
{
    Vector3 p(req->origin.position.x, req->origin.position.y, req->origin.position.z);
    Quaternion q(req->origin.orientation.x, req->origin.orientation.y, req->origin.orientation.z, req->origin.orientation.w);
    Transform origin(q, p);
    
    if(RespawnROS2Robot(req->name, origin))
    {
        res->message = "Robot respawned.";
        res->success = true;
    }
    else    
    {
        res->message = "Robot not found.";
        res->success= false;
    }
}

void ROS2SimulationManager::UniformVFCallback(const geometry_msgs::msg::Vector3::SharedPtr msg, Uniform* vf)
{
    vf->SetVelocity(Vector3(msg->x, msg->y, msg->z));   
}

void ROS2SimulationManager::JetVFCallback(const std_msgs::msg::Float64::SharedPtr msg, Jet* vf)
{
    vf->SetOutletVelocity(msg->data);
}

void ROS2SimulationManager::ActuatorOriginCallback(const geometry_msgs::msg::Transform::SharedPtr msg, LinkActuator* act)
{
    Transform T;
    T.setOrigin(Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
    T.setRotation(Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));

    switch(act->GetLinkActuatorType())
    {
        case LinkActuatorType::PUSH:
        case LinkActuatorType::SIMPLE_THRUSTER:
        case LinkActuatorType::THRUSTER:
        case LinkActuatorType::PROPELLER:
        case LinkActuatorType::VBS:
        case LinkActuatorType::LIGHT:
            act->SetRelativeActuatorFrame(T);
            break;

        default:
            RCLCPP_WARN_STREAM(nh_->get_logger(), "Live update of origin frame of actuator '" << act->GetName() << "' not supported!");
            break;
    }
}

void ROS2SimulationManager::TrajectoryCallback(const nav_msgs::msg::Odometry::SharedPtr msg, ManualTrajectory* tr)
{
    Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    Vector3 p(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Vector3 v(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Vector3 omega(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    tr->SetTransform(Transform(q, p));
    tr->SetLinearVelocity(v);
    tr->SetAngularVelocity(omega);
}

void ROS2SimulationManager::SimpleThrusterCallback(const std_msgs::msg::Float64::SharedPtr msg, SimpleThruster* th)
{
    th->SetSetpoint(msg->data, Scalar(0));
}

void ROS2SimulationManager::ThrusterCallback(const std_msgs::msg::Float64::SharedPtr msg, Thruster* th)
{
    th->SetSetpoint(msg->data);
}

void ROS2SimulationManager::PropellerCallback(const std_msgs::msg::Float64::SharedPtr msg, Propeller* prop)
{
    prop->SetSetpoint(msg->data);
}

void ROS2SimulationManager::PushCallback(const std_msgs::msg::Float64::SharedPtr msg, Push* push)
{
    push->SetForce(msg->data);
}

void ROS2SimulationManager::VBSCallback(const std_msgs::msg::Float64::SharedPtr msg, VariableBuoyancy* act)
{
    act->SetFlowRate(msg->data);
}

void ROS2SimulationManager::CommCallback(const std_msgs::msg::String::SharedPtr msg, Comm* comm)
{
    comm->SendMessage(msg->data);
}

void ROS2SimulationManager::SuctionCupService(const std_srvs::srv::SetBool::Request::SharedPtr req,
                            std_srvs::srv::SetBool::Response::SharedPtr res, SuctionCup* suction)
{
    suction->SetPump(req->data);
    if(req->data)
        res->message = "Pump turned on.";
    else 
        res->message = "Pump turned off.";
    res->success = true;
}

void ROS2SimulationManager::SensorService(const std_srvs::srv::SetBool::Request::SharedPtr req,
                                        std_srvs::srv::SetBool::Response::SharedPtr res, Sensor* sens)
{
    {
        std::lock_guard<std::mutex> lock(cameraUseMutex_);
        if(req->data)
            disabledSensors_.erase(sens->GetName());
        else
            disabledSensors_.insert(sens->GetName());
    }
    sens->SetEnabled(req->data);
    if(req->data)
        res->message = "Sensor turned on.";
    else
        res->message = "Sensor turned off.";
    res->success = true;    
}

void ROS2SimulationManager::SensorOriginCallback(const geometry_msgs::msg::Transform::SharedPtr msg, Sensor* sens)
{
    Transform T;
    T.setOrigin(Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
    T.setRotation(Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));

    switch(sens->GetType())
    {
        case SensorType::LINK:
            static_cast<LinkSensor*>(sens)->SetRelativeSensorFrame(T);
            break;

        case SensorType::VISION:
            static_cast<VisionSensor*>(sens)->SetRelativeSensorFrame(T);
            break;

        default:
            RCLCPP_WARN_STREAM(nh_->get_logger(), "Live update of origin frame of sensor '" << sens->GetName() << "' not supported!");
            break;
    }
}

void ROS2SimulationManager::FLSService(const stonefish_ros2::srv::SonarSettings::Request::SharedPtr req,
                                    stonefish_ros2::srv::SonarSettings::Response::SharedPtr res, FLS* fls)
{
    if(req->range_min <= 0 || req->range_max <= 0 || req->gain <= 0 || req->range_min >= req->range_max)
    {
        res->success = false;
        res->message = "Wrong sonar settings!";
    }
    else
    {
        fls->SetRangeMax(req->range_max);
        fls->SetRangeMin(req->range_min);
        fls->SetGain(req->gain);
        res->success = true;
        res->message = "New sonar settings applied.";
    }
}

void ROS2SimulationManager::SSSService(const stonefish_ros2::srv::SonarSettings::Request::SharedPtr req,
                                    stonefish_ros2::srv::SonarSettings::Response::SharedPtr res, SSS* sss)
{
    if(req->range_min <= 0 || req->range_max <= 0 || req->gain <= 0 || req->range_min >= req->range_max)
    {
        res->success = false;
        res->message = "Wrong sonar settings!";
    }
    else
    {
        sss->SetRangeMax(req->range_max);
        sss->SetRangeMin(req->range_min);
        sss->SetGain(req->gain);
        res->success = true;
        res->message = "New sonar settings applied.";
    }
}

void ROS2SimulationManager::MSISService(const stonefish_ros2::srv::SonarSettings2::Request::SharedPtr req,
                                    stonefish_ros2::srv::SonarSettings2::Response::SharedPtr res, MSIS* msis)
{
    if(req->range_min <= 0 || req->range_max <= 0 || req->gain <= 0
       || req->range_min >= req->range_max
       || req->rotation_min < -180.0
       || req->rotation_max > 180.0
       || req->rotation_min >= req->rotation_max)
    {
        res->success = false;
        res->message = "Wrong sonar settings!";
    }
    else
    {
        msis->SetRangeMax(req->range_max);
        msis->SetRangeMin(req->range_min);
        msis->SetGain(req->gain);
        msis->SetRotationLimits(req->rotation_min, req->rotation_max);
        res->success = true;
        res->message = "New sonar settings applied.";
    }
}

void ROS2SimulationManager::ThrustersCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, ROS2Robot* robot)
{
    if(msg->data.size() != robot->thrusterSetpoints_.size())
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Wrong number of thruster setpoints for robot: " << robot->robot_->GetName());
        return;
    }
    for(size_t i=0; i<robot->thrusterSetpoints_.size(); ++i)
        robot->thrusterSetpoints_[i] = msg->data[i];
    robot->thrusterSetpointsChanged_ = true;
}

void ROS2SimulationManager::SupplyVoltageCallback(const std_msgs::msg::Float64::SharedPtr msg, ROS2Robot* robot)
{
    robot->supplyVoltage_ = msg->data;
    robot->supplyVoltageChanged_ = true;
}

void ROS2SimulationManager::PropellersCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, ROS2Robot* robot)
{
    if(msg->data.size() != robot->propellerSetpoints_.size())
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Wrong number of propeller setpoints for robot: " << robot->robot_->GetName());
        return;
    }
    for(size_t i=0; i<robot->propellerSetpoints_.size(); ++i)
        robot->propellerSetpoints_[i] = msg->data[i];
    robot->propellerSetpointsChanged_ = true;
}

void ROS2SimulationManager::RuddersCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, ROS2Robot* robot)
{
    if(msg->data.size() != robot->rudderSetpoints_.size())
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Wrong number of rudder setpoints for robot: " << robot->robot_->GetName());
        return;
    }
    for(size_t i=0; i<robot->rudderSetpoints_.size(); ++i)
        robot->rudderSetpoints_[i] = msg->data[i];
    robot->rudderSetpointsChanged_ = true;
}

void ROS2SimulationManager::ServosCallback(const sensor_msgs::msg::JointState::SharedPtr msg, ROS2Robot* robot)
{
    if(msg->name.size() == 0)
    {
        RCLCPP_ERROR(nh_->get_logger(), "Desired joint state message is missing joint names!");
        return;
    }

    if(msg->position.size() > 0)
    {
        for(size_t i=0; i<msg->position.size(); ++i)
        {
            try
            {
                robot->servoSetpoints_.at(msg->name[i]) = std::make_pair(ServoControlMode::POSITION, (Scalar)msg->position[i]);
            }
            catch(const std::out_of_range& e)
            {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid joint name in desired joint state message: " << msg->name[i]);
            }
        }
    }
    else if(msg->velocity.size() > 0)
    {
        for(size_t i=0; i<msg->velocity.size(); ++i)
        {
            try
            {
                robot->servoSetpoints_.at(msg->name[i]) = std::make_pair(ServoControlMode::VELOCITY, (Scalar)msg->velocity[i]);
            }
            catch(const std::out_of_range& e)
            {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid joint name in desired joint state message: " << msg->name[i]);
            }
        }
    }
    else if(msg->effort.size() > 0)
    {
        RCLCPP_ERROR(nh_->get_logger(), "No effort control mode implemented in simulation!");
    }
}

void ROS2SimulationManager::JointCallback(const std_msgs::msg::Float64::SharedPtr msg, ROS2Robot* robot, 
                                                            ServoControlMode mode, const std::string& jointName)
{
    try
    {
        robot->servoSetpoints_.at(jointName) = std::make_pair(mode, (Scalar)msg->data);
    }
    catch(const std::out_of_range& e)
    {
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid joint name: " << jointName);
    }
}

void ROS2SimulationManager::JointGroupCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, ROS2Robot* robot, 
                                                            ServoControlMode mode, const std::vector<std::string>& jointNames)
{
    if(msg->data.size() != jointNames.size())
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Wrong size of joint group message! Required: " << jointNames.size() << " Received: " << msg->data.size());
        return;
    }
    for(size_t i=0; i<jointNames.size(); ++i)
    {
        try
        {
            robot->servoSetpoints_.at(jointNames[i]) = std::make_pair(mode, (Scalar)msg->data[i]);
        }
        catch(const std::out_of_range& e)
        {
            RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid joint name: " << jointNames[i]);
        }
    }
}

void ROS2SimulationManager::GlueService(const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res, FixedJoint* fix)
{
    if(req->data)
    {
        fix->RemoveFromSimulation(this);
        fix->UpdateDefinition();
        fix->AddToSimulation(this);
        res->message = "Glue activated.";
    }
    else
    {
        fix->RemoveFromSimulation(this);  
        res->message = "Glue deactivated.";
    }
    res->success = true;
}

void ROS2SimulationManager::LightService(const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res, Light* light)
{
    light->Switch(req->data);
    if(req->data)
        res->message = "Light turned on.";
    else
        res->message = "Light turned off.";
    res->success = true;
}

rcl_interfaces::msg::SetParametersResult ROS2SimulationManager::ParametersCallback(const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;

    //Validate all parameters before applying any
    for(const rclcpp::Parameter& p : params)
    {
        if(p.get_name() == "publish_clock" || p.get_name() == "seed" || p.get_name() == "lock_step" 
           || p.get_name() == "start_paused" || p.get_name() == "offscreen")
        {
            res.successful = false;
            res.reason = "Parameter '" + p.get_name() + "' can only be set at startup.";
            return res;
        }
        else if(p.get_name() == "realtime_factor" && p.as_double() <= 0.0)
        {
            res.successful = false;
            res.reason = "Parameter 'realtime_factor' has to be positive.";
            return res;
        }
    }

    for(const rclcpp::Parameter& p : params)
    {
        if(p.get_name() == "realtime_factor")
            SetRealtimeFactor(p.as_double());
    }
    return res;
}

}