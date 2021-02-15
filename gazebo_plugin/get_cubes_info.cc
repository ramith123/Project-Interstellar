#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <iostream>
#include <string>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/String.h"

namespace gazebo
{
  class GetCubesInfo : public WorldPlugin
  {
    public: unsigned int modelCount = 0;
    public: std::string cubeLocations = "";
    public: physics::WorldPtr world;
    public: sdf::ElementPtr sdf;

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      world = _world;
      sdf = _sdf;

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to the Gazebo node
      ros::NodeHandle rosNode;


      // Create a named topic, and subscribe to it.
      ros::Publisher sendCubeLocations = rosNode.advertise<std_msgs::String>("locations", 1000);
      ros::Rate loop_rate(1);
      cubeLocations = "\n";
      modelCount = this->world->ModelCount();
      for (int i=0; i<modelCount; i++){
        physics::ModelPtr m = this->world->ModelByIndex(i);
        world->PublishModelPose(m);
        // Get name of model/entity
        std::string modelName =  m->GetSDF()->GetAttribute("name")->GetAsString();

        // Only proceeds if the name of the model has the word "cube" or "box" in it.
        if (modelName.find("box")!= std::string::npos||modelName.find("cube")!= std::string::npos){
        // Get pose of model/entity
        double x,y,z;
        ignition::math::Pose3d pose;     
        pose = m->WorldPose();
        ignition::math::Vector3<double> v1(0, 0, 0);
        v1 = pose.Pos();
        x = v1.X(); // x coordinate
        y = v1.Y(); // y coordinate
        z = v1.Z(); // z coordinate

        cubeLocations = cubeLocations + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "\n";
        }
      }

      while (ros::ok()){
        std_msgs::String msg;

        std::stringstream ss;
        ss << this->cubeLocations;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        sendCubeLocations.publish(msg);
        loop_rate.sleep();
      }
    }
  };
  GZ_REGISTER_WORLD_PLUGIN(GetCubesInfo)
}
