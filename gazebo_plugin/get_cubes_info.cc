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
  class GetCubesInfo : public ModelPlugin
  {

    ros::Publisher sendCubeLocations;
    // Pointer to the model
  private:
    physics::ModelPtr model;

    // Pointer to the update event connection
  private:
    event::ConnectionPtr updateConnection;

  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)

    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      ros::NodeHandle rosNode;
      sendCubeLocations = rosNode.advertise<std_msgs::String>("locations", 1000);
      std::cout << "Loaded";
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectBeforePhysicsUpdate(std::bind(&GetCubesInfo::OnUpdate, this));
      // this->publishToTopic();
    }

    // Called by the world update start event
  public:
    void OnUpdate()

    {
      std::cout << "Update";
      this->publishToTopic();
    }

  public:
    void publishToTopic()
    {

      // Get the pose of the model, and store it in a Vector.
      ignition::math::Vector3<double> v1(0, 0, 0);
      ignition::math::Pose3d pose;

      pose = this->model->WorldPose();
      v1 = pose.Pos();
      double x = v1.X(); // x coordinate
      double y = v1.Y(); // y coordinate
      double z = v1.Z(); // z coordinate

      std::string cubeLocations = this->model->GetScopedName() + "," + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "\n";
      std_msgs::String msg;
      // std::cout << cubeLocations;
      std::stringstream ss;
      ss << cubeLocations;
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());
      sendCubeLocations.publish(msg);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GetCubesInfo)
}

/* OLD CODE 
{
  class GetCubesInfo : public WorldPlugin
  {
    public: unsigned int modelCount = 0;
    public: std::string cubeLocations = "";
    public: physics::WorldPtr world;
    public: static sdf::ElementPtr sdf;

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

      cubeLocations = "\n";
      modelCount = this->world->ModelCount();
      for (int i=0; i<modelCount; i++){
        physics::ModelPtr m = CustomModel();
        m = this->world->ModelByIndex(i);

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
      runTopic(cubeLocations);
    }

    public: static void runTopic(std::string poses){
      // Create our ROS node. This acts in a similar manner to the Gazebo node
      ros::NodeHandle rosNode;

      // Create a named topic, and subscribe to it.
      ros::Publisher sendCubeLocations = rosNode.advertise<std_msgs::String>("locations", 1000);

      // Set the rate at which the while loop runs.
      //ros::Rate loop_rate(1);
     // while (ros::ok()){
      std_msgs::String msg;

      std::stringstream ss;
      ss << poses;
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());
      sendCubeLocations.publish(msg);
      //loop_rate.sleep();
      //}
    }
  };
  GZ_REGISTER_WORLD_PLUGIN(GetCubesInfo)

  */
