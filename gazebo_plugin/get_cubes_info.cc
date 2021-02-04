#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <iostream>

namespace gazebo
{
  class GetCubesInfo : public WorldPlugin
  {
    public: unsigned int modelCount = 0;

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {

      modelCount = _world->ModelCount();
      for (int i=0; i<modelCount; i++){
        physics::ModelPtr m = _world->ModelByIndex(i);

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

        std::cout << modelName << ": x:" << x << " y:" << y << " z:" << z << "\n";
        }
      }

    }
  };

  GZ_REGISTER_WORLD_PLUGIN(GetCubesInfo)
}
