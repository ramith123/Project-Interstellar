#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
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
        std::cout << m->GetSDF()->ToString("");
      }

    }
  };

  GZ_REGISTER_WORLD_PLUGIN(GetCubesInfo)
}
