#ifndef ROBOT_MANAGER_PLUGIN_BASE_JACOBIAN_HPP
#define ROBOT_MANAGER_PLUGIN_BASE_JACOBIAN_HPP

namespace control_plugin_base
{
  class Jacobian {
    public:
      virtual void jacobian() = 0;
      virtual ~Jacobian(){}
    
    protected:
      Jacobian(){}
  };
}

#endif