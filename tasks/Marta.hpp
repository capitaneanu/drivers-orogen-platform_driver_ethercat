#ifndef PLATFORM_DRIVER_MARTA_TASK_HPP
#define PLATFORM_DRIVER_MARTA_TASK_HPP

#include "platform_driver/MartaBase.hpp"

namespace platform_driver
{
class Marta : public MartaBase
{
    friend class MartaBase;

  protected:
    void setJointCommands();
    void getJointInformation();
    void getFtsInformation();

    int numFts;
    base::samples::Wrenches fts_readings;
  public:
    Marta(std::string const& name = "platform_driver::Marta");
    Marta(std::string const& name, RTT::ExecutionEngine* engine);
    ~Marta();

    bool configureHook();
    void updateHook();
};
}

#endif
