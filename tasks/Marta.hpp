#pragma once

#include "platform_driver_ethercat/MartaBase.hpp"

namespace platform_driver_ethercat
{
class Marta : public MartaBase
{
    friend class MartaBase;

  protected:
    void setJointCommands();
    void getJointInformation();
    void getFtsInformation();

    int num_fts_;
    base::samples::Wrenches fts_readings_;

  public:
    Marta(std::string const& name = "platform_driver_ethercat::Marta");
    Marta(std::string const& name, RTT::ExecutionEngine* engine);
    ~Marta();

    bool configureHook();
    void updateHook();
};
}  // namespace platform_driver
