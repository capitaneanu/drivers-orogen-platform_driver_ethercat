#pragma once

#include "platform_driver/HdprBase.hpp"
#include <memory>

class PlatformDriverPcan;

namespace platform_driver
{

class Hdpr : public HdprBase
{
    friend class HdprBase;

  protected:
    void setJointCommands();
    void getJointInformation();

  public:
    Hdpr(std::string const& name = "platform_driver::Hdpr");
    Hdpr(std::string const& name, RTT::ExecutionEngine* engine);
    ~Hdpr();
    bool configureHook();
};
}  // namespace platform_driver
