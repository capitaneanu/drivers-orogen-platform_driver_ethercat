#pragma once

#include "platform_driver/ExoterBase.hpp"

namespace platform_driver
{
class Exoter : public ExoterBase
{
    friend class ExoterBase;

  protected:
    void setJointCommands();
    void getJointInformation();

  public:
    Exoter(std::string const& name = "platform_driver::Exoter");
    Exoter(std::string const& name, RTT::ExecutionEngine* engine);
    ~Exoter();
    bool configureHook();
};
}  // namespace platform_driver
