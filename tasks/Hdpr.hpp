#ifndef PLATFORM_DRIVER_HDPR_TASK_HPP
#define PLATFORM_DRIVER_HDPR_TASK_HPP

#include "platform_driver/HdprBase.hpp"

namespace platform_driver{

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
    };
}

#endif
