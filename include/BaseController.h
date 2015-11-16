#ifndef __BASE_CONTROLLER__
#define __BASE_CONTROLLER__

#include "RobotState.h"
#include <boost/thread/mutex.hpp>
#include "geometry_msgs/Twist.h"

class BaseController {
    protected:
        RobotState *state;
        boost::mutex mutex;
        geometry_msgs::Twist cmd;

    public:
        void setTargetVelocity(const geometry_msgs::Twist &twist);
        BaseController(RobotState *state):state(state) {
        };
        void run(); ///Foo blabla

};

#endif
