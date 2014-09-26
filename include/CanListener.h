#ifndef __CAN_LISTENER__
#define __CAN_LISTENER__

#include "RobotState.h"

class CanListener {
    protected:
        RobotState *state;

    public:
        CanListener(RobotState *state):state(state) {};
        void run();

};

#endif
