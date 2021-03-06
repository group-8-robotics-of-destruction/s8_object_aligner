#ifndef __OBJECT_ALIGNER_NODE_H
#define __OBJECT_ALIGNER_NODE_H

#include <s8_motor_controller/motor_controller_node.h>

namespace s8 {
    namespace object_aligner_node {
        const std::string NODE_NAME =               "s8_object_aligner_node";

        const std::string TOPIC_TWIST =             s8::motor_controller_node::TOPIC_TWIST;
        const std::string TOPIC_OBJECT_DIST_POSE =  "/s8/ip/classification/distPose";

        const std::string ACTION_STOP =             s8::motor_controller_node::ACTION_STOP;
        const std::string ACTION_OBJECT_ALIGN =     "/s8/object_align";

        const std::string CONFIG_DOC =              "/catkin_ws/src/s8_object_aligner/parameters/parameters.json";

        enum ObjectAlignFinishedReason {
            TIMEOUT,
            FAILED,
            PREEMPTED,
            ALIGNED
        };
    }
}

#endif
