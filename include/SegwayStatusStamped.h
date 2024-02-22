#ifndef _segway_rmp_SegwayStatusStamped_h
#define _segway_rmp_SegwayStatusStamped_h

#include "SegwayStatus.h"
#include "std_msgs/msg/header.hpp"


namespace segway_rmp {

    class SegwayStatusStamped {
        
        public:
            segway_rmp::SegwayStatus segway;
            std_msgs::msg::Header header;

        SegwayStatusStamped(): 
            segway(),
            header() {
        }
    };
};
#endif