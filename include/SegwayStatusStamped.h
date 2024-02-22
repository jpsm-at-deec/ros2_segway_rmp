#ifndef _segway_rmp_SegwayStatusStamped_h
#define _segway_rmp_SegwayStatusStamped_h

#include "SegwayStatus.h"

namespace segway_rmp {

    class SegwayStatusStamped {
        
        public:
            segway_rmp::SegwayStatus segway;
            

        SegwayStatusStamped(): 
            segway() {
        }
    };
};
#endif