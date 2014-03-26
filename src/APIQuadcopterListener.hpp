#pragma once

#include "APIQuadcopterUpdateEvent.hpp"

namespace kitrokopter {

class APIQuadcopterListener {

 public:

    virtual void updateQuadcopterValues(APIQuadcopterUpdateEvent e)  = 0;

    // virtual destructor for interface 
    virtual ~APIQuadcopterListener() { }

};

}
