#pragma once

#include "APIQuadcopterUpdateEvent.hpp"

namespace kitrokopter {

class APIQuadcopterListener {

 public:

    virtual void updateQuadrocopterValues(APIQuadcopterUpdateEvent e)  = 0;

    // virtual destructor for interface 
    virtual ~APIQuadcopterListener() { }

};

}
