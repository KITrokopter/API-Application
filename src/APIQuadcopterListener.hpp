#pragma once

namespace kitrokopter {

class APIQuadcopterListener {

 public:

    virtual void updateQuadrocopterValues(APIPackage::APIQuadcopterUpdateEvent e)  = 0;

    // virtual destructor for interface 
    virtual ~APIQuadcopterListener() { }

};

}
