//
//  maestro.h
//  sim
//

#include "serial.h"
#include "robot.h"

#ifndef __MAESTRO_H__
#define __MAESTRO_H__

namespace robot {
  
  class Maestro {
  protected:
    Serial serial;
    
  public:
    Maestro() throw() { };
    
    void setup(const char * path);
    void setChannel(id_t channel, uint16_t value);
  };

}

#endif // __MAESTRO_H__