/* -*-C++-*-
 * Interface to the serial port
 */

#include <exception>
#include <stdlib.h>

#ifndef __SERIAL_H__
#define __SERIAL_H__

namespace robot {
  class Serial {
    int fd;
    int baud;
    const char * devicePath;
    
  public:
    class WriteError : public std::exception {
    };
    
    class ReadError : public std::exception{
    };
    
    Serial() {
      fd = 0;
      baud = 0;
    }
    ~Serial();
    
    void Open(const char * path);
    void Write(const char * data, size_t length) throw(Serial::WriteError);
    // reads until count bytes are read
    void Read(char * buff, size_t count) throw(Serial::ReadError);
  };
}

#endif
