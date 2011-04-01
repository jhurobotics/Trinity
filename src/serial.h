/* -*-C++-*-
 * Interface to the serial port
 */

#include <exception>
#include <stdlib.h>
#include <string>

#ifndef __SERIAL_H__
#define __SERIAL_H__

namespace robot {
  class Serial {
    int fd;
    int baud;
    const char * devicePath;
    
  public:
    class OpenError : public std::exception {
    protected:
      std::string msg;
    public:
      OpenError(const std::string& str) throw() : msg(str) { }
      ~OpenError() throw() { }
      const char * what() throw() {
        return msg.c_str();
      }
    };
    class WriteError : public std::exception {
    public:
      int err;
      WriteError(int e) throw() : err(e) { }
    };
    class ReadError : public std::exception{
    public:
      int err;
      ReadError(int e) throw() : err(e) { }
    };
    
    Serial() throw() {
      fd = 0;
      baud = 9600;
    }
    ~Serial() throw();
    
    void Open(const char * path) throw(Serial::OpenError);
    void Write(const char * data, size_t length) throw(Serial::WriteError);
    // reads until count bytes are read
    void Read(char * buff, size_t count) throw(Serial::ReadError);
  };
}

#endif
