#include <errno.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <limits.h>
#include "serial.h"
using namespace robot;

#define CASE(b) case b:\
  return B##b

static speed_t get_baud_constant(long baud) {
  switch( baud ) {
    CASE(0);
    CASE(50);
    CASE(75);
    CASE(110);
    CASE(134);
    CASE(150);
    CASE(200);
    CASE(300);
    CASE(600);
    CASE(1200);
    CASE(2400);
    CASE(4800);
    CASE(9600);
    CASE(19200);
    CASE(38400);
  default:
    return -1;
  }
}

void Serial::Open(const char * path)
{
  fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
}

Serial::~Serial() {
  if( fd > 0 ) {
    close(fd);
  }
}

void Serial::Write(const char * buff, size_t length) throw(Serial::WriteError) {
  size_t bytes = 0;
  if( length > SSIZE_MAX ) {
    throw WriteError();
  }
  do {
    ssize_t err = write(fd, buff, length);
    if( err >= 0 ) {
      bytes += err;
    }
    else {
      switch(errno) {
      case EAGAIN:
      case EBADF:
      case EFAULT:
      case EINTR:
      case EINVAL:
      case EIO:
      case ENOSPC:
      case EPIPE:
      default:
        throw WriteError();
      }
    }
  }
  while( bytes < length );
}

// keep reading until count bytes are read
void Serial::Read(char * buff, size_t count) throw(Serial::ReadError) {
  size_t bytes = 0;
  do {
    ssize_t err = read(fd, buff, count - bytes);
    if( err >= 0 ) {
      bytes += err;
    }
    else {
      switch( errno ) {
      case EAGAIN:
      case EBADF:
      case EFAULT:
      case EINTR:
      case EINVAL:
      case EIO:
      case EISDIR:
      default:
        throw ReadError();
      }
    }
  }
  while(bytes < count);
}
