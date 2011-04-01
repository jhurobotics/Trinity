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

void Serial::Open(const char * path) throw(Serial::OpenError)
{
  fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK );
  if( fd < 0 ) {
    throw OpenError("could not open file");
  }
  struct termios  config;
  if(!isatty(fd)) {
    throw OpenError(std::string(path) + "is not a tty");
  }
  if(tcgetattr(fd, &config) < 0) {
    throw OpenError("could not get attributes\n");
  }
  //
  // Input flags - Turn off input processing
  // convert break to null byte, no CR to NL translation,
  // no NL to CR translation, don't mark parity errors or breaks
  // no input parity check, don't strip high bit off,
  // no XON/XOFF software flow control
  //
  config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                      INLCR | PARMRK | INPCK | ISTRIP | IXON);
  //
  // Output flags - Turn off output processing
  // no CR to NL translation, no NL to CR-NL translation,
  // no NL to CR translation, no column 0 CR suppression,
  // no Ctrl-D suppression, no fill characters, no case mapping,
  // no local output processing
  //
   config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
                       ONOCR | /*ONOEOT|*/ OFILL  | OPOST);
  //config.c_oflag = 0;
  //
  // No line processing:
  // echo off, echo newline off, canonical mode off, 
  // extended input processing off, signal chars off
  //
  config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
  //
  // Turn off character processing
  // clear current char size mask, no parity checking,
  // no output processing, force 8 bit input
  //
  config.c_cflag &= ~(CSIZE | PARENB);
  config.c_cflag |= CS8;
  //
  // One input byte is enough to return from read()
  // Inter-character timer off
  //
  config.c_cc[VMIN]  = 1;
  config.c_cc[VTIME] = 0;
  //
  // Communication speed (simple version, using the predefined
  // constants)
  //
  if(cfsetispeed(&config, get_baud_constant(baud)) < 0 || cfsetospeed(&config, get_baud_constant(baud)) < 0) {
    throw OpenError("could not set baud rate");
  }
  //
  // Finally, apply the configuration
  //
  if(tcsetattr(fd, TCSAFLUSH, &config) < 0) {
    throw OpenError("could not set configuration\n");
  }
}

Serial::~Serial() throw() {
  if( fd > 0 ) {
    close(fd);
  }
}

void Serial::Write(const char * buff, size_t length) throw(Serial::WriteError) {
  size_t bytes = 0;
  if( length > SSIZE_MAX ) {
    throw WriteError(-1);
  }
  do {
    ssize_t err = write(fd, buff, length-bytes);
    if( err >= 0 ) {
      bytes += err;
    }
    else {
      return;
      switch(errno) {
      case EAGAIN:
          break;
      case EBADF:
      case EFAULT:
      case EINTR:
      case EINVAL:
      case EIO:
      case ENOSPC:
      case EPIPE:
      default:
        throw WriteError(errno);
      }
    }
  }
  while( bytes < length );
}

// keep reading until count bytes are read
void Serial::Read(char * buff, size_t count) throw(Serial::ReadError) {
  size_t bytes = 0;
  do {
    ssize_t err = read(fd, buff+bytes, count-bytes);
    if( err >= 0 ) {
      bytes += err;
    }
    else {
      switch( errno ) {
      case EAGAIN:
          break;
      case EBADF:
      case EFAULT:
      case EINTR:
      case EINVAL:
      case EIO:
      case EISDIR:
      default:
        throw ReadError(errno);
      }
    }
  }
  while(bytes < count);
}
