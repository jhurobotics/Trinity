/* -*-C++-*-
 * Arduino software for the
 * JHU Robotics Team Trinity Fire Fighting Robot
 */
#define SONAR_COUNT     4
#define SONAR_PIN_START 0

static float sonarVals[SONAR_COUNT];

#define ENCODER_COUNT   2
#define ENCODER_PIN_START 4
#define ENC_CHAN_1 0x01
#define ENC_CHAN_2 0x02
volatile long encoderVals[ENCODER_COUNT];
// The order in which the states should occur
static byte stateMap[4] = {
  0,
  1,
  3,
  2
};
volatile byte encoderState[ENCODER_COUNT];

inline byte encoderStatus(byte pinStart) {
  byte result = 0;
  if( digitalRead(pinStart) == HIGH ) {
    result |= ENC_CHAN_1;
  }
  if( digitalRead(pinStart+1) == HIGH ) {
    result |= ENC_CHAN_2;
  }
  return stateMap[result];
}

// these counters may be backwards
void encoder_tick(byte pin) {
  byte newState = encoderStatus(ENCODER_PIN_START + pin*2);
  byte dir = (newState - encoderState[pin-1] + 4) % 4;
  if( dir == 1 ) {
    encoderVals[pin]++;
  }
  else if( dir == 3) {
    encoderVals[pin]--;
  }
  else { // we missed a tick, this is bad, so just guess
    encoderVals[pin] += dir;
  }
}

void left_encoder_tick() {
  encoder_tick(0);
}

void right_encoder_tick() {
  encoder_tick(1);
}

#define SENSOR_COUNT 6
// index into the array is the id of that sensor
byte* sensorVals[SENSOR_COUNT+1];

#define MOTOR_PIN_START 8

#define LED_PIN 13

enum name_t {
  GET = 0x01,
  SET_MOTOR = 0x02,
  SET_SENSOR = 0x03,
  SET_LIGHT = 0x04,
};

static const byte MOTOR_FLAG = 0x80;
static const byte END = 0xFF;

typedef struct get_command_t {
  byte name;
  byte id;
  byte end;
} get_command_t;
typedef struct set_command_t {
  byte name;
  byte id;
  long val;
  byte end;
} set_command_t;
typedef struct light_command_t {
  byte name;
  byte state;
  byte end;
} light_command_t;


static byte read() {
  int result = 0;
  do {
    result = Serial.read();
  } while( result != -1 );
  return (byte)result;
}

static bool read(byte * b) {
  int result = 0;
  for( byte i = 0; i < 10; i++ ) {
    result = Serial.read();
    if( result != -1 ) {
      (*b) = (byte)result;
      return true;
    }
  }
  return false;
}

static void fail() {
  // Arduino compiler won't let me &END in the last line,
  // So use an extra byte ... sigh ...
  while( Serial.read() != -1 ) ;
  byte buff = END;
  Serial.write(&buff, 1);
}

static void getValue(struct get_command_t get_cmd) {
  struct resp {
    byte name;
    byte id;
    byte val[4];
    byte end;
  };
  union {
    byte buff[sizeof(struct resp)];
    struct resp resp;
  };
  
  // check the validity of the request
  if( get_cmd.id > SENSOR_COUNT ) {
    fail();
    return;
  }
  if( get_cmd.end != END ) {
    fail();
    return;
  }
  
  resp.name = GET;
  resp.id = get_cmd.id;
  for( byte i = 0; i < sizeof(long); i++ ) {
    resp.val[i] = sensorVals[get_cmd.id][i];
  }
  resp.end = END;
  Serial.write(buff, sizeof(buff));
}

static void setMotor(struct set_command_t set_cmd) {
  // leaving this out, probably use Maestro for PWM
}

static void setSensor(struct set_command_t set_cmd) {
  // Send the response back
  struct resp {
    byte name;
    byte id;
    long val;
    byte end;
  };
  union {
    byte buff[sizeof(struct resp)];
    struct resp resp;
  };
  
  if( set_cmd.id > ENCODER_COUNT ) {
    fail();
    return;
  }
  
  noInterrupts();
  resp.val = encoderVals[set_cmd.id];
  encoderVals[set_cmd.id] = set_cmd.val;
  interrupts();
  
  resp.name = SET_SENSOR;
  resp.id = set_cmd.id;
  resp.end = END;
  Serial.write(buff, sizeof(buff));
}

static void setLight(struct light_command_t light_cmd) {
  if( light_cmd.end != END ) {
    fail();
    return;
  }
  
  digitalWrite(LED_PIN, (light_cmd.state ? HIGH : LOW ));
  
  Serial.write((byte*)&light_cmd, sizeof(light_cmd));
}

void parseCommand() {
  union {
    byte data[7];
    get_command_t get_cmd;
    set_command_t set_cmd;
    light_command_t light_cmd;
  };
  data[0] = (name_t)read();
  // I need to read at least 2 more bytes, so do it now:
  if( !read(&data[1]) ) {
    fail();
    return;
  }
  if( !read(&data[2]) ) {
    fail();
    return;
  }
  switch (data[0]) {
    case GET:
      getValue(get_cmd);
      break;
    case SET_MOTOR:
    case SET_SENSOR:
      // read in 4 more bytes
      for( byte i = 0; i < 4; i++) {
        if( !read(&data[i+3]) ) {
          fail();
          return;
        }
      }
      // the last byte in both cases must be END, so check here
      if( data[6] != END ) {
        fail();
        return;
      }
      if( set_cmd.name == SET_MOTOR ) {
        setMotor(set_cmd);
      }
      else {
        setSensor(set_cmd);
      }
      break;
    case SET_LIGHT:
      setLight(light_cmd);
      break;
  }
}

void ping(int id) {
  unsigned long startTime, endTime;
  byte pin = id + SONAR_PIN_START;
  
  // trigger the ping
  pinMode(pin, OUTPUT);
  digitalWrite(id, HIGH);
  delayMicroseconds(5);
  digitalWrite(id, LOW);
  pinMode(id, INPUT);
  
  while( digitalRead(id) == LOW ) ; // wait for the holdoff time
  startTime = micros(); // record the time the signal goes high
  while( digitalRead(id) == HIGH ) ;
  endTime = micros();
  
  sonarVals[id] = (endTime - startTime)*0.0343;
}

void setup() {
  pinMode(13, OUTPUT);
  // These values are specified in the robot configuration
  int sensCount = 0;
  for( int i = 0; i < ENCODER_COUNT; i++ ) {
    encoderVals[i] = 0;
    encoderState[i] = encoderStatus(i);
    sensorVals[++sensCount] = (byte*)(encoderVals+i);
  }
  for( int i = 0; i < SONAR_COUNT; i++ ) {
    sonarVals[i] = 0;
    sensorVals[++sensCount] = (byte*)(sonarVals+i);
  }
  
  attachInterrupt(0, left_encoder_tick, CHANGE);  
  attachInterrupt(1, right_encoder_tick, CHANGE);
  
  Serial.begin(9600);
  while( Serial.available() > 0 ) {
    parseCommand();
  }
}

void loop() {
  for( byte i = 0; i < SONAR_COUNT; i++ ) {
    ping(i);
    while( Serial.available() > 0 ) {
      parseCommand();
    }
  }
}
