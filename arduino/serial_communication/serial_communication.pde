#define SONAR_COUNT     4
#define SONAR_PIN_START 0

static float sonarVals[SONAR_COUNT];

#define ENCODER_COUNT   2
volatile long encoderVals[ENCODER_COUNT];

#define SENSOR_COUNT 6
// index into the array is the id of that sensor
unsigned char* sensorVals[SENSOR_COUNT+1];

#define MOTOR_PIN_START

#define LED_PIN 13

enum name_t {
  GET = 0x01,
  SET_MOTOR = 0x02,
  SET_SENSOR = 0x03,
  SET_LIGHT = 0x04,
};

static const unsigned char MOTOR_FLAG = 0x80;
static const unsigned char END = 0xFF;

typedef struct get_command_t {
  unsigned char name;
  unsigned char id;
  unsigned char end;
} get_command_t;
typedef struct set_command_t {
  unsigned char name;
  unsigned char id;
  long val;
  unsigned char end;
} set_command_t;
typedef struct light_command_t {
  unsigned char name;
  unsigned char state;
  unsigned char end;
} light_command_t;


static unsigned char read() {
  int result = 0;
  do {
    result = Serial.read();
  } while( result != -1 );
  return (unsigned char)result;
}

static bool read(unsigned char * b) {
  int result = 0;
  for( unsigned char i = 0; i < 10; i++ ) {
    result = Serial.read();
    if( result != -1 ) {
      (*b) = (unsigned char)result;
      return true;
    }
  }
  return false;
}

static void fail() {
  // Arduino compiler won't let me &END in the last line,
  // So use an extra unsigned char ... sigh ...
  while( Serial.read() != -1 ) ;
  unsigned char buff = END;
  Serial.write(&buff, 1);
}

void getValue(struct get_command_t get_cmd) {
  struct resp {
    unsigned char name;
    unsigned char id;
    unsigned char val[4];
    unsigned char end;
  };
  union {
    unsigned char buff[sizeof(struct resp)];
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
  for( unsigned char i = 0; i < sizeof(long); i++ ) {
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
    unsigned char name;
    unsigned char id;
    long val;
    unsigned char end;
  };
  union {
    unsigned char buff[sizeof(struct resp)];
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
  
  Serial.write((unsigned char*)&light_cmd, sizeof(light_cmd));
}

void parseCommand() {
  union {
    unsigned char data[7];
    get_command_t get_cmd;
    set_command_t set_cmd;
    light_command_t light_cmd;
  };
  data[0] = (name_t)read();
  // I need to read at least 2 more unsigned chars, so do it now:
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
      // read in 4 more unsigned chars
      for( unsigned char i = 0; i < 4; i++) {
        if( !read(&data[i+3]) ) {
          fail();
          return;
        }
      }
      // the last unsigned char in both cases must be END, so check here
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

void setup() {
  pinMode(13, OUTPUT);
  // These values are specified in the robot configuration
  int sensCount = 0;
  for( int i = 0; i < ENCODER_COUNT; i++ ) {
    sensorVals[++sensCount] = (unsigned char*)(encoderVals+i);
  }
  for( int i = 0; i < SONAR_COUNT; i++ ) {
    sensorVals[++sensCount] = (unsigned char*)(sonarVals+i);
  }
  
  Serial.begin(9600);
  
}

void loop() {
  if(Serial.available() > 0) {
    parseCommand();
  }
}
