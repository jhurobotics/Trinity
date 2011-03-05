#define VALUE_SIZE 4
#define MAX_OUTPUT_SIZE 8

class Command {
  public:
    static int com_end = 0x00;
    static int com_get_sensor = 0x01;
    static int com_get_sensor_response = 0x02;
    static int com_set_motor = 0x03;
    static int com_set_motor_response = 0x04;
    static int com_set_sensor = 0x05;
    static int com_set_sensor_response = 0x06;
    static int com_led = 0xff;
    int last_command;
    int ID;
    int value[VALUE_SIZE];
    int count;
    byte output[MAX_OUTPUT_SIZE];
    int output_size;
    Command() : last_command(0), ID(0), count(0), output_size(0) {
      for( int i=0; i < VALUE_SIZE; ++i) {
        value[i] = 0;
      }
      for( int i=0; i < MAX_OUTPUT_SIZE; ++i) {
        output[i] = 0;
      }
    }
    // store next byte in the command sequence
    next(int b) {
      if(count == 0) {    // command head
        last_command = b;
      }
      else { // command body
        
      }
      count++;
    }
    
    // returns true if theres information the arduino wants to send to the computer
    bool available() {
      return output_size>0;  // will be non-zero if theres output waiting to be sent
    }
    
    // write out the response data
    void send_output() {
      for(int i=0; i<output_size; ++i) {
        Serial.write(output[i]);
      }
      output_size=0;
    }
};


Command com = Command();
void setup() {
  Serial.begin(9600);
  
}


void loop() {
  if(Serial.available() > 0) {
    com.next(Serial.read());
  }
}
