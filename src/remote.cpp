#include <iostream>
#include "arduino.h"
#include "maestro.h"

using namespace robot;

int main(){
/*	
	Maestro *mae = new Maestro();
	mae->setup("/dev/ttyUSB0");
        
*/
  
  
  Arduino *ard = new Arduino();
  ard->setup("/dev/ttyACM0");
  int sensorId = 0;
  float sensorVal;
  for(;;){
    std::cin >> sensorId;
    
    try{
      ard->getSensor(sensorId, &sensorVal);
    }catch(...){
      std::cout<< "Read Error! :(" << std::endl;
    }
    
    std::cout << "Sensor " << sensorId<< " just read " << sensorVal <<std::endl;
  }
  return 0;
}
