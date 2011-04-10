#include <iostream>
#include "arduino.h"

//#include "maestro.h"


using namespace robot;

int main(){
/*	
	Maestro *mae = new Maestro();
	mae->setup("/dev/ttyUSB0");
        
*/
  
  
  Arduino *ard = new Arduino();
  ard->setup("/dev/ttyACM0");
  int sensorId = 0;
  int encoderVal;
  float sonarVal;
  for(;;){

		std::cout<< "\n Please enter a sensor to query. (1-2 for encoder, 3-6 sonars " << std::endl;
    std::cin >> sensorId;
    
		if(sensorId == 1 || sensorId == 2){
			try{
		    ard->getSensor(sensorId, &encoderVal);
		  }catch(...){
		    std::cout<< "Read Error! :(" << std::endl;
		  }

	    std::cout << "Sensor (encoder) " << sensorId<< " just read " << encoderVal <<std::endl;
		}else if(sensorId == 3 || sensorId == 4 || sensorId == 5 || sensorId == 6){
		  try{
		    ard->getSensor(sensorId, &sonarVal);
		  }catch(...){
		    std::cout<< "Read Error! :(" << std::endl;
		  }

	    std::cout << "Sensor (sonar) " << sensorId<< " just read " << sonarVal <<std::endl;
		}else{
			std::cout << "Invalid sensor ID" <<std::endl;
		}

  }
  return 0;
}
