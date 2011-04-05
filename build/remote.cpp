#include <iostream>
#include "../src/arduino.h"

using namespace robot;

int main(){
	
	Arduino *ard = new Arduino();
	ard->setup("/dev/ttyUSB1");
	
	int sensorId = 0;
	float sensorVal;
	for(;;){
			std::cin >> sensorId;

		try{
			std::cout<< "hey" << std::endl;
			ard->getSensor(sensorId,(char*)&sensorVal);
			std::cout<< "hi" << std::endl;
		}catch(...){
			std::cout<< "Read Error! :(" << std::endl;
		}

		std::cout << "Sensor " << sensorId<< " just read " << sensorVal <<std::endl;
	}

}
