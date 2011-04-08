#include <iostream>
#include "../src/arduino.h"

using namespace robot;

int main(){
	
	Arduino *ard = new Arduino();
	ard->setup("/dev/ttyUSB0");
	
	int sensorId = 0;
	float* sensorVal = (float*)new char[4];
	for(;;){
			std::cin >> sensorId;

		try{
			ard->getSensor(sensorId,(char*)sensorVal);
		}catch(...){
			std::cout<< "Read Error! :(" << std::endl;
		}

		std::cout << "Sensor " << sensorId<< " just read " << *sensorVal <<std::endl;
	}

}