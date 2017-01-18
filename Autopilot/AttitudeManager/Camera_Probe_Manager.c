#include "Camera_Probe_Manager.h"

// recieves two integers, first is the probe number to drop, second is the pwm signal for the camera

void probeDropTrigger (int* command){
	
	dropProbe(command[0]);
	triggerCamera(command[1]);
	
}
