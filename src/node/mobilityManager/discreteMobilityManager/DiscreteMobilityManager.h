/****************************************************************************
 *      Author: Balint Uveges                                               *
 *                                                                          *  
 ****************************************************************************/

#ifndef _MOBILITYMODULE_H_
#define _MOBILITYMODULE_H_

#include "node/mobilityManager/MobilityManagerMessage_m.h"
#include "node/mobilityManager/VirtualMobilityManager.h"

using namespace std;

class DiscreteMobilityManager: public VirtualMobilityManager {
 private:
	/*--- The .ned file's parameters ---*/
	double updateInterval;
	double loc1_x;
	double loc1_y;
	double loc1_z;
	double loc2_x;
	double loc2_y;
	double loc2_z;
	double speed;
    double dm_delay;

	/*--- Custom class parameters ---*/
	double incr_x;
	double incr_y;
	double incr_z;
	double distance;
	int direction;
    bool timer_armed=false;
    double j_x;
    double j_y;
    double j_z;

 protected:
	void initialize();
	void handleMessage(cMessage * msg);
};

#endif
