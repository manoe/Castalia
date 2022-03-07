/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2022                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/


#ifndef _WILDFIREPHYSICALPROCESS_H_
#define _WILDFIREPHYSICALPROCESS_H_

#define SIMTIME_STEP 0.01

#include "helpStructures/CastaliaModule.h"
#include "physicalProcess/PhysicalProcessMessage_m.h"

using namespace std;

typedef struct {
	simtime_t time;
	double x;
	double y;
} sourceSnapshot;

class WildFirePhysicalProcess: public CastaliaModule {
 private:
	int max_num_cars;
	double car_speed;
	double car_value;
	double car_interarrival;
	double point1_x_coord;
	double point1_y_coord;
	double point2_x_coord;
	double point2_y_coord;

	double road_length;
	sourceSnapshot **sources_snapshots;	// N by M array, where N is numSources and, M is the 
										// maximum number of source snapshots. A source 
										// snapshot is a tuple (time, x, y, value)
	const char *description;

 protected:
	virtual void initialize();
	virtual void handleMessage(cMessage * msg);
	virtual void finishSpecific();
	double calculateScenarioReturnValue(const double &x_coo,
					    const double &y_coo, const simtime_t & stime);
	void readIniFileParameters(void);
};

#endif /* _WILDFIREPHYSICALPROCESS_H_ */
