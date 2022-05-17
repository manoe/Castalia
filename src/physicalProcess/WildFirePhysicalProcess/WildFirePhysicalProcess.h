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
#include "physicalProcess/PhysicalEventMessage_m.h"
#include "wf.h"

typedef struct {
	simtime_t time;
	double x;
	double y;
} sourceSnapshot;

struct nodeLocation {
    int id;
    double x_coord;
    double y_coord;
};

class WildFirePhysicalProcess: public CastaliaModule {
 private:
    int wf_start_x_coord;
    int wf_start_y_coord;
    int sim_x_size;
    int sim_y_size;
    bool no_map_file;
    int map_scale;
    int ca_step_period;
    int ca_start_timer;
    const char *map_file;
	const char *description;
    WildFireCA *wf_ca;
    std::vector<nodeLocation> subs;
    WildFireParams wf_params;

 protected:
	virtual void initialize();
	virtual void handleMessage(cMessage * msg);
	virtual void finishSpecific();
	void readIniFileParameters();
    int getInt16(const std::vector<unsigned char> &buffer, int index);
    int getInt8(const std::vector<unsigned char> &buffer, int index);
    float getFloat(const std::vector<unsigned char> &buffer, int index);
    void getMapSize(const std::vector<unsigned char> &buffer, int &map_x_size, int &map_y_size);
    VegetationType mapFnfValue(int value);
    std::vector<unsigned char> readMapFile();
    GridCell** generatePlane(const std::vector<unsigned char> &buffer, int map_x_size, int map_y_size);
    void deletePlane(GridCell **plane, int map_x_size);
    CellPosition getMapCoordinates(double x_sim_coord, double y_sim_coord); 
    double convertStateToSensedValue(CellState state);

    std::vector<int> getDestroyedNodes(std::vector<CellPosition> cells);
    void signalTermination(std::vector<int> nodes);


};

#endif /* _WILDFIREPHYSICALPROCESS_H_ */
