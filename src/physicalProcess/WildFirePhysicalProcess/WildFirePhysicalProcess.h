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
#include <yaml-cpp/yaml.h>
#include <cmath>

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

struct nodeRecord {
    double x;
    double y;
    int node;
    int em_node;
};

class WildFirePhysicalProcess: public CastaliaModule {
 private:
    int wf_start_x_coord;
    int wf_start_y_coord;
    int wf_start_x_offset;
    int wf_start_y_offset;
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
    bool first_step;
    bool spatial_sense;
    int sense_distance;
    double sense_attn;
    bool plane_to_yaml;
    int step_limit;
    int step=0;
    std::string yp_coding;
    YAML::Emitter y_out;
    double rad_res;
    bool sel_all_cell;
    double look_rad;

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

    double calculateSensorValue(CellState** states);
    void deleteCellStates(CellState** states);
    double calculateDistance(CellPosition x, CellPosition y);
    double calculateDistance(nodeRecord x, nodeRecord y);
    void dumpPlane();
     vector<nodeRecord> collectCellsInsideRadius(double radius, vector<nodeRecord> points);
 public:
     vector<nodeRecord> collectCellsInRadius(double radius, double x_sim_coord, double y_sim_coord);
};

#endif /* _WILDFIREPHYSICALPROCESS_H_ */
