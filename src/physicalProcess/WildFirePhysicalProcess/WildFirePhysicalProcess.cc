/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2022                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/


#include "physicalProcess/WildFirePhysicalProcess/WildFirePhysicalProcess.h"

#define K_PARAM 0.1
#define A_PARAM 1

Define_Module(WildFirePhysicalProcess);

void WildFirePhysicalProcess::initialize()
{
	readIniFileParameters();
    cModule *app=getParentModule();
    int sim_x_size=0;
    if(app->hasPar("field_x")) {
        double tmp_sim_x_size=app->par("field_x");
        sim_x_size=static_cast<int>(tmp_sim_x_size);
    }

    int sim_y_size=0;
    if(app->hasPar("field_y")) {
        double tmp_sim_y_size=app->par("field_y");
        sim_y_size=static_cast<int>(tmp_sim_y_size);
    }

    std::vector<unsigned char> map_buffer;
    int map_x_size;
    int map_y_size;

    map_buffer=readMapFile();
    getMapSize(map_buffer,map_x_size,map_y_size);
    if(map_x_size*map_scale != sim_x_size || map_y_size*map_scale != sim_y_size) {
        throw cRuntimeError("map_size*map_scale does not match simulation field size\n");
    }

    GridCell **plane=generatePlane(map_buffer, map_x_size, map_y_size);
    if(plane) {
        wf_ca=new WildFireCA(map_x_size, map_y_size,{ 0.58, 0.045, 0.131, 0.078, 0, 8.1, static_cast<float>(map_scale),true}, plane);
        deletePlane(plane, map_x_size);
    } else {
        throw cRuntimeError("Cannot generate plane\n");
    }
    if(wf_start_x_coord < 0 || wf_start_x_coord >= map_x_size || wf_start_y_coord < 0 || wf_start_y_coord >= map_y_size) {
        throw cRuntimeError("WildFire starting coordinate is invalid");
    }

	//int i, j;
	//sources_snapshots = new sourceSnapshot *[max_num_cars];
	//for (i = 0; i < max_num_cars; i++) {
	//	sources_snapshots[i] = new sourceSnapshot[2];
	//	for (j = 0; j < 2; j++) {
	//		sources_snapshots[i][j].time = -1;
	//	}
	//}
    //
	//double arrival = getRNG(0)->doubleRand() * car_interarrival + car_interarrival / 2;
	//trace() << "First car arrival at " << arrival;
	//scheduleAt(arrival,	new cMessage("New car arrival message", TIMER_SERVICE));
    //
	//declareOutput("Cars generated on the road");
}

void WildFirePhysicalProcess::handleMessage(cMessage * msg)
{
	switch (msg->getKind()) {
		case PHYSICAL_PROCESS_SAMPLING: {
			PhysicalProcessMessage *phyMsg = check_and_cast < PhysicalProcessMessage * >(msg);
			// int nodeIndex = phyMsg->getSrcID();
			// int sensorIndex = phyMsg->getSensorIndex();

			// get the sensed value based on node location
			phyMsg->setValue(calculateScenarioReturnValue(
				phyMsg->getXCoor(), phyMsg->getYCoor(), phyMsg->getSendingTime()));
			// Send reply back to the node who made the request
			send(phyMsg, "toNode", phyMsg->getSrcID());
			return;
		}

		case TIMER_SERVICE: {
		//	int pos = -1;
		//	for (int i = 0; pos == -1 && i < max_num_cars; i++) {
		//		if (sources_snapshots[i][1].time < simTime())
		//			pos = i;
		//	}
        //
		//	if (pos != -1) {
		//		trace() << "New car arrives on the bridge, index " << pos;
		//		if (getRNG(0)->doubleRand() > 0.5) {
		//			sources_snapshots[pos][0].x = point1_x_coord;
		//			sources_snapshots[pos][0].y = point1_y_coord;
		//			sources_snapshots[pos][1].x = point2_x_coord;
		//			sources_snapshots[pos][1].y = point2_y_coord;
		//		} else {
		//			sources_snapshots[pos][0].x = point2_x_coord;
		//			sources_snapshots[pos][0].y = point2_y_coord;
		//			sources_snapshots[pos][1].x = point1_x_coord;
		//			sources_snapshots[pos][1].y = point1_y_coord;
		//		}
		//		sources_snapshots[pos][0].time = simTime();
		//		sources_snapshots[pos][1].time = simTime() + road_length / car_speed;
		//		collectOutput("Cars generated on the road");
		//	}
        //
		//	double arrival = getRNG(0)->doubleRand() * car_interarrival + car_interarrival / 2;
		//	scheduleAt(simTime() + arrival,	msg);
			return;
		}

		default: {
			throw cRuntimeError(":\n Physical Process received message other than PHYSICAL_PROCESS_SAMPLING");
		}
	}
}

void WildFirePhysicalProcess::finishSpecific()
{
    delete wf_ca;
	//int i;
	//for (i = 0; i < max_num_cars; i++) {
	//	delete[]sources_snapshots[i];
	//}
	//delete[]sources_snapshots;
}

void WildFirePhysicalProcess::readIniFileParameters() {
    wf_start_x_coord = par("wf_start_x_coord");
    wf_start_y_coord = par("wf_start_y_coord");
    map_scale        = par("map_scale");
    map_file         = par("map_file");
	description      = par("description");
}

std::vector<unsigned char> WildFirePhysicalProcess::readMapFile() {
    std::ifstream is;
    is.open(map_file, std::ios::binary);
    std::vector<unsigned char> buffer(std::istreambuf_iterator<char>(is), {});
    if( buffer.size() == 0 ) {
        throw cRuntimeError("Cannot open map file\n");
    }
    return buffer;
}

GridCell **WildFirePhysicalProcess::generatePlane(const std::vector<unsigned char> &buffer, int map_x_size, int map_y_size) {
    GridCell **plane=new GridCell*[map_x_size];
    for(int i=0 ; i < map_x_size ; ++i) {
        plane[i]=new GridCell[map_y_size];
    }
    for(int i=0 ; i < map_x_size ; ++i) {
        for(int j=0 ; j < map_y_size ; ++j) {
            int offset=(i*map_y_size+j)*5+4;
            trace()<<"x: "<<i<<" y: "<<j<<" value: "<<getFloat(buffer,offset)<<" value: "<<getInt8(buffer, offset+4);
        }
    }
    return plane;
}

VegetationType WildFirePhysicalProcess::mapFnfValue(int value) {
    switch (value) {
        case 1: {
            return VegetationType::NO_VEGETATION;
        }
        case 2: {
            return VegetationType::PINE;
        }
        case 4: {
            return VegetationType::THICKETS;
        }
        case 8: {
            return VegetationType::NO_VEGETATION;
        }
    }
    throw cRuntimeError("Invalid FNF value");
}

void WildFirePhysicalProcess::deletePlane(GridCell **plane, int map_x_size) { 
    for(int i=0 ; i < map_x_size ; ++i) {
        delete[] plane[i];
    }
    delete[] plane;
}

int WildFirePhysicalProcess::getInt16(const std::vector<unsigned char> &buffer, int index) {
    return static_cast<int>(buffer[index]) * 256 + static_cast<int>(buffer[index+1]);
}

int WildFirePhysicalProcess::getInt8(const std::vector<unsigned char> &buffer, int index) {
    return static_cast<int>(buffer[index]);
}

float WildFirePhysicalProcess::getFloat(const std::vector<unsigned char> &buffer, int index) {
    float ret = 0;
    unsigned char data[4];
    for (int i=0; i<4; ++i) data[i] = buffer[index+i];
    std::memcpy(&ret, data, 4);
    return ret;
}

void WildFirePhysicalProcess::getMapSize(const std::vector<unsigned char> &buffer, int &map_x_size, int &map_y_size) {
   if(buffer.size() < 4) {
      throw cRuntimeError("Insufficient (<4) map file size");
   }
   map_x_size=getInt16(buffer,0);
   trace()<<"map x size: "<<map_x_size;
   map_y_size=getInt16(buffer,2);
   trace()<<"map y size: "<<map_y_size;
}

double WildFirePhysicalProcess::calculateScenarioReturnValue(const double &x_coo,
							 const double &y_coo, const simtime_t &stime)
{
	double retVal = 0.0f;
	//int i;
	//double linear_coeff, distance, x, y;
    //
	//for (i = 0; i < max_num_cars; i++) {
	//	if (sources_snapshots[i][1].time >= stime) {
	//		linear_coeff = (stime - sources_snapshots[i][0].time) /
	//		    (sources_snapshots[i][1].time - sources_snapshots[i][0].time);
	//		x = sources_snapshots[i][0].x + linear_coeff * 
	//			(sources_snapshots[i][1].x - sources_snapshots[i][0].x);
	//		y = sources_snapshots[i][0].y + linear_coeff * 
	//			(sources_snapshots[i][1].y - sources_snapshots[i][0].y);
	//		distance = sqrt((x_coo - x) * (x_coo - x) +
	//			 (y_coo - y) * (y_coo - y));
	//		retVal += pow(K_PARAM * distance + 1, -A_PARAM) * car_value;
	//	}
	//}
	return retVal;
}

