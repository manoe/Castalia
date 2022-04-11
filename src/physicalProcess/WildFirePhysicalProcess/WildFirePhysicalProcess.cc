/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2022                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/


#include "physicalProcess/WildFirePhysicalProcess/WildFirePhysicalProcess.h"


Define_Module(WildFirePhysicalProcess);

void WildFirePhysicalProcess::initialize()
{
    readIniFileParameters();
    cModule *app=getParentModule();
    sim_x_size=0;
    if(app->hasPar("field_x")) {
        double tmp_sim_x_size=app->par("field_x");
        sim_x_size=static_cast<int>(tmp_sim_x_size);
    }

    sim_y_size=0;
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

    wf_ca->addFireSpot({wf_start_x_coord,wf_start_y_coord});

    trace() << "Firt CA step at: "<<ca_start_timer<<" seconds";
    scheduleAt(SimTime()+static_cast<double>(ca_start_timer), new cMessage("CA step timer expired", TIMER_SERVICE));
    //declareOutput("Cars GENErated on the road");
}

void WildFirePhysicalProcess::handleMessage(cMessage * msg)
{
    switch (msg->getKind()) {
        case PHYSICAL_PROCESS_SAMPLING: {
            PhysicalProcessMessage *phyMsg = check_and_cast < PhysicalProcessMessage * >(msg);
            CellPosition pos=getMapCoordinates(phyMsg->getXCoor(), phyMsg->getYCoor());

//            std::cout<<pos<<std::endl;
//
            CellState state;
            try {
                state=wf_ca->getState(pos);
            } catch(std::string &e) {
                trace()<<"Error: "<<e<<" x: "<<pos.x<<" y: "<<pos.y;
                return;
            }
            phyMsg->setValue(convertStateToSensedValue(state));
            // Send reply back to the node who made the request
            send(phyMsg, "toNode", phyMsg->getSrcID());
            return;
        }

        case TIMER_SERVICE: {
            trace()<<"CA timer expired";
            auto b_cells=wf_ca->stepAndCollect();
            for(auto cell: b_cells) {
                trace()<<"Burnt cell: "<<cell;
            }
            auto d_nodes=getDestroyedNodes(b_cells);
            if(d_nodes.size()) {
                signalTermination(d_nodes);
            }
            scheduleAt(simTime() + static_cast<double>(ca_step_period), msg);
            return;
        }

        case PHYSICAL_EVENT: {
            trace()<<"Subscription to physical event";
            PhysicalEventMessage *reg_msg=check_and_cast<PhysicalEventMessage *>(msg);
            if(EventType::REGISTER==reg_msg->getEvent()) {
                trace()<<reg_msg->getSrcID()<<" registered. Position x: "<<reg_msg->getXCoor()<<" y: "<<reg_msg->getYCoor();
                subs.push_back({reg_msg->getSrcID(), reg_msg->getXCoor(), reg_msg->getYCoor()});
            }
            delete msg;
            return;
        }

        default: {
            throw cRuntimeError(":\n Physical Process received message other than PHYSICAL_PROCESS_SAMPLING or PHYSICAL_EVENT");
        }
    }
}

void WildFirePhysicalProcess::finishSpecific()
{
    delete wf_ca;
}

void WildFirePhysicalProcess::readIniFileParameters() {
    wf_start_x_coord = par("wf_start_x_coord");
    wf_start_y_coord = par("wf_start_y_coord");
    map_scale        = par("map_scale");
    ca_step_period   = par("ca_step_period");
    ca_start_timer   = par("ca_start_timer");
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

CellPosition WildFirePhysicalProcess::getMapCoordinates(double x_sim_coord, double y_sim_coord) {
    double x_d_pos=x_sim_coord/static_cast<double>(map_scale);
    if(static_cast<int>(x_d_pos)*map_scale >= static_cast<double>(sim_x_size)) {
        x_d_pos-=1;
    }

    double y_d_pos=y_sim_coord/static_cast<double>(map_scale);
    if(static_cast<int>(y_d_pos)*map_scale >= static_cast<double>(sim_y_size)) {
        y_d_pos-=1;
    }

    return CellPosition(static_cast<int>(x_d_pos),static_cast<int>(y_d_pos));
}

double WildFirePhysicalProcess::convertStateToSensedValue(CellState state) {
    if(CellState::BURNING==state) {
        return 1.0;
    } else {
        return 0.0;
    }
}

std::vector<int> WildFirePhysicalProcess::getDestroyedNodes(std::vector<CellPosition> cells) {
    std::vector<int> b_nodes;
    for(auto node: subs) {
        for(auto cell: cells) {
            if(getMapCoordinates(node.x_coord, node.y_coord)==cell) {
                trace()<<node.id<<" terminated, position: "<<cell<<" real position: "<<node.x_coord<<" "<<node.y_coord;
                b_nodes.push_back(node.id);
            }
        }
    }
    return b_nodes;
}

void WildFirePhysicalProcess::signalTermination(std::vector<int> nodes) {
    if(!nodes.size()) {
        return;
    }
    
    for(auto node: nodes) {
        PhysicalEventMessage *msg=new PhysicalEventMessage("destroyed node", PHYSICAL_EVENT);
        msg->setSrcID(node);
        msg->setEvent(EventType::TERMINATE);
        send(msg, "toNode", msg->getSrcID());
    }
}

