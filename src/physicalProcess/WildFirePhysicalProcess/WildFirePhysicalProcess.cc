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
   
    
    if(no_map_file) {
        if(0!=sim_x_size%map_scale || 0!=sim_y_size%map_scale) {
            throw cRuntimeError("Simulation size is not an integer multiple of cell_size");
        }
        map_x_size=sim_x_size/map_scale;
        map_y_size=sim_y_size/map_scale;
        wf_ca=new WildFireCA(map_x_size, map_y_size,wf_params);
    } else {
        map_buffer=readMapFile();
        getMapSize(map_buffer,map_x_size,map_y_size);
        if(map_x_size*map_scale != sim_x_size || map_y_size*map_scale != sim_y_size) {
            throw cRuntimeError("map_size*map_scale does not match simulation field size\n");
        }
       GridCell **plane=generatePlane(map_buffer, map_x_size, map_y_size);

       if(plane) {
           wf_ca=new WildFireCA(map_x_size, map_y_size,wf_params, plane);
           deletePlane(plane, map_x_size);
       } else {
           throw cRuntimeError("Cannot generate plane\n");
       }
    }
    if(wf_start_x_coord < 0 || wf_start_x_coord+wf_start_x_offset-1 >= map_x_size || wf_start_y_coord < 0 || wf_start_y_coord+wf_start_y_offset-1 >= map_y_size) {
        trace()<<"[info] wf_start_x_coord="<<wf_start_x_coord<<", wf_start_y_coord="<<wf_start_y_coord;
        throw cRuntimeError("WildFire starting coordinate is invalid");
    }


    trace() << "First CA step at: "<<ca_start_timer<<" seconds";
    if(par("enabled")) {
        scheduleAt(SimTime()+static_cast<double>(ca_start_timer), new cMessage("CA step timer expired", TIMER_SERVICE));
        first_step=true;
    }
    if(plane_to_yaml) {
        y_out<<YAML::BeginSeq;
    }
}

double WildFirePhysicalProcess::calculateDistance(CellPosition x, CellPosition y) {
    return sqrt(pow(static_cast<double>(x.x) - static_cast<double>(y.x),2) + pow(static_cast<double>(x.y) - static_cast<double>(y.y),2));
}


double WildFirePhysicalProcess::calculateDistance(nodeRecord x, nodeRecord y) {
    return sqrt(pow(x.x - y.x,2) + pow(x.y - y.y,2));
}



double WildFirePhysicalProcess::calculateSensorValue(CellState** states) {
    trace()<<"[info] calculateSensorValue()";
    double ret_val=0;
    for(int i=0 ; i < sense_distance*2+1 ; ++i) {
        for(int j=0 ; j < sense_distance*2+1 ; ++j) {
            if(states[i][j] == CellState::BURNING) {
                auto dist=calculateDistance(CellPosition(i,j),CellPosition(sense_distance,sense_distance) );
                auto val=pow(1/dist,sense_attn);
                trace()<<"[info] Pos - x,y: "<<i<<", "<<j<<" - distance: "<<dist<<" - value: "<< val;
                ret_val+=val;
            }
        }
    }
    if(states[sense_distance][sense_distance] == CellState::BURNING) {
        ret_val=8;
    }
    trace()<<"[info] Value: "<<ret_val;
    return ret_val;
}


double WildFirePhysicalProcess::calculateDiskModelSensorValue(CellState **states) {
    trace()<<"[info] calculateDiskModelSensorValue()";
    for(int i=0 ; i < sense_distance*2+1 ; ++i) {
        for(int j=0 ; j < sense_distance*2+1 ; ++j) {
            if(states[i][j] == CellState::BURNING && calculateDistance(CellPosition(i,j),CellPosition(sense_distance,sense_distance)) <= sense_distance) {
                return 8;
            }
        }
    }
    return 0;

}

double WildFirePhysicalProcess::calculateDistanceDiskModelSensorValue(CellState **states) {
    trace()<<"[info] calculateDistanceDiskModelSensorValue()";
    double min_dist = sense_distance;
    bool valid = false;
    for(int i=0 ; i < sense_distance*2+1 ; ++i) {
        for(int j=0 ; j < sense_distance*2+1 ; ++j) {
            if(states[i][j] == CellState::BURNING && calculateDistance(CellPosition(i,j),CellPosition(sense_distance,sense_distance)) <= sense_distance) {
                valid = true;
                if(min_dist > calculateDistance(CellPosition(i,j),CellPosition(sense_distance,sense_distance))) {
                    min_dist = calculateDistance(CellPosition(i,j),CellPosition(sense_distance,sense_distance));
                }
            }
        }
    }
    return 8.0/min_dist;
}


double WildFirePhysicalProcess::calculateProbModelSensorValue(CellState **states) {
    trace()<<"[info] calculateProbModelSensorValue()";
    double min_dist = sense_distance;
    bool valid = false;
    for(int i=0 ; i < sense_distance*2+1 ; ++i) {
        for(int j=0 ; j < sense_distance*2+1 ; ++j) {
            if(states[i][j] == CellState::BURNING && calculateDistance(CellPosition(i,j),CellPosition(sense_distance,sense_distance)) <= sense_distance) {
                valid = true;
                if(min_dist > calculateDistance(CellPosition(i,j),CellPosition(sense_distance,sense_distance))) {
                    min_dist = calculateDistance(CellPosition(i,j),CellPosition(sense_distance,sense_distance));
                }
            }
        }
    }
    double prob;
    if(valid) {
        prob = std::exp(std::pow(-lambda*(min_dist-r_u),gamma));
        // ide matek exp - meg minden
    } else {
        prob = 0.0;
    }
    if(getRNG(0)->doubleRand() < prob) {
        return 8.0;
    }
    return 0.0;
}

double WildFirePhysicalProcess::calculateProbSpatialModelSensorValue(CellState **states) {
    trace()<<"[info] calculateProbModelSensorValue()";
    double value = calculateSensorValue(states);
    double prob = std::exp(std::pow(-lambda*(value-8),gamma));
    if(getRNG(0)->doubleRand() < prob) {
        return value;
    }
    return 0.0;
}

void WildFirePhysicalProcess::deleteCellStates(CellState** states) {
    for(int i=0 ; i < sense_distance*2+1 ; ++i) {
        delete states[i];
    }
    delete states;

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
            double value;
            if(spatial_sense) {
                auto states=wf_ca->getStates(pos,sense_distance);
                switch (sensing_model) {
                    case sensingModel::SPATIAL_SENSE: {
                        value=calculateSensorValue(states);
                        break; 
                    }
                    case sensingModel::PROB_SPATIAL_SENSE: {
                        value=calculateProbSpatialModelSensorValue(states);
                        break;
                    }
                    case sensingModel::DISK_MODEL: {
                        value=calculateDiskModelSensorValue(states);
                        break;
                    }
                    case sensingModel::DISTANCE_DISK_MODEL: {
                        calculateDistanceDiskModelSensorValue(states);
                        break;
                    }
                    case sensingModel::PROB_MODEL: {
                        value=calculateProbModelSensorValue(states);
                        break;
                    }
                }
                deleteCellStates(states);
            } else {
                try {
                    state=wf_ca->getState(pos);
                    value=convertStateToSensedValue(state);
                } catch(std::string &e) {
                    trace()<<"Error: "<<e<<" x: "<<pos.x<<" y: "<<pos.y;
                    return;
                }
            }
            phyMsg->setValue(value);
            // Send reply back to the node who made the request
            send(phyMsg, "toNode", phyMsg->getSrcID());
            return;
        }

        case TIMER_SERVICE: {
            trace()<<"CA timer expired";
            if(first_step) {
                trace()<<"[info] First step, do not burn anything.";
                for(auto i=0 ; i < wf_start_x_offset ; ++i) {
                    for(auto j=0 ; j < wf_start_y_offset ; ++j) {
                        wf_ca->addFireSpot({wf_start_x_coord+i,wf_start_y_coord+j});
                    }
                }
                first_step=false;
            } else {
                auto b_cells=wf_ca->stepAndCollect();
                for(auto cell: b_cells) {
                    trace()<<"Burnt cell: "<<cell;
                }
                auto d_nodes=getDestroyedNodes(b_cells);
                if(d_nodes.size()) {
                    signalTermination(d_nodes);
                }
                if(step_limit<step && step_limit>0) {
                    trace()<<"[info] Step limit "<<step_limit<<" exceeded";
                    return;
                }
            }
            ++step;
            scheduleAt(simTime() + static_cast<double>(ca_step_period), msg);
            if(plane_to_yaml) {
                dumpPlane();
            }
            return;
        }

        case PHYSICAL_EVENT: {
            trace()<<"Subscription to physical event";
            PhysicalEventMessage *reg_msg=check_and_cast<PhysicalEventMessage *>(msg);
            if(EventType::REGISTER==reg_msg->getEvent()) {
                trace()<<reg_msg->getSrcID()<<" registered. Position x: "<<reg_msg->getXCoor()<<" y: "<<reg_msg->getYCoor();
                for(auto it = subs.begin() ; it != subs.end() ;) {
                    if(it->id == reg_msg->getSrcID()) {
                        trace()<<"record "<<reg_msg->getSrcID()<<" found, removing";
                        it = subs.erase(it);
                    } else {
                        ++it;
                    }
                }
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
    if(plane_to_yaml) {
        y_out<<YAML::EndSeq;
        ofstream pdr_file("phy_proc.yaml");
        pdr_file<<y_out.c_str();
        pdr_file.close();
    }
    if(plane_at_finish) {
        YAML::Emitter yplane_out;
        dumpPlane(yplane_out);
        ofstream plane_file("plane.yaml");
        plane_file<<yplane_out.c_str();
        plane_file<<std::endl;
        plane_file.close();
    }
    delete wf_ca;
}

void WildFirePhysicalProcess::readIniFileParameters() {
    wf_start_x_coord = par("wf_start_x_coord");
    wf_start_y_coord = par("wf_start_y_coord");
    wf_start_x_offset= par("wf_start_x_offset");
    wf_start_y_offset= par("wf_start_y_offset");

    map_scale        = par("map_scale");
    ca_step_period   = par("ca_step_period");
    ca_start_timer   = par("ca_start_timer");
    description      = par("description");
    no_map_file      = par("no_map_file");
    if(!no_map_file) {
        map_file         = par("map_file");
    }
    wf_params        = { static_cast<float>( (double)par("p_h")),
                         static_cast<float>( (double)par("c_1")),
                         static_cast<float>( (double)par("c_2")),
                         static_cast<float>( (double)par("a")),
                         static_cast<float>( (double)par("w_a")),
                         static_cast<float>( (double)par("w_s")),
                         static_cast<float>( (double)par("l")),
                         par("sp"),
                         par("seed"),
                         static_cast<float>( (double)par("p_mois"))};
    spatial_sense   = par("spatial_sense");
    sense_distance  = par("sense_distance");
    sense_attn      = par("sense_attn");
    plane_to_yaml   = par("plane_to_yaml");
    step_limit      = par("step_limit");
    yp_coding       = par("yp_coding").stringValue();
    rad_res         = par("rad_res");
    sel_all_cell    = par("sel_all_cell");
    look_rad        = par("look_rad");
    sensing_model   = strToSensingModel(par("sensing_model").stringValue());
    r_u             = par("r_u"); 
    lambda          = par("lambda");
    gamma           = par("gamma");
    plane_at_finish = par("plane_at_finish");
}

sensingModel WildFirePhysicalProcess::strToSensingModel(string str) {
    trace()<<"[info] strToSensingModel("<<str<<")";
    if("spatial_sense" == str) {
        trace()<<"[info] SPATIAL_SENSE selected";
        return sensingModel::SPATIAL_SENSE; 
    } else if("disk_model" == str) {
        trace()<<"[info] DISK_MODEL selected";
        return sensingModel::DISK_MODEL;
    } else if("distance_disk_model" == str) {
        trace()<<"[info] DISTANCE_DISK_MODEL selected";
        return sensingModel::DISTANCE_DISK_MODEL; 
    } else if("prob_model" == str) {
        trace()<<"[info] PROB_MODEL selected";
        return sensingModel::PROB_MODEL;
    } else if("prob_spatial_sense" == str) {
        trace()<<"[info] PROB_SPATIAL_SENSE selected";
       return sensingModel::PROB_SPATIAL_SENSE; 
    }
    return sensingModel::UNKNOWN;
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


vector<nodeRecord> WildFirePhysicalProcess::collectCellsInRadius(double radius, double x_sim_coord, double y_sim_coord) {
    double pi = std::acos(-1);
    vector<nodeRecord> v_pos;
    for(double i=0; i < 1 ; i+=rad_res) {
        double x=x_sim_coord+radius*std::cos(2*pi*i);
        double y=y_sim_coord+radius*std::sin(2*pi*i);
        auto state = wf_ca->getState(getMapCoordinates(x, y));
        if(CellState::NOT_IGNITED == state || CellState::NO_FUEL == state) {
            v_pos.push_back({x,y,0,0});
        }
    }
    return collectCellsInsideRadius(look_rad, v_pos);
}


vector<nodeRecord> WildFirePhysicalProcess::collectCellsInsideRadius(double radius, vector<nodeRecord> points) {
    vector<nodeRecord> res;
    for(auto nr: points) {
        int node_count=0;
        int em_node_count=0;
        for(auto x = nr.x-radius ; x < nr.x+radius  ; x+=static_cast<double>(map_scale)) {
            for(auto y = nr.y-radius ; y < nr.y+radius; y+=static_cast<double>(map_scale)) {
                if(radius >= calculateDistance(nr, {x,y,0})) {
                   if(wf_ca->validPosition(getMapCoordinates(x,y))) {
                       for(auto nl: subs) {
                           if(getMapCoordinates(nl.x_coord,nl.y_coord) == getMapCoordinates(x,y)) {
                               node_count++;
                               auto state=wf_ca->getState(getMapCoordinates(nl.x_coord,nl.y_coord));
                               if(state == CellState::BURNING) {
                                   em_node_count++;
                               }
                           }
                       }
                   }
                }
            }
        }
        nodeRecord new_nr=nr;
        new_nr.node=node_count;
        new_nr.em_node=em_node_count;
        if(new_nr.em_node == 0) {
            int rep=1;
            switch (new_nr.node) {
                case 1: {
                    rep+=3;
                    break;
                }
                case 2: {
                    rep+=2;
                    break;
                }
                case 3: {
                    rep+=1;
                    break;
                }
            }
            for(int i=0 ; i < rep ; ++i) {
                res.push_back(new_nr);
            }
        }
        if(new_nr.em_node == 1 && new_nr.node == 2 ||  sel_all_cell) {
            res.push_back(new_nr);
        }
    }
    return res;
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

void WildFirePhysicalProcess::dumpPlane() {
    dumpPlane(y_out);
}

void WildFirePhysicalProcess::dumpPlane(YAML::Emitter &out) {
    auto to_char = [](CellState s) -> std::string {
        switch(s) {
            case CellState::NO_FUEL: {
                return "0";
            }
            case CellState::NOT_IGNITED: {
                return "1";
            }
            case CellState::BURNING: {
                return "2";
            }
            case CellState::BURNED_DOWN: {
                return "3";
            }
            default: {
                return "4";
            }

        }
    };

    auto to_str = [](CellState s) -> std::string {
        switch (s) {
            case CellState::NO_FUEL: {
                return "no_fuel";
            }
            case CellState::NOT_IGNITED: {
                return "not_ignited";
            }
            case CellState::BURNING: {
                return "burning";
            }
            case CellState::BURNED_DOWN: {
                return "burned_down";
            }
            default: {
                return "unknown";
            }
        }
    };

    auto plane=wf_ca->getPlane();
    if(yp_coding == "enum") {
        out<<YAML::BeginMap;
        out<<YAML::Key<<"timestamp";
        out<<YAML::Value<<simTime().dbl();
        out<<YAML::Key<<"plane";
        out<<YAML::BeginSeq;
        for(int x=0; x < wf_ca->getSizeX(); ++x) {
            for(int y=0; y < wf_ca->getSizeY(); ++y) {
                out<<YAML::BeginMap;
                out<<YAML::Key<<"x";
                out<<YAML::Value<<x;
                out<<YAML::Key<<"y";
                out<<YAML::Value<<y;
                out<<YAML::Key<<"state";
                out<<YAML::Value<<to_str(plane[x][y].state);
                out<<YAML::EndMap;
            }
        }
        out<<YAML::EndSeq;
        out<<YAML::EndMap;
    }
    if(yp_coding == "digit") {
        out<<YAML::BeginMap;
        out<<YAML::Key<<"timestamp";
        out<<YAML::Value<<simTime().dbl();
        out<<YAML::Key<<"plane";
        out<<YAML::BeginSeq;
        for(int y=0; y < wf_ca->getSizeY(); ++y) {
            out<<YAML::BeginMap;
            out<<YAML::Key<<"y";
            std::string line;
            for(int x=0; x < wf_ca->getSizeX(); ++x) {
                line+=to_char(plane[x][y].state);
            }
            out<<YAML::Value<<line;
            out<<YAML::EndMap;
        }
        out<<YAML::EndSeq;
        out<<YAML::EndMap;
    }
}

