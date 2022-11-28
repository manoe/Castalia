/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2022                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/

#include "node/communication/routing/shmrp/shmrp.h"

Define_Module(shmrp);

void shmrp::startup() {
    cModule *appModule = getParentModule()->getParentModule()->getSubmodule("Application");
    if(appModule->hasPar("isSink")) {
        g_is_sink=appModule->par("isSink");
    } else {
        g_is_sink=false;
    }

    if(hasPar("sink_address")) {
        g_sink_addr.assign(par("sink_address").stringValue());
    } else {
        trace()<<"[error] No sink address provisioned";
    }

    if(isSink()) {
        setHop(0);
        setTimer(shmrpTimerDef::SINK_START,par("t_start"));
        setState(shmrpStateDef::WORK);
    } else {
        setHop(std::numeric_limits<int>::max());
        setState(shmrpStateDef::INIT);
    }
    setRound(0);
    fp.t_l              = par("t_l");
    fp.ring_radius      = par("ring_radius");
    fp.t_est            = par("t_est");
    fp.t_meas           = par("t_meas");
    fp.rresp_req        = par("f_rresp_required");
    fp.rst_learn        = par("f_restart_learning");
    fp.replay_rinv      = par("f_replay_rinv");
    fp.cost_func        = strToCostFunc(par("f_cost_function").stringValue());
    fp.cost_func_alpha  = par("f_cost_func_alpha");
    fp.cost_func_beta   = par("f_cost_func_beta");
    fp.random_t_l       = par("f_random_t_l");
    fp.random_t_l_sigma = par("f_random_t_l_sigma");
    fp.rinv_tbl_admin   = strToRinvTblAdmin(par("f_rinv_table_admin").stringValue());
    fp.interf_ping      = par("f_interf_ping");
    fp.round_keep_pong  = par("f_round_keep_pong");
    fp.rand_ring_hop    = par("f_rand_ring_hop");
}

bool shmrp::isSink() const {
    return g_is_sink;
}

void shmrp::setSinkAddress(const char *p_sink_addr) {
    g_sink_addr.assign(p_sink_addr);
}

std::string shmrp::getSinkAddress() const {
    return g_sink_addr;
}

double shmrp::getTl() {
    double t_l=fp.t_l;
    if(fp.random_t_l) {
        t_l=omnetpp::normal(getRNG(0),fp.t_l,fp.random_t_l_sigma);
    }
    trace()<<"[info] T_l timer's value: "<<t_l;
    return t_l;
}


double shmrp::getTmeas() const {
    return fp.t_meas;
}

double shmrp::getTest() const {
    return fp.t_est;
}

shmrpCostFuncDef shmrp::strToCostFunc(string str) const {
    if("hop" == str) {
        return shmrpCostFuncDef::HOP;
    } else if("hop_and_interf" == str) {
        return shmrpCostFuncDef::HOP_AND_INTERF;
    } else if("hop_emerg_and_interf" == str) {
        return shmrpCostFuncDef::HOP_EMERG_AND_INTERF;
    }
    throw std::invalid_argument("[error] Unkown cost function");
    return shmrpCostFuncDef::NOT_DEFINED; 
}

shmrpRinvTblAdminDef shmrp::strToRinvTblAdmin(string str) const {
    if("erase_on_learn" == str) {
        return shmrpRinvTblAdminDef::ERASE_ON_LEARN;
    } else if("erase_on_round" == str) {
        return shmrpRinvTblAdminDef::ERASE_ON_ROUND;
    } else if(" never_erase" == str) {
        return shmrpRinvTblAdminDef::NEVER_ERASE;
    }
    throw std::invalid_argument("[error] Unkown RINV table admin");
    return shmrpRinvTblAdminDef::UNDEF_ADMIN;
}

void shmrp::setHop(int hop) {
    trace()<<"[info] Update hop "<<g_hop<<" to "<<hop;
    g_hop=hop;
}

int shmrp::getHop() const {
    return g_hop;
}

int shmrp::calculateHop() {
    trace()<<"[info] Entering shmrp::calculateHop()";
    if(rinv_table.empty()) {
        throw rinv_table_empty("[error] RINV table empty");
    }
    if(std::all_of(rinv_table.begin(), rinv_table.end(),[](std::pair<std::string,node_entry> ne){return ne.second.used; } )) {
        throw no_available_entry("[error] All RINV table entry used");
    }

    int hop=std::numeric_limits<int>::max();
    for(auto ne: rinv_table) {
        trace()<<"[info] Hop level for node "<<ne.second.nw_address<<" is "<<ne.second.hop;
        if(hop > ne.second.hop && !ne.second.used) {
            hop=ne.second.hop;
        }
    }
    return hop+1;
}

void shmrp::setRound(int round) {
    trace()<<"[info] Changing round "<<g_round<<" to "<<round;
    g_round=round;
}

int shmrp::getRound() const {
    return g_round;
}

void shmrp::setState(shmrpStateDef state) {
    trace()<<"[info] State change from "<<stateToStr(g_state)<<" to "<<stateToStr(state);
    g_state=state;
}

string shmrp::stateToStr(shmrpStateDef state) const {
    switch (state) {
        case shmrpStateDef::UNDEF: {
            return "UNDEF";
        }
        case shmrpStateDef::WORK: {
            return "WORK";
        }
        case shmrpStateDef::INIT: {
            return "INIT";
        }
        case shmrpStateDef::LEARN: {
            return "LEARN";
        }
        case shmrpStateDef::ESTABLISH: {
            return "ESTABLISH";
        }
        case shmrpStateDef::MEASURE: {
            return "MEASURE";
        }
    }
    return "UNKNOWN";
       
}

shmrpStateDef shmrp::getState() const {
    return g_state;
}

void shmrp::sendPing(int round) {
    trace()<<"[info] Entering shmrp::sendPing(round = "<<round<<")";
    shmrpPingPacket *ping_pkt=new shmrpPingPacket("SHMRP PING packet", NETWORK_LAYER_PACKET);
    ping_pkt->setByteLength(netDataFrameOverhead);
    ping_pkt->setShmrpPacketKind(shmrpPacketDef::PING_PACKET);
    ping_pkt->setSource(SELF_NETWORK_ADDRESS);
    ping_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    ping_pkt->setRound(round);
    ping_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(ping_pkt, BROADCAST_MAC_ADDRESS);
}

void shmrp::sendPong(int round) {
    trace()<<"[info] Entering shmrp::sendPong(round = "<<round<<")";
    shmrpPongPacket *pong_pkt=new shmrpPongPacket("SHMRP PONG packet", NETWORK_LAYER_PACKET);
    pong_pkt->setByteLength(netDataFrameOverhead);
    pong_pkt->setShmrpPacketKind(shmrpPacketDef::PONG_PACKET);
    pong_pkt->setSource(SELF_NETWORK_ADDRESS);
    pong_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    pong_pkt->setRound(round);
    pong_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(pong_pkt, BROADCAST_MAC_ADDRESS);

}

void shmrp::storePong(shmrpPongPacket *pong_pkt) {
    trace()<<"[info] Entering shmrp::storePong(pong_pkt.source="<<pong_pkt->getSource()<<")";
    pong_table.insert({pong_pkt->getSource(),{pong_pkt->getSource(),0,0,false,0,0,false,pong_pkt->getRound()}});
}

void shmrp::clearPongTable() {
    pong_table.clear();
}

void shmrp::clearPongTable(int round) {
    trace()<<"[info] Entering shmrp::clearPongTable(round="<<round<<")";
    for(auto it=pong_table.begin() ; it != pong_table.end() ; ++it) {
        if(it->second.round < round ) {
            pong_table.erase(it);
        }
    }
}

int shmrp::getPongTableSize() const {
    return pong_table.size();
}

void shmrp::sendRinv(int round, int pathid) {
    trace()<<"[info] Entering shmrp::sendRinv(round = "<<round<<", pathid = "<<pathid<<")";
    shmrpRinvPacket *rinv_pkt=new shmrpRinvPacket("SHMRP RINV packet", NETWORK_LAYER_PACKET);
    rinv_pkt->setByteLength(netDataFrameOverhead);
    rinv_pkt->setShmrpPacketKind(shmrpPacketDef::RINV_PACKET);
    rinv_pkt->setSource(SELF_NETWORK_ADDRESS);
    rinv_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    rinv_pkt->setRound(round);
    rinv_pkt->setPathid(pathid);
    rinv_pkt->setHop(getHop());
    rinv_pkt->setInterf(getPongTableSize());
    rinv_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(rinv_pkt, BROADCAST_MAC_ADDRESS);
}

void shmrp::sendRinv(int round) {
    trace()<<"[info] Entering shmrp::sendRinv(round = "<<round<<")";
    sendRinv(round,0);
}

void shmrp::sendRinvBasedOnHop() {
    trace()<<"[info] Entering sendRinvBasedOnHop()";
    if(getHop() < fp.ring_radius) {
        trace()<<"[info] Node inside mesh ring";
        sendRinv(getRound());
    } else if(getHop() == fp.ring_radius) {
        trace()<<"[info] Node at mesh ring border";
        sendRinv(getRound(), resolveNetworkAddress(SELF_NETWORK_ADDRESS));
    } else {
        trace()<<"[info] Node outside mesh ring";
        int pathid;
        try {
            pathid=selectPathid();
        } catch (std::exception &e) {
            trace()<<e.what();
            throw e;
        }
        sendRinv(getRound(), pathid);
    }
}


void shmrp::clearRinvTable() {
    trace()<<"[info] RINV table erased";
    rinv_table.clear();
}

void shmrp::addToRinvTable(shmrpRinvPacket *rinv_pkt) {
    trace()<<"[info] Add entry to RINV table - source: "<<rinv_pkt->getSource()<<" pathid: "<<rinv_pkt->getPathid()<<" hop: "<<rinv_pkt->getHop()<<" interf: "<<rinv_pkt->getInterf();
    node_entry ne;
    ne.nw_address.assign(rinv_pkt->getSource());
    ne.pathid = rinv_pkt->getPathid();
    ne.hop = rinv_pkt->getHop();
    ne.interf = rinv_pkt->getInterf();
    if(rinv_table.find(ne.nw_address) != rinv_table.end()) {
        trace()<<"[info] Entry already exists, overriding";
    }
    rinv_table.insert({ne.nw_address, ne});
}

int shmrp::getRinvTableSize() const {
    return rinv_table.size();
}

void shmrp::clearRreqTable() {
    trace()<<"[info] RREQ table erased";
    rreq_table.clear();
}

bool shmrp::isRreqTableEmpty() const {
    return rreq_table.empty();
}

double shmrp::calculateCostFunction(node_entry ne) {
    double ret_val;
    switch (fp.cost_func) {
        case shmrpCostFuncDef::HOP: {
            ret_val=static_cast<double>(ne.hop);
            break;
        }
        case shmrpCostFuncDef::HOP_AND_INTERF: {
            ret_val=static_cast<double>(ne.hop) * pow(ne.interf,fp.cost_func_beta);
            break;
        }
        case shmrpCostFuncDef::HOP_EMERG_AND_INTERF: {
            ret_val=static_cast<double>(ne.hop) * pow(ne.emerg,fp.cost_func_alpha) * pow(ne.interf,fp.cost_func_beta);
            break;
        }
        default: {
            trace()<<"[error] Unknown cost function: "<<fp.cost_func;
            throw unknown_cost_function("[error] Unknown cost function");
        }
    }
    trace()<<"[info] Cost function for node entry "<<ne.nw_address<<": "<<ret_val;
    return ret_val;
}


void shmrp::constructRreqTable() {
    trace()<<"[info] Entering shmrp::constructRreqTable()";
    if(rinv_table.empty()) {
        throw rinv_table_empty("[error] RINV table empty");
    }
    if(!rreq_table.empty()) {
        throw rreq_table_non_empty("[error] RREQ table not empty");
    }

    auto hop=calculateHop();
    setHop(hop);

    if(hop <= fp.ring_radius) {
        trace()<<"[info] Node inside mesh ring";
        for(auto ne: rinv_table) {
            if(ne.second.hop < getHop() && !ne.second.used) {
                trace()<<"[info] Adding entry address: "<<ne.second.nw_address<<" hop: "<<ne.second.hop<<" pathid: "<<ne.second.pathid;
                rreq_table.insert(ne);
                rinv_table[ne.second.nw_address].used=true;
            }
        }
    } else {
        trace()<<"[info] Node outside mesh ring";
        std::map<int, std::vector<node_entry>> cl;
        std::for_each(rinv_table.begin(),rinv_table.end(),[&](std::pair<std::string,node_entry> ne){
            if(ne.second.hop < getHop() && !ne.second.used) {
                if(cl.find(ne.second.pathid) == cl.end()) {
                    trace()<<"[info] Adding entry address: "<<ne.second.nw_address<<" hop: "<<ne.second.hop<<" pathid: "<<ne.second.pathid;
                    cl.insert({ne.second.pathid,std::vector<node_entry>{ne.second}});
                } else {
                    trace()<<"[info] Adding entry address: "<<ne.second.nw_address<<" hop: "<<ne.second.hop<<" pathid: "<<ne.second.pathid;
                    cl[ne.second.pathid].push_back(ne.second);
                }
            }
        });

        for(auto l: cl) {
            node_entry c_ne=l.second[0];
            trace()<<"[info] Candidate list entries for pathid "<<c_ne.pathid<<" is " <<l.second.size();
            for(auto ne: l.second) {
                if(calculateCostFunction(ne) < calculateCostFunction(c_ne)) {
                    trace()<<"[info] Node "<<ne.nw_address<<" preferred over node "<<c_ne.nw_address;
                    c_ne=ne;
                }
            }
            trace()<<"[info] Selecting node "<<c_ne.nw_address<<" with pathid "<<c_ne.pathid;
            rreq_table.insert({c_ne.nw_address,c_ne});

            rinv_table[c_ne.nw_address].used=true;
        }
    }
    if(rreq_table.empty()) {
        throw rreq_table_empty("[error] RREQ table empty after constructRreqTable()");
    }


//            for(auto cl: candidate_list) {
//                auto i=getRNG(0)->intRand(cl.second.size());
//                trace()<<"[info] Selecting node "<<cl.second[i].nw_address <<" with pathid "<<cl.second[i].pathid<<" from "<<cl.second.size()<<" nodes";
//                rreq_table.insert({cl.second[i].nw_address,cl.second[i]});
//            }
}

bool shmrp::rreqEntryExists(const char *addr, int pathid) {
    if(rreq_table.find(string(addr)) != rreq_table.end() && rreq_table[string(addr)].pathid == pathid) {
        return true;
    }
    return false;
}

void shmrp::updateRreqTableWithRresp(const char *addr, int pathid) {
    trace()<<"[info] Entering shmrp::updateRreqTableWithRresp(addr="<<addr<<", pathid="<<pathid;
    if(rreq_table.find(string(addr)) != rreq_table.end() && rreq_table[string(addr)].pathid == pathid) {
        rreq_table[string(addr)].rresp=true;
    } else {
        throw std::length_error("[error] Entry not found");
    }
}

bool shmrp::rrespReceived() const {
    if(std::any_of(rreq_table.begin(), rreq_table.end(),[](std::pair<std::string,node_entry> ne){return ne.second.rresp; } )) {
        return true;
    }
    return false;
}


void shmrp::sendRreqs() {
    trace()<<"[info] Entering shmrp::sendRreqs()";
    if(rreq_table.empty()) {
       throw std::length_error("[error] RREQ table empty");
    }
    for(auto ne: rreq_table) {
        shmrpRreqPacket* rreq_pkt=new shmrpRreqPacket("SHMRP RREQ packet",NETWORK_LAYER_PACKET);
        rreq_pkt->setByteLength(netDataFrameOverhead);
        rreq_pkt->setShmrpPacketKind(shmrpPacketDef::RREQ_PACKET);
        rreq_pkt->setSource(SELF_NETWORK_ADDRESS);
        rreq_pkt->setDestination(ne.second.nw_address.c_str());
        rreq_pkt->setRound(getRound());
        rreq_pkt->setPathid(ne.second.pathid);
        rreq_pkt->setSequenceNumber(currentSequenceNumber++);
        trace()<<"[info] Sending RREQ to "<<ne.second.nw_address<<" with pathid: "<<ne.second.pathid;
        toMacLayer(rreq_pkt, resolveNetworkAddress(ne.second.nw_address.c_str()));
    }
}

void shmrp::sendRresp(const char *dest, int round, int pathid) {
    trace()<<"[info] Sending RRESP to "<<dest<<" with round "<<round<<" and pathid "<<pathid;
    shmrpRrespPacket* rresp_pkt=new shmrpRrespPacket("SHMRP RRESP packet",NETWORK_LAYER_PACKET);
    rresp_pkt->setByteLength(netDataFrameOverhead);
    rresp_pkt->setShmrpPacketKind(shmrpPacketDef::RRESP_PACKET);
    rresp_pkt->setSource(SELF_NETWORK_ADDRESS);
    rresp_pkt->setDestination(dest);
    rresp_pkt->setRound(round);
    rresp_pkt->setPathid(pathid);
    rresp_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(rresp_pkt, resolveNetworkAddress(dest));
}

void shmrp::sendData(cPacket *pkt, std::string dest, int pathid) {
    trace()<<"[info] Sending DATA to "<<dest<<" via pathid "<<pathid;
    shmrpDataPacket *data_pkt=new shmrpDataPacket("SHMRP DATA packet",NETWORK_LAYER_PACKET);
    data_pkt->setByteLength(netDataFrameOverhead);
    data_pkt->setShmrpPacketKind(shmrpPacketDef::DATA_PACKET);
    data_pkt->setSource(SELF_NETWORK_ADDRESS);
    data_pkt->setOrigin(SELF_NETWORK_ADDRESS);
    data_pkt->setDestination(dest.c_str());
    data_pkt->setPathid(pathid);
    data_pkt->setSequenceNumber(currentSequenceNumber++);
    data_pkt->encapsulate(pkt);
    toMacLayer(data_pkt, resolveNetworkAddress(dest.c_str()));
}

void shmrp::forwardData(shmrpDataPacket *data_pkt, std::string dest) {
    trace()<<"[info] Entering forwardData(dest="<<dest<<")";
    forwardData(data_pkt, dest, data_pkt->getPathid());
}

void shmrp::forwardData(shmrpDataPacket *data_pkt, std::string dest, int pathid) {
    trace()<<"[info] Entering forwardData(dest="<<dest<<", pathid="<<pathid<<")";
    data_pkt->setSource(SELF_NETWORK_ADDRESS);
    data_pkt->setDestination(dest.c_str());
    data_pkt->setPathid(pathid);
    data_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(data_pkt, resolveNetworkAddress(dest.c_str()));
}

void shmrp::clearRoutingTable() {
    trace()<<"[info] Routing table erased";
    routing_table.clear();
}

bool shmrp::isRoutingTableEmpty() const {
    return routing_table.empty();
}

void shmrp::constructRoutingTable(bool rresp_req) {
    trace()<<"[info] Entering shmrp::constructRoutingTable()";
    for(auto ne: rreq_table) {
        if(ne.second.rresp || !rresp_req) {
            trace()<<"[info] Adding node "<<ne.second.nw_address<<" with pathid "<<ne.second.pathid;
            routing_table.insert(ne);
        }
    }
    if(routing_table.empty()) {
        throw routing_table_empty("[error] Routing table empty");
    }
}

int shmrp::selectPathid() {
    trace()<<"[info] Entering shmrp::selectPathid()";
    if(routing_table.empty()) {
        throw routing_table_empty("[error] Routing table empty");
    }
    auto i=getRNG(0)->intRand(routing_table.size());
    auto it=routing_table.begin();
    std::advance(it,i);
    trace()<<"[info] Selected pathid: "<<it->second.pathid;
    return it->second.pathid;
}

std::string shmrp::getNextHop(int pathid) {
    trace()<<"[info] Entering getNextHop(pathid="<<pathid<<")";
    node_entry next_hop;
    bool found=false;
    std::for_each(routing_table.begin(),routing_table.end(),
        [&](std::pair<std::string,node_entry> ne){
            if(pathid==ne.second.pathid) {
                found=true;
                next_hop=ne.second;
        }
    });
    if(!found) {
        throw no_available_entry("Next hop not available");
    }
    return next_hop.nw_address;
}

std::string shmrp::getNextHop(int pathid, bool random_node) {
    trace()<<"[info] Entering getNextHop(pathid="<<pathid<<", random_node="<<random_node<<")";
    std::string next_hop;
    if(random_node) {
        std::vector<node_entry> nodes;
        for(auto a : routing_table) {
            if(a.second.pathid == pathid) {
                nodes.push_back(a.second);
            }
        }
        if(nodes.empty()) {
            throw no_available_entry("[error] Next hop not available");
        }
        auto i=getRNG(0)->intRand(nodes.size());
        next_hop=nodes[i].nw_address;
        trace()<<"[info] Randomly selected node: "<<next_hop;    
    } else {
        next_hop=getNextHop(pathid);
    }
    return next_hop;
}

void shmrp::incPktCountInRoutingTable(std::string next_hop) {
    trace()<<"[info] Entering incPktCountInRoutingTable(next_hop="<<next_hop<<")";
    if(routing_table.find(next_hop) == routing_table.end()) {
        throw no_available_entry("[error] No available entry to update Pkt count"); 
    }

    routing_table[next_hop].pkt_count++;
}

void shmrp::incPktCountInRecvTable(std::string entry) {
    trace()<<"[info] Entering incPktCountInRecvTable("<<entry<<")";
    if(recv_table.find(entry) == recv_table.end()) {
        recv_table[entry] = {entry,0,0,false,0,0,false,0,1};
    } else {
        recv_table[entry].pkt_count++;
    }
}


void shmrp::timerFiredCallback(int index) {
    switch (index) {
        case shmrpTimerDef::SINK_START: {
            trace()<<"[timer] SINK_START timer expired";
            setRound(1+getRound());
            sendRinv(getRound());
            break;
        }
        case shmrpTimerDef::T_L: {
            trace()<<"[timer] T_L timer expired";
            if(shmrpStateDef::LEARN != getState()) {
                trace()<<"[error] State is not LEARN: "<<stateToStr(getState());
                break;
            }
            setState(shmrpStateDef::ESTABLISH);
            clearRreqTable();


            try {
                constructRreqTable();
            } catch (rinv_table_empty &e) {
                trace()<<e.what();
                trace()<<"[info] Empty RINV table after LEARNING state - most probably due to re-learn";
                setRound(getRound()-1);
                setState(shmrpStateDef::INIT); // could be also work, if routing table is not empty
                break;
            } catch (rreq_table_empty &e) {
                trace()<<e.what();
                trace()<<"[info] Empty RREQ table after LEARNING state - returning to INIT";
                setRound(getRound()-1);
                setState(shmrpStateDef::INIT);
                break;
            } catch (no_available_entry &e) {
                trace()<<e.what();
                trace()<<"[info] No node to connect to - returning to INIT";
                setRound(getRound()-1);
                setState(shmrpStateDef::INIT);
                break;
            } catch (std::exception &e) {
                trace()<<e.what();
                break;
            }
            sendRreqs();
            setTimer(shmrpTimerDef::T_ESTABLISH,getTest());
            break;
        }
        case shmrpTimerDef::T_ESTABLISH: {
            trace()<<"[timer] T_ESTABLISH timer expired";
            
            if(isRreqTableEmpty()) {
                trace()<<"[error] RREQ table empty, impossibru";
                throw rreq_table_empty("[error] RREQ table empty");
            }

            if(!rrespReceived() && fp.rresp_req) {
                trace()<<"[error] No RRESP packet received";
                if(fp.rst_learn) {
                    trace()<<"[info] Returning to learning state, resetting round";
                    setState(shmrpStateDef::LEARN);
                    //setRound(getRound()-1);
                    setTimer(shmrpTimerDef::T_L,getTl());
                    if(shmrpRinvTblAdminDef::ERASE_ON_LEARN==fp.rinv_tbl_admin) {
                        trace()<<"[info] Clearing RINV table";
                        clearRinvTable();
                    }
                    break;
                }
            }

            clearRoutingTable();

            try {
                constructRoutingTable(fp.rresp_req);
            } catch (routing_table_empty &e) {
                trace()<<e.what();
                break;
            }

            if(fp.interf_ping) {
                trace()<<"[info] Performing PING based interference measurement";
                setState(shmrpStateDef::MEASURE);
                setTimer(shmrpTimerDef::T_MEASURE,getTmeas());
                if(fp.round_keep_pong) {
                    clearPongTable(getRound());
                } else {
                    clearPongTable();
                }
                sendPing(getRound());

            } else {
                trace()<<"[info] Establishment done, transitioning to WORK state";
                setState(shmrpStateDef::WORK);
                sendRinvBasedOnHop();
            }
            break;
        }
        case shmrpTimerDef::T_MEASURE: {
            trace()<<"[timer] T_MEASURE timer expired, PONG table size: "<<getPongTableSize();
            // What if it is in ESTABLISH state? What if new round? how to handle RINV table?
            setState(shmrpStateDef::WORK);
            sendRinvBasedOnHop();
            break;
        }
        default: {
            trace()<<"[error] Unknown timer expired: "<<index;
            break;
        }
    }
}

void shmrp::fromApplicationLayer(cPacket * pkt, const char *destination) {
    if(0!=std::strcmp(destination,getSinkAddress().c_str())) {
        trace()<<"[error] Packet's destination not sink: "<<destination;
        return;
    }

    // Shouldn't we buffer?
    if(isRoutingTableEmpty()) {
        trace()<<"[error] Routing table empty, can't route packet";
        return;
    }

    auto pathid=selectPathid();
    std::string next_hop;
    
    if(shmrpRingDef::EXTERNAL==getRingStatus()) {
        next_hop=getNextHop(pathid);
    } else {
        next_hop=getNextHop(pathid,fp.rand_ring_hop);
    }
    
    incPktCountInRoutingTable(next_hop);
    sendData(pkt,next_hop,pathid);

}

void shmrp::fromMacLayer(cPacket * pkt, int srcMacAddress, double rssi, double lqi) {
    shmrpPacket *net_pkt=dynamic_cast<shmrpPacket *>(pkt);
    if(!net_pkt) {
        trace()<<"[error] Dynamic cast of packet failed";
    }

    switch (net_pkt->getShmrpPacketKind()) {
        case shmrpPacketDef::RINV_PACKET: {
            trace()<<"[info] RINV_PACKET received";
            auto rinv_pkt=dynamic_cast<shmrpRinvPacket *>(pkt);

            if(isSink()) {
                trace()<<"[info] RINV_PACKET discarded by Sink";
                break;
            }

            if(rinv_pkt->getRound() > getRound()) {
                setRound(rinv_pkt->getRound());
                switch (getState()) {
                    case shmrpStateDef::LEARN: {
                        cancelTimer(shmrpTimerDef::T_L);
                        break;
                    }
                    case shmrpStateDef::ESTABLISH: {
                        cancelTimer(shmrpTimerDef::T_ESTABLISH);
                        break;
                    }
                    case shmrpStateDef::MEASURE: {
                        cancelTimer(shmrpTimerDef::T_MEASURE);
                        break;
                    }
                }
                if(shmrpRinvTblAdminDef::ERASE_ON_LEARN==fp.rinv_tbl_admin || shmrpRinvTblAdminDef::ERASE_ON_ROUND==fp.rinv_tbl_admin) {
                    clearRinvTable();
                }

                addToRinvTable(rinv_pkt);

                trace()<<"[info] Start LEARNING process"; 
                // What if it is in ESTABLISH state? What if new round? how to handle RINV table?
                setState(shmrpStateDef::LEARN);
                setTimer(shmrpTimerDef::T_L,getTl());

            } else if(rinv_pkt->getRound() == getRound()) {
                if(shmrpStateDef::LEARN != getState() && shmrpRinvTblAdminDef::ERASE_ON_LEARN==fp.rinv_tbl_admin) {
                    trace()<<"[info] RINV packet discarded by node, not in learning state anymore";
                    break;
                }
                addToRinvTable(rinv_pkt);
            } else {
                trace()<<"[info] RINV_PACKET with round "<<rinv_pkt->getRound()<<" discarded by node with round "<<getRound();
            }

            break;
        }
        case shmrpPacketDef::RREQ_PACKET: {
            trace()<<"[info] RREQ_PACKET received";
            auto rreq_pkt=dynamic_cast<shmrpRreqPacket *>(pkt);
            if(rreq_pkt->getRound() != getRound()) {
                trace()<<"[error] RREQ_PACKET's round: "<<rreq_pkt->getRound()<<" does not equal local round: "<<getRound();
                break;
            }
            if(getHop()<fp.ring_radius) {
                sendRresp(rreq_pkt->getSource(),getRound(),rreq_pkt->getPathid());
            } else if(getHop()==fp.ring_radius) {
                if(resolveNetworkAddress(SELF_NETWORK_ADDRESS)!=rreq_pkt->getPathid()) {
                    trace()<<"[error] RREQ packet's pathid "<<rreq_pkt->getPathid()<<" does not match node's network address: "<<SELF_NETWORK_ADDRESS;
                    break;
                }
                sendRresp(rreq_pkt->getSource(),getRound(),rreq_pkt->getPathid());
            } else {
                sendRresp(rreq_pkt->getSource(),getRound(),rreq_pkt->getPathid());
            }
            break;
        }
        case shmrpPacketDef::RRESP_PACKET: {
            trace()<<"[info] RRESP_PACKET received";
            auto rresp_pkt=dynamic_cast<shmrpRrespPacket *>(pkt);
            if(rreqEntryExists(rresp_pkt->getSource(),rresp_pkt->getPathid())) {
                updateRreqTableWithRresp(rresp_pkt->getSource(),rresp_pkt->getPathid());
            } else {
                trace()<<"[error] No entry in RREQ table with address "<<rresp_pkt->getSource()<<" and pathid: "<<rresp_pkt->getPathid();
                break;
            }
            break;
        }
        case shmrpPacketDef::PING_PACKET: {
            trace()<<"[info] PING_PACKET received";
            sendPong(dynamic_cast<shmrpPingPacket *>(pkt)->getRound());
            break;
        }
        case shmrpPacketDef::PONG_PACKET: {
            trace()<<"[info] PONG_PACKET received";
            storePong(dynamic_cast<shmrpPongPacket *>(pkt));
            break;
        }
        case shmrpPacketDef::DATA_PACKET: {
            trace()<<"[info] DATA_PACKET received";
            shmrpDataPacket *data_pkt=dynamic_cast<shmrpDataPacket *>(pkt);
            incPktCountInRecvTable(std::string(data_pkt->getSource()));
            if(isSink() && 0==std::strcmp(data_pkt->getDestination(),SELF_NETWORK_ADDRESS)) {
                trace()<<"[info] DATA packet arrived, forwarding to Application layer";
                data_pkt->setSource(data_pkt->getOrigin());
                toApplicationLayer(decapsulatePacket(data_pkt));
                break;
            } else {
                trace()<<"[info] DATA packet at interim node, routing forward";

                std::string next_hop;
                try {
                    if(shmrpRingDef::EXTERNAL==getRingStatus()) {
                        next_hop=getNextHop(data_pkt->getPathid());
                    } else {
                        next_hop=getNextHop(selectPathid(),fp.rand_ring_hop );
                    }
                } catch (no_available_entry &e) {
                    trace()<<"[error] Next hop not available for pathid: "<<data_pkt->getPathid();
                    break;
                }

                incPktCountInRoutingTable(next_hop);
                forwardData(data_pkt->dup(),next_hop);
            }
            break;
        }
        case shmrpPacketDef::UNDEF_PACKET: {
            trace()<<"[error] UNDEF_PACKET received";
            break;
        }
        default: {
            trace()<<"[error] Unknown packet received with shmrpPacketKind value: "<<net_pkt->getShmrpPacketKind();
            break;
        }
    }
}

map<int,string> shmrp::getPathsAndHops() {
    map<int,string> ret;
    if(0==routing_table.size()) {
        throw std::length_error("Routing table empty");
    }
    for(auto it=routing_table.begin(); it != routing_table.end(); ++it) {
        ret.insert(std::pair<int,string>(it->second.pathid,it->second.nw_address));
    }
    return ret;
}

shmrpRingDef shmrp::getRingStatus() const {
    if(0 == getHop()) {
        return shmrpRingDef::CENTRAL;
    } else if(fp.ring_radius > getHop()) {
        return shmrpRingDef::INTERNAL;
    } else if(fp.ring_radius == getHop()) {
        return shmrpRingDef::BORDER;
    }
    return shmrpRingDef::EXTERNAL;
}

std::string shmrp::ringToStr(shmrpRingDef pos) const {
    switch (pos) {
        case shmrpRingDef::CENTRAL: {
            return string("central");
        }
        case shmrpRingDef::INTERNAL: {
            return string("internal");
        }
        case shmrpRingDef::BORDER: {
            return string("border");
        }
        case shmrpRingDef::EXTERNAL: {
            return string("external");
        }
    }
    return string("unkown");
}

void shmrp::serializeRoutingTable() {
    serializeRoutingTable(routing_table);
}
void shmrp::serializeRoutingTable(std::map<std::string,node_entry> table) {
    y_out<<YAML::BeginSeq;
    for(auto i : table) {
        y_out<<YAML::BeginMap;
        y_out<<YAML::Key<<"node";
        y_out<<YAML::Value<<i.second.nw_address;
        y_out<<YAML::Key<<"pathid";
        y_out<<YAML::Value<<i.second.pathid;
        y_out<<YAML::Key<<"pkt_count";
        y_out<<YAML::Value<<i.second.pkt_count;
        y_out<<YAML::EndMap;
    }
    y_out<<YAML::EndSeq;
}

void shmrp::serializeRecvTable() {
    serializeRecvTable(recv_table);
}

void shmrp::serializeRecvTable(std::map<std::string,node_entry> table) {
    y_out<<YAML::BeginSeq;
    for(auto i : table) {
        y_out<<YAML::BeginMap;
        y_out<<YAML::Key<<"node";
        y_out<<YAML::Value<<i.second.nw_address;
        y_out<<YAML::Key<<"pkt_count";
        y_out<<YAML::Value<<i.second.pkt_count;
        y_out<<YAML::EndMap;
    }
    y_out<<YAML::EndSeq;

}


void shmrp::finishSpecific() {
    if (isSink()) { // && getParentModule()->getIndex() == 0 ) {
        cTopology *topo;        // temp variable to access energy spent by other nodes
        topo = new cTopology("topo");
        topo->extractByNedTypeName(cStringTokenizer("node.Node").asVector());
        
        set<int> paths;
        ofstream g_out("graph.py"), p_out("pos.py"),r_out("role.py");
        g_out<<"edges=[";
        p_out<<"pos={";
        r_out<<"role={";
        auto sink_pos=dynamic_cast<VirtualMobilityManager *>(topo->getNode(0)->getModule()->getSubmodule("MobilityManager"))->getLocation();
        p_out<<"'0':["<<sink_pos.x<<","<<sink_pos.y<<"],";
        r_out<<"'0':'"<<ringToStr(getRingStatus())<<"',";

        y_out<<YAML::BeginSeq;
        y_out<<YAML::BeginMap;
        y_out<<YAML::Key<<"node";
        y_out<<YAML::Value<<"0";
        y_out<<YAML::Key<<"recv_table";
        y_out<<YAML::Value;
        serializeRecvTable();
        y_out<<YAML::Key<<"state";
        y_out<<YAML::Value;
        y_out<<stateToStr(getState());


            auto mob_mgr=dynamic_cast<VirtualMobilityManager *>(topo->getNode(0)->getModule()->getSubmodule("MobilityManager"));

            auto loc=mob_mgr->getLocation();
            y_out<<YAML::Key<<"x";
            y_out<<YAML::Value;
            y_out<<loc.x;
            y_out<<YAML::Key<<"y";
            y_out<<YAML::Value;
            y_out<<loc.y;
            y_out<<YAML::Key<<"role";
            y_out<<YAML::Value;
            y_out<<ringToStr(getRingStatus());

        y_out<<YAML::EndMap;

        for (int i = 1; i < topo->getNumNodes(); ++i) {
            shmrp *shmrp_instance = dynamic_cast<shmrp*>
                (topo->getNode(i)->getModule()->getSubmodule("Communication")->getSubmodule("Routing"));
            auto mob_mgr=dynamic_cast<VirtualMobilityManager *>(topo->getNode(i)->getModule()->getSubmodule("MobilityManager"));
           
             y_out<<YAML::BeginMap;
             y_out<<YAML::Key<<"node";
             y_out<<YAML::Value<<i;
             try {
                auto table=shmrp_instance->getRecvTable();
                 y_out<<YAML::Key<<"recv_table";
                 y_out<<YAML::Value;
                 serializeRecvTable(table);
             } catch (exception &e) {
                 trace()<<"[error] No recv table: "<<e.what();
             }
             try {
                auto table=shmrp_instance->getRoutingTable();
                 y_out<<YAML::Key<<"routing_table";
                 y_out<<YAML::Value;
                 serializeRoutingTable(table);
             } catch (exception &e) {
                 trace()<<"[error] No routing table: "<<e.what();
             }
               y_out<<YAML::Key<<"state";
               y_out<<YAML::Value<<stateToStr(shmrp_instance->getState());

  

            map<int,string> routes;
            try {
                routes=shmrp_instance->getPathsAndHops();
            } catch(std::length_error &e) {
                trace()<<"[error] Can't retrieve paths and hops for node "<<i<<": "<<e.what();
            }

            auto res_mgr=dynamic_cast<ResourceManager *>(topo->getNode(i)->getModule()->getSubmodule("ResourceManager"));
            if(res_mgr->isDead()) {
                r_out<<"'"<<i<<"':'"<<"dead',";
            } else {
                r_out<<"'"<<i<<"':'"<<ringToStr(shmrp_instance->getRingStatus())<<"',";
            }


            for(auto route: routes) {
                // Format: [('Node1', 'Node2',{ 'path': 1}),
                //          ('Node1','Node3', {'path': 2})]
                g_out<<"('"<<i<<"','"<<route.second<<"',{'path':"<<route.first<<"}),";


                
                                
            }

            // Format: {'Node1': [0,0], 'Node2': [0,10],'Node3':[10,0]}
            auto loc=mob_mgr->getLocation();
            p_out<<"'"<<i<<"':["<<loc.x<<","<<loc.y<<"],";
            y_out<<YAML::Key<<"x";
            y_out<<loc.x;
            y_out<<YAML::Key<<"y";
            y_out<<loc.y;
            y_out<<YAML::Key;
            y_out<<"role";
            y_out<<YAML::Value;
            y_out<<(res_mgr->isDead()?"dead":ringToStr(shmrp_instance->getRingStatus()));
            y_out<<YAML::EndMap;

            // seek back one character

        }
        y_out<<YAML::EndSeq;
        ofstream loc_pdr_file("loc_pdr.yaml");
        loc_pdr_file<<y_out.c_str();
        loc_pdr_file.close();
        g_out.seekp(g_out.tellp()-static_cast<long int>(1));
        p_out.seekp(p_out.tellp()-static_cast<long int>(1));
        r_out.seekp(r_out.tellp()-static_cast<long int>(1));

        g_out<<"]"<<std::endl;
        p_out<<"}"<<std::endl;
        r_out<<"}"<<std::endl;

        
        delete(topo);
    }
    return;
}
