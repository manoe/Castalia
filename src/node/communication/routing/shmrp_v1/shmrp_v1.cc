/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2022                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/

#include "node/communication/routing/shmrp_v1/shmrp_v1.h"

Define_Module(shmrp_v1);

void shmrp_v1::startup() {
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
        setTimer(shmrp_v1TimerDef::SINK_START,par("t_start"));
        setState(shmrp_v1StateDef::WORK);
    } else {
        setHop(std::numeric_limits<int>::max());
        setState(shmrp_v1StateDef::INIT);
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
    fp.cf_after_rresp   = par("f_cf_after_rresp");
    fp.random_t_l       = par("f_random_t_l");
    fp.random_t_l_sigma = par("f_random_t_l_sigma");
    fp.rinv_tbl_admin   = strToRinvTblAdmin(par("f_rinv_table_admin").stringValue());
    fp.interf_ping      = par("f_interf_ping");
    fp.round_keep_pong  = par("f_round_keep_pong");
    fp.rand_ring_hop    = par("f_rand_ring_hop");
}

bool shmrp_v1::isSink() const {
    return g_is_sink;
}

void shmrp_v1::setSinkAddress(const char *p_sink_addr) {
    g_sink_addr.assign(p_sink_addr);
}

std::string shmrp_v1::getSinkAddress() const {
    return g_sink_addr;
}

double shmrp_v1::getTl() {
    double t_l=fp.t_l;
    if(fp.random_t_l) {
        t_l=omnetpp::normal(getRNG(0),fp.t_l,fp.random_t_l_sigma);
    }
    trace()<<"[info] T_l timer's value: "<<t_l;
    return t_l;
}


double shmrp_v1::getTmeas() const {
    return fp.t_meas;
}

double shmrp_v1::getTest() const {
    return fp.t_est;
}

shmrp_v1CostFuncDef shmrp_v1::strToCostFunc(string str) const {
    if("hop" == str) {
        return shmrp_v1CostFuncDef::HOP;
    } else if("hop_and_interf" == str) {
        return shmrp_v1CostFuncDef::HOP_AND_INTERF;
    } else if("hop_emerg_and_interf" == str) {
        return shmrp_v1CostFuncDef::HOP_EMERG_AND_INTERF;
    }
    throw std::invalid_argument("[error] Unkown cost function");
    return shmrp_v1CostFuncDef::NOT_DEFINED; 
}

shmrp_v1RinvTblAdminDef shmrp_v1::strToRinvTblAdmin(string str) const {
    if("erase_on_learn" == str) {
        return shmrp_v1RinvTblAdminDef::ERASE_ON_LEARN;
    } else if("erase_on_round" == str) {
        return shmrp_v1RinvTblAdminDef::ERASE_ON_ROUND;
    } else if(" never_erase" == str) {
        return shmrp_v1RinvTblAdminDef::NEVER_ERASE;
    }
    throw std::invalid_argument("[error] Unkown RINV table admin");
    return shmrp_v1RinvTblAdminDef::UNDEF_ADMIN;
}

void shmrp_v1::setHop(int hop) {
    trace()<<"[info] Update hop "<<g_hop<<" to "<<hop;
    g_hop=hop;
}

int shmrp_v1::getHop() const {
    return g_hop;
}

int shmrp_v1::calculateHop() {
    trace()<<"[info] Entering shmrp_v1::calculateHop()";
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

void shmrp_v1::setRound(int round) {
    trace()<<"[info] Changing round "<<g_round<<" to "<<round;
    g_round=round;
}

int shmrp_v1::getRound() const {
    return g_round;
}

void shmrp_v1::setState(shmrp_v1StateDef state) {
    trace()<<"[info] State change from "<<stateToStr(g_state)<<" to "<<stateToStr(state);
    g_state=state;
}

string shmrp_v1::stateToStr(shmrp_v1StateDef state) const {
    switch (state) {
        case shmrp_v1StateDef::UNDEF: {
            return "UNDEF";
        }
        case shmrp_v1StateDef::WORK: {
            return "WORK";
        }
        case shmrp_v1StateDef::INIT: {
            return "INIT";
        }
        case shmrp_v1StateDef::LEARN: {
            return "LEARN";
        }
        case shmrp_v1StateDef::ESTABLISH: {
            return "ESTABLISH";
        }
        case shmrp_v1StateDef::MEASURE: {
            return "MEASURE";
        }
    }
    return "UNKNOWN";
       
}

shmrp_v1StateDef shmrp_v1::getState() const {
    return g_state;
}

void shmrp_v1::sendPing(int round) {
    trace()<<"[info] Entering shmrp_v1::sendPing(round = "<<round<<")";
    shmrp_v1PingPacket *ping_pkt=new shmrp_v1PingPacket("SHMRP PING packet", NETWORK_LAYER_PACKET);
    ping_pkt->setByteLength(netDataFrameOverhead);
    ping_pkt->setShmrp_v1PacketKind(shmrp_v1PacketDef::PING_PACKET);
    ping_pkt->setSource(SELF_NETWORK_ADDRESS);
    ping_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    ping_pkt->setRound(round);
    ping_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(ping_pkt, BROADCAST_MAC_ADDRESS);
}

void shmrp_v1::sendPong(int round) {
    trace()<<"[info] Entering shmrp_v1::sendPong(round = "<<round<<")";
    shmrp_v1PongPacket *pong_pkt=new shmrp_v1PongPacket("SHMRP PONG packet", NETWORK_LAYER_PACKET);
    pong_pkt->setByteLength(netDataFrameOverhead);
    pong_pkt->setShmrp_v1PacketKind(shmrp_v1PacketDef::PONG_PACKET);
    pong_pkt->setSource(SELF_NETWORK_ADDRESS);
    pong_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    pong_pkt->setRound(round);
    pong_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(pong_pkt, BROADCAST_MAC_ADDRESS);

}

void shmrp_v1::storePong(shmrp_v1PongPacket *pong_pkt) {
    trace()<<"[info] Entering shmrp_v1::storePong(pong_pkt.source="<<pong_pkt->getSource()<<")";
    pong_table.insert({pong_pkt->getSource(),{pong_pkt->getSource(),0,0,false,0,0,false,pong_pkt->getRound()}});
}

void shmrp_v1::clearPongTable() {
    pong_table.clear();
}

void shmrp_v1::clearPongTable(int round) {
    trace()<<"[info] Entering shmrp_v1::clearPongTable(round="<<round<<")";
    for(auto it=pong_table.begin() ; it != pong_table.end() ; ++it) {
        if(it->second.round < round ) {
            pong_table.erase(it);
        }
    }
}

int shmrp_v1::getPongTableSize() const {
    return pong_table.size();
}

void shmrp_v1::sendRinv(int round, int pathid) {
    trace()<<"[info] Entering shmrp_v1::sendRinv(round = "<<round<<", pathid = "<<pathid<<")";
    shmrp_v1RinvPacket *rinv_pkt=new shmrp_v1RinvPacket("SHMRP RINV packet", NETWORK_LAYER_PACKET);
    rinv_pkt->setByteLength(netDataFrameOverhead);
    rinv_pkt->setShmrp_v1PacketKind(shmrp_v1PacketDef::RINV_PACKET);
    rinv_pkt->setSource(SELF_NETWORK_ADDRESS);
    rinv_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    rinv_pkt->setRound(round);
    rinv_pkt->setPathid(pathid);
    rinv_pkt->setHop(getHop());
    rinv_pkt->setInterf(getPongTableSize());
    rinv_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(rinv_pkt, BROADCAST_MAC_ADDRESS);
}

void shmrp_v1::sendRinv(int round) {
    trace()<<"[info] Entering shmrp_v1::sendRinv(round = "<<round<<")";
    sendRinv(round,0);
}

void shmrp_v1::sendRinvBasedOnHop() {
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


void shmrp_v1::clearRinvTable() {
    trace()<<"[info] RINV table erased";
    rinv_table.clear();
}

void shmrp_v1::addToRinvTable(shmrp_v1RinvPacket *rinv_pkt) {
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

int shmrp_v1::getRinvTableSize() const {
    return rinv_table.size();
}

void shmrp_v1::clearRreqTable() {
    trace()<<"[info] RREQ table erased";
    rreq_table.clear();
}

bool shmrp_v1::isRreqTableEmpty() const {
    return rreq_table.empty();
}

double shmrp_v1::calculateCostFunction(node_entry ne) {
    double ret_val;
    switch (fp.cost_func) {
        case shmrp_v1CostFuncDef::HOP: {
            ret_val=static_cast<double>(ne.hop);
            break;
        }
        case shmrp_v1CostFuncDef::HOP_AND_INTERF: {
            ret_val=static_cast<double>(ne.hop) * pow(ne.interf,fp.cost_func_beta);
            break;
        }
        case shmrp_v1CostFuncDef::HOP_EMERG_AND_INTERF: {
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


void shmrp_v1::constructRreqTable() {
    trace()<<"[info] Entering shmrp_v1::constructRreqTable()";
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
        if(fp.cf_after_rresp) {
            trace()<<"[info] Selecting all RINV nodes to RREQ";
            for(auto l: cl) {
                for(auto n: l.second) {
                    if(n.hop < getHop()) {
                        rreq_table.insert({n.nw_address,n});
//                        rinv_table[n.nw_address].used = true;
                    }
                }
            }
        } else {
           for(auto l: cl) {
               trace()<<"[info] Selecting nodes per pathid to RREQ";
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

bool shmrp_v1::rreqEntryExists(const char *addr, int pathid) {
    if(rreq_table.find(string(addr)) != rreq_table.end() && rreq_table[string(addr)].pathid == pathid) {
        return true;
    }
    return false;
}

void shmrp_v1::updateRreqTableWithRresp(const char *addr, int pathid) {
    trace()<<"[info] Entering shmrp_v1::updateRreqTableWithRresp(addr="<<addr<<", pathid="<<pathid;
    if(rreq_table.find(string(addr)) != rreq_table.end() && rreq_table[string(addr)].pathid == pathid) {
        rreq_table[string(addr)].rresp=true;
    } else {
        throw std::length_error("[error] Entry not found");
    }
}

bool shmrp_v1::rrespReceived() const {
    if(std::any_of(rreq_table.begin(), rreq_table.end(),[](std::pair<std::string,node_entry> ne){return ne.second.rresp; } )) {
        return true;
    }
    return false;
}


void shmrp_v1::sendRreqs() {
    trace()<<"[info] Entering shmrp_v1::sendRreqs()";
    if(rreq_table.empty()) {
       throw std::length_error("[error] RREQ table empty");
    }
    for(auto ne: rreq_table) {
        shmrp_v1RreqPacket* rreq_pkt=new shmrp_v1RreqPacket("SHMRP RREQ packet",NETWORK_LAYER_PACKET);
        rreq_pkt->setByteLength(netDataFrameOverhead);
        rreq_pkt->setShmrp_v1PacketKind(shmrp_v1PacketDef::RREQ_PACKET);
        rreq_pkt->setSource(SELF_NETWORK_ADDRESS);
        rreq_pkt->setDestination(ne.second.nw_address.c_str());
        rreq_pkt->setRound(getRound());
        rreq_pkt->setPathid(ne.second.pathid);
        rreq_pkt->setSequenceNumber(currentSequenceNumber++);
        trace()<<"[info] Sending RREQ to "<<ne.second.nw_address<<" with pathid: "<<ne.second.pathid;
        toMacLayer(rreq_pkt, resolveNetworkAddress(ne.second.nw_address.c_str()));
    }
}

void shmrp_v1::sendRresp(const char *dest, int round, int pathid) {
    trace()<<"[info] Sending RRESP to "<<dest<<" with round "<<round<<" and pathid "<<pathid;
    shmrp_v1RrespPacket* rresp_pkt=new shmrp_v1RrespPacket("SHMRP RRESP packet",NETWORK_LAYER_PACKET);
    rresp_pkt->setByteLength(netDataFrameOverhead);
    rresp_pkt->setShmrp_v1PacketKind(shmrp_v1PacketDef::RRESP_PACKET);
    rresp_pkt->setSource(SELF_NETWORK_ADDRESS);
    rresp_pkt->setDestination(dest);
    rresp_pkt->setRound(round);
    rresp_pkt->setPathid(pathid);
    rresp_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(rresp_pkt, resolveNetworkAddress(dest));
}

void shmrp_v1::sendData(cPacket *pkt, std::string dest, int pathid) {
    trace()<<"[info] Sending DATA to "<<dest<<" via pathid "<<pathid;
    shmrp_v1DataPacket *data_pkt=new shmrp_v1DataPacket("SHMRP DATA packet",NETWORK_LAYER_PACKET);
    data_pkt->setByteLength(netDataFrameOverhead);
    data_pkt->setShmrp_v1PacketKind(shmrp_v1PacketDef::DATA_PACKET);
    data_pkt->setSource(SELF_NETWORK_ADDRESS);
    data_pkt->setOrigin(SELF_NETWORK_ADDRESS);
    data_pkt->setDestination(dest.c_str());
    data_pkt->setPathid(pathid);
    data_pkt->setSequenceNumber(currentSequenceNumber++);
    data_pkt->encapsulate(pkt);
    toMacLayer(data_pkt, resolveNetworkAddress(dest.c_str()));
}

void shmrp_v1::forwardData(shmrp_v1DataPacket *data_pkt, std::string dest) {
    trace()<<"[info] Entering forwardData(dest="<<dest<<")";
    forwardData(data_pkt, dest, data_pkt->getPathid());
}

void shmrp_v1::forwardData(shmrp_v1DataPacket *data_pkt, std::string dest, int pathid) {
    trace()<<"[info] Entering forwardData(dest="<<dest<<", pathid="<<pathid<<")";
    data_pkt->setSource(SELF_NETWORK_ADDRESS);
    data_pkt->setDestination(dest.c_str());
    data_pkt->setPathid(pathid);
    data_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(data_pkt, resolveNetworkAddress(dest.c_str()));
}

void shmrp_v1::clearRoutingTable() {
    trace()<<"[info] Routing table erased";
    routing_table.clear();
}

bool shmrp_v1::isRoutingTableEmpty() const {
    return routing_table.empty();
}

void shmrp_v1::constructRoutingTable(bool rresp_req) {
    trace()<<"[info] Entering shmrp_v1::constructRoutingTable(rresp_req="<<rresp_req<<")";
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

void shmrp_v1::constructRoutingTable(bool rresp_req, bool app_cf) {
    trace()<<"[info] Entering shmrp_v1::constructRoutingTable(rresp_req="<<rresp_req<<", app_cf="<<app_cf<<")";
    if(!app_cf) {
        constructRoutingTable(rresp_req);
        return;
    }

    if(getHop() <= fp.ring_radius) {
        trace()<<"[info] Node inside mesh ring";
        for(auto ne: rreq_table) {
            if(ne.second.hop < getHop() && (ne.second.rresp || !fp.rresp_req )) {
                trace()<<"[info] Adding entry address: "<<ne.second.nw_address<<" hop: "<<ne.second.hop<<" pathid: "<<ne.second.pathid;
                routing_table.insert(ne);
//                rinv_table[ne.second.nw_address].used=true;
            }
        }
        return;
    }

    std::map<int, std::vector<node_entry>> cl;
    std::for_each(rreq_table.begin(),rreq_table.end(),[&](std::pair<std::string,node_entry> ne){
        // Either select only hop-based, if rresp is not required or based on rresp received
        if(ne.second.hop < getHop() && (ne.second.rresp || !fp.rresp_req) ) {
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
        trace()<<"[info] Selecting nodes per pathid to RREQ";
        node_entry c_ne=l.second[0];
        trace()<<"[info] Candidate list entries for pathid "<<c_ne.pathid<<" is " <<l.second.size();
        for(auto ne: l.second) {
            if(calculateCostFunction(ne) < calculateCostFunction(c_ne)) {
                trace()<<"[info] Node "<<ne.nw_address<<" preferred over node "<<c_ne.nw_address;
                c_ne=ne;
            }
        }
        trace()<<"[info] Selecting node "<<c_ne.nw_address<<" with pathid "<<c_ne.pathid;
        routing_table.insert({c_ne.nw_address,c_ne});

        rinv_table[c_ne.nw_address].used=true;
    }
    if(routing_table.empty()) {
        throw rreq_table_empty("[error] routing table empty after constructRreqTable()");
    }
}

int shmrp_v1::selectPathid() {
    trace()<<"[info] Entering shmrp_v1::selectPathid()";
    if(routing_table.empty()) {
        throw routing_table_empty("[error] Routing table empty");
    }
    auto i=getRNG(0)->intRand(routing_table.size());
    auto it=routing_table.begin();
    std::advance(it,i);
    trace()<<"[info] Selected pathid: "<<it->second.pathid;
    return it->second.pathid;
}

std::string shmrp_v1::getNextHop(int pathid) {
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

std::string shmrp_v1::getNextHop(int pathid, bool random_node) {
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

void shmrp_v1::incPktCountInRoutingTable(std::string next_hop) {
    trace()<<"[info] Entering incPktCountInRoutingTable(next_hop="<<next_hop<<")";
    if(routing_table.find(next_hop) == routing_table.end()) {
        throw no_available_entry("[error] No available entry to update Pkt count"); 
    }

    routing_table[next_hop].pkt_count++;
}

void shmrp_v1::incPktCountInRecvTable(std::string entry) {
    trace()<<"[info] Entering incPktCountInRecvTable("<<entry<<")";
    if(recv_table.find(entry) == recv_table.end()) {
        recv_table[entry] = {entry,0,0,false,0,0,false,0,1};
    } else {
        recv_table[entry].pkt_count++;
    }
}

void shmrp_v1::updateRinvTableFromRreqTable() {
    trace()<<"[info] Entering updateRinvTableFromRreqTable()";
    for(auto ne: rreq_table) {
        if(rinv_table.find(ne.first) != rinv_table.end()) {
            trace()<<"[info] Updating node "<<ne.first<<" as used in RINV table";
            rinv_table[ne.first].used = true;
        }
    }
}

void shmrp_v1::timerFiredCallback(int index) {
    switch (index) {
        case shmrp_v1TimerDef::SINK_START: {
            trace()<<"[timer] SINK_START timer expired";
            setRound(1+getRound());
            sendRinv(getRound());
            break;
        }
        case shmrp_v1TimerDef::T_L: {
            trace()<<"[timer] T_L timer expired";
            if(shmrp_v1StateDef::LEARN != getState()) {
                trace()<<"[error] State is not LEARN: "<<stateToStr(getState());
                break;
            }
            setState(shmrp_v1StateDef::ESTABLISH);
            clearRreqTable();


            try {
                constructRreqTable();
            } catch (rinv_table_empty &e) {
                trace()<<e.what();
                trace()<<"[info] Empty RINV table after LEARNING state - most probably due to re-learn";
                setRound(getRound()-1);
                setState(shmrp_v1StateDef::INIT); // could be also work, if routing table is not empty
                break;
            } catch (rreq_table_empty &e) {
                trace()<<e.what();
                trace()<<"[info] Empty RREQ table after LEARNING state - returning to INIT";
                setRound(getRound()-1);
                setState(shmrp_v1StateDef::INIT);
                break;
            } catch (no_available_entry &e) {
                trace()<<e.what();
                trace()<<"[info] No node to connect to - returning to INIT";
                setRound(getRound()-1);
                setState(shmrp_v1StateDef::INIT);
                break;
            } catch (std::exception &e) {
                trace()<<e.what();
                break;
            }
            sendRreqs(); //maybe we could go directly to measure or work in case of hdmrp
            setTimer(shmrp_v1TimerDef::T_ESTABLISH,getTest());
            break;
        }
        case shmrp_v1TimerDef::T_ESTABLISH: {
            trace()<<"[timer] T_ESTABLISH timer expired";
            
            if(isRreqTableEmpty()) {
                trace()<<"[error] RREQ table empty, impossibru";
                throw rreq_table_empty("[error] RREQ table empty");
            }

            if(!rrespReceived() && fp.rresp_req) {
                trace()<<"[error] No RRESP packet received";
                if(fp.rst_learn) {
                    trace()<<"[info] Returning to learning state, resetting round";
                    setState(shmrp_v1StateDef::LEARN);
                    //setRound(getRound()-1);
                    setTimer(shmrp_v1TimerDef::T_L,getTl());
                    if(shmrp_v1RinvTblAdminDef::ERASE_ON_LEARN==fp.rinv_tbl_admin) {
                        trace()<<"[info] Clearing RINV table";
                        clearRinvTable();
                    }
                    if(fp.cf_after_rresp) {
                        trace()<<"[info] Setting used flag in all RINV entries that are present in RREQ";
                        updateRinvTableFromRreqTable();
                    }
                    break;
                }
            }

            clearRoutingTable();

            try {
                if(fp.cf_after_rresp) {
                    constructRoutingTable(fp.rresp_req, fp.cf_after_rresp);
                } else {
                    constructRoutingTable(fp.rresp_req);
                }
            } catch (routing_table_empty &e) {
                trace()<<e.what();
                break;
            }

            if(fp.interf_ping) {
                trace()<<"[info] Performing PING based interference measurement";
                setState(shmrp_v1StateDef::MEASURE);
                setTimer(shmrp_v1TimerDef::T_MEASURE,getTmeas());
                if(fp.round_keep_pong) {
                    clearPongTable(getRound());
                } else {
                    clearPongTable();
                }
                sendPing(getRound());

            } else {
                trace()<<"[info] Establishment done, transitioning to WORK state";
                setState(shmrp_v1StateDef::WORK);
                sendRinvBasedOnHop();
            }
            break;
        }
        case shmrp_v1TimerDef::T_MEASURE: {
            trace()<<"[timer] T_MEASURE timer expired, PONG table size: "<<getPongTableSize();
            // What if it is in ESTABLISH state? What if new round? how to handle RINV table?
            setState(shmrp_v1StateDef::WORK);
            sendRinvBasedOnHop();
            break;
        }
        default: {
            trace()<<"[error] Unknown timer expired: "<<index;
            break;
        }
    }
}

void shmrp_v1::fromApplicationLayer(cPacket * pkt, const char *destination) {
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
    
    if(shmrp_v1RingDef::EXTERNAL==getRingStatus()) {
        next_hop=getNextHop(pathid);
    } else {
        next_hop=getNextHop(pathid,fp.rand_ring_hop);
    }
    
    incPktCountInRoutingTable(next_hop);
    sendData(pkt,next_hop,pathid);

}

void shmrp_v1::fromMacLayer(cPacket * pkt, int srcMacAddress, double rssi, double lqi) {
    shmrp_v1Packet *net_pkt=dynamic_cast<shmrp_v1Packet *>(pkt);
    if(!net_pkt) {
        trace()<<"[error] Dynamic cast of packet failed";
    }

    switch (net_pkt->getShmrp_v1PacketKind()) {
        case shmrp_v1PacketDef::RINV_PACKET: {
            trace()<<"[info] RINV_PACKET received";
            auto rinv_pkt=dynamic_cast<shmrp_v1RinvPacket *>(pkt);

            if(isSink()) {
                trace()<<"[info] RINV_PACKET discarded by Sink";
                break;
            }

            if(rinv_pkt->getRound() > getRound()) {
                setRound(rinv_pkt->getRound());
                switch (getState()) {
                    case shmrp_v1StateDef::LEARN: {
                        cancelTimer(shmrp_v1TimerDef::T_L);
                        break;
                    }
                    case shmrp_v1StateDef::ESTABLISH: {
                        cancelTimer(shmrp_v1TimerDef::T_ESTABLISH);
                        break;
                    }
                    case shmrp_v1StateDef::MEASURE: {
                        cancelTimer(shmrp_v1TimerDef::T_MEASURE);
                        break;
                    }
                }
                if(shmrp_v1RinvTblAdminDef::ERASE_ON_LEARN==fp.rinv_tbl_admin || shmrp_v1RinvTblAdminDef::ERASE_ON_ROUND==fp.rinv_tbl_admin) {
                    clearRinvTable();
                }

                addToRinvTable(rinv_pkt);

                trace()<<"[info] Start LEARNING process"; 
                // What if it is in ESTABLISH state? What if new round? how to handle RINV table?
                setState(shmrp_v1StateDef::LEARN);
                setTimer(shmrp_v1TimerDef::T_L,getTl());

            } else if(rinv_pkt->getRound() == getRound()) {
                if(shmrp_v1StateDef::LEARN != getState() && shmrp_v1RinvTblAdminDef::ERASE_ON_LEARN==fp.rinv_tbl_admin) {
                    trace()<<"[info] RINV packet discarded by node, not in learning state anymore";
                    break;
                }
                addToRinvTable(rinv_pkt);
            } else {
                trace()<<"[info] RINV_PACKET with round "<<rinv_pkt->getRound()<<" discarded by node with round "<<getRound();
            }

            break;
        }
        case shmrp_v1PacketDef::RREQ_PACKET: {
            trace()<<"[info] RREQ_PACKET received";
            auto rreq_pkt=dynamic_cast<shmrp_v1RreqPacket *>(pkt);
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
        case shmrp_v1PacketDef::RRESP_PACKET: {
            trace()<<"[info] RRESP_PACKET received";
            auto rresp_pkt=dynamic_cast<shmrp_v1RrespPacket *>(pkt);
            if(rreqEntryExists(rresp_pkt->getSource(),rresp_pkt->getPathid())) {
                updateRreqTableWithRresp(rresp_pkt->getSource(),rresp_pkt->getPathid());
            } else {
                trace()<<"[error] No entry in RREQ table with address "<<rresp_pkt->getSource()<<" and pathid: "<<rresp_pkt->getPathid();
                break;
            }
            break;
        }
        case shmrp_v1PacketDef::PING_PACKET: {
            trace()<<"[info] PING_PACKET received";
            sendPong(dynamic_cast<shmrp_v1PingPacket *>(pkt)->getRound());
            break;
        }
        case shmrp_v1PacketDef::PONG_PACKET: {
            trace()<<"[info] PONG_PACKET received";
            storePong(dynamic_cast<shmrp_v1PongPacket *>(pkt));
            break;
        }
        case shmrp_v1PacketDef::DATA_PACKET: {
            trace()<<"[info] DATA_PACKET received";
            shmrp_v1DataPacket *data_pkt=dynamic_cast<shmrp_v1DataPacket *>(pkt);
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
                    if(shmrp_v1RingDef::EXTERNAL==getRingStatus()) {
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
        case shmrp_v1PacketDef::UNDEF_PACKET: {
            trace()<<"[error] UNDEF_PACKET received";
            break;
        }
        default: {
            trace()<<"[error] Unknown packet received with shmrp_v1PacketKind value: "<<net_pkt->getShmrp_v1PacketKind();
            break;
        }
    }
}

map<int,string> shmrp_v1::getPathsAndHops() {
    map<int,string> ret;
    if(0==routing_table.size()) {
        throw std::length_error("Routing table empty");
    }
    for(auto it=routing_table.begin(); it != routing_table.end(); ++it) {
        ret.insert(std::pair<int,string>(it->second.pathid,it->second.nw_address));
    }
    return ret;
}

shmrp_v1RingDef shmrp_v1::getRingStatus() const {
    if(0 == getHop()) {
        return shmrp_v1RingDef::CENTRAL;
    } else if(fp.ring_radius > getHop()) {
        return shmrp_v1RingDef::INTERNAL;
    } else if(fp.ring_radius == getHop()) {
        return shmrp_v1RingDef::BORDER;
    }
    return shmrp_v1RingDef::EXTERNAL;
}

std::string shmrp_v1::ringToStr(shmrp_v1RingDef pos) const {
    switch (pos) {
        case shmrp_v1RingDef::CENTRAL: {
            return string("central");
        }
        case shmrp_v1RingDef::INTERNAL: {
            return string("internal");
        }
        case shmrp_v1RingDef::BORDER: {
            return string("border");
        }
        case shmrp_v1RingDef::EXTERNAL: {
            return string("external");
        }
    }
    return string("unkown");
}

void shmrp_v1::serializeRoutingTable() {
    serializeRoutingTable(routing_table);
}
void shmrp_v1::serializeRoutingTable(std::map<std::string,node_entry> table) {
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

void shmrp_v1::serializeRecvTable() {
    serializeRecvTable(recv_table);
}

void shmrp_v1::serializeRecvTable(std::map<std::string,node_entry> table) {
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


void shmrp_v1::finishSpecific() {
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
            shmrp_v1 *shmrp_v1_instance = dynamic_cast<shmrp_v1*>
                (topo->getNode(i)->getModule()->getSubmodule("Communication")->getSubmodule("Routing"));
            auto mob_mgr=dynamic_cast<VirtualMobilityManager *>(topo->getNode(i)->getModule()->getSubmodule("MobilityManager"));
           
             y_out<<YAML::BeginMap;
             y_out<<YAML::Key<<"node";
             y_out<<YAML::Value<<i;
             try {
                auto table=shmrp_v1_instance->getRecvTable();
                 y_out<<YAML::Key<<"recv_table";
                 y_out<<YAML::Value;
                 serializeRecvTable(table);
             } catch (exception &e) {
                 trace()<<"[error] No recv table: "<<e.what();
             }
             try {
                auto table=shmrp_v1_instance->getRoutingTable();
                 y_out<<YAML::Key<<"routing_table";
                 y_out<<YAML::Value;
                 serializeRoutingTable(table);
             } catch (exception &e) {
                 trace()<<"[error] No routing table: "<<e.what();
             }
               y_out<<YAML::Key<<"state";
               y_out<<YAML::Value<<stateToStr(shmrp_v1_instance->getState());

  

            map<int,string> routes;
            try {
                routes=shmrp_v1_instance->getPathsAndHops();
            } catch(std::length_error &e) {
                trace()<<"[error] Can't retrieve paths and hops for node "<<i<<": "<<e.what();
            }

            auto res_mgr=dynamic_cast<ResourceManager *>(topo->getNode(i)->getModule()->getSubmodule("ResourceManager"));
            if(res_mgr->isDead()) {
                r_out<<"'"<<i<<"':'"<<"dead',";
            } else {
                r_out<<"'"<<i<<"':'"<<ringToStr(shmrp_v1_instance->getRingStatus())<<"',";
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
            y_out<<(res_mgr->isDead()?"dead":ringToStr(shmrp_v1_instance->getRingStatus()));
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

void shmrp_v1::handleMacControlMessage(cMessage *msg) {
    trace()<<"[info] Entering handleMacControlMessage()";
    TMacControlMessage *mac_msg=check_and_cast<TMacControlMessage *>(msg);
    if(!mac_msg) {
        trace()<<"[error] Not TMacControlMessage";
    }
    trace()<<"[info] Event: "<<mac_msg->getMacControlMessageKind()<<" Node: "<<mac_msg->getDestination()<<" Seqnum: "<<mac_msg->getSeq_num();
    std::string nw_address = std::to_string(mac_msg->getDestination());
    if(MacControlMessage_type::ACK_RECV == mac_msg->getMacControlMessageKind() && routing_table.find(nw_address) != routing_table.end()) {
        routing_table[nw_address].ack_count++;
    }
            
    delete msg;
}
