/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2022                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/

#include "node/communication/routing/efmrp/efmrp.h"

Define_Module(efmrp);

void efmrp::startup() {
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
        setTimer(efmrpTimerDef::SINK_START,par("t_start"));
        setState(efmrpStateDef::WORK);
    } else {
        setHop(std::numeric_limits<int>::max());
        setState(efmrpStateDef::INIT);
    }
    setRound(0);
    fp.ttl              = par("ttl");

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

bool efmrp::isSink() const {
    return g_is_sink;
}

void efmrp::setSinkAddress(const char *p_sink_addr) {
    g_sink_addr.assign(p_sink_addr);
}

std::string efmrp::getSinkAddress() const {
    return g_sink_addr;
}

double efmrp::getTl() {
    double t_l=fp.t_l;
    if(fp.random_t_l) {
        t_l=omnetpp::normal(getRNG(0),fp.t_l,fp.random_t_l_sigma);
    }
    trace()<<"[info] T_l timer's value: "<<t_l;
    return t_l;
}


double efmrp::getTmeas() const {
    return fp.t_meas;
}

double efmrp::getTest() const {
    return fp.t_est;
}

efmrpCostFuncDef efmrp::strToCostFunc(string str) const {
    if("hop" == str) {
        return efmrpCostFuncDef::HOP;
    } else if("hop_and_interf" == str) {
        return efmrpCostFuncDef::HOP_AND_INTERF;
    } else if("hop_emerg_and_interf" == str) {
        return efmrpCostFuncDef::HOP_EMERG_AND_INTERF;
    }
    throw std::invalid_argument("[error] Unkown cost function");
    return efmrpCostFuncDef::NOT_DEFINED; 
}

efmrpRinvTblAdminDef efmrp::strToRinvTblAdmin(string str) const {
    if("erase_on_learn" == str) {
        return efmrpRinvTblAdminDef::ERASE_ON_LEARN;
    } else if("erase_on_round" == str) {
        return efmrpRinvTblAdminDef::ERASE_ON_ROUND;
    } else if(" never_erase" == str) {
        return efmrpRinvTblAdminDef::NEVER_ERASE;
    }
    throw std::invalid_argument("[error] Unkown RINV table admin");
    return efmrpRinvTblAdminDef::UNDEF_ADMIN;
}

void efmrp::setHop(int hop) {
    trace()<<"[info] Update hop "<<g_hop<<" to "<<hop;
    g_hop=hop;
}

int efmrp::getHop() const {
    return g_hop;
}

int efmrp::calculateHop() {
    trace()<<"[info] Entering efmrp::calculateHop()";
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

void efmrp::setRound(int round) {
    trace()<<"[info] Changing round "<<g_round<<" to "<<round;
    g_round=round;
}

int efmrp::getRound() const {
    return g_round;
}

void efmrp::setState(efmrpStateDef state) {
    trace()<<"[info] State change from "<<stateToStr(g_state)<<" to "<<stateToStr(state);
    g_state=state;
}

string efmrp::stateToStr(efmrpStateDef state) const {
    switch (state) {
        case efmrpStateDef::UNDEF: {
            return "UNDEF";
        }
        case efmrpStateDef::WORK: {
            return "WORK";
        }
        case efmrpStateDef::INIT: {
            return "INIT";
        }
        case efmrpStateDef::LEARN: {
            return "LEARN";
        }
        case efmrpStateDef::ESTABLISH: {
            return "ESTABLISH";
        }
        case efmrpStateDef::MEASURE: {
            return "MEASURE";
        }
    }
    return "UNKNOWN";
       
}

efmrpStateDef efmrp::getState() const {
    return g_state;
}

void efmrp::sendPing(int round) {
    trace()<<"[info] Entering efmrp::sendPing(round = "<<round<<")";
    efmrpPingPacket *ping_pkt=new efmrpPingPacket("SHMRP PING packet", NETWORK_LAYER_PACKET);
    ping_pkt->setByteLength(netDataFrameOverhead);
    ping_pkt->setEfmrpPacketKind(efmrpPacketDef::PING_PACKET);
    ping_pkt->setSource(SELF_NETWORK_ADDRESS);
    ping_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    ping_pkt->setRound(round);
    ping_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(ping_pkt, BROADCAST_MAC_ADDRESS);
}

void efmrp::sendPong(int round) {
    trace()<<"[info] Entering efmrp::sendPong(round = "<<round<<")";
    efmrpPongPacket *pong_pkt=new efmrpPongPacket("SHMRP PONG packet", NETWORK_LAYER_PACKET);
    pong_pkt->setByteLength(netDataFrameOverhead);
    pong_pkt->setEfmrpPacketKind(efmrpPacketDef::PONG_PACKET);
    pong_pkt->setSource(SELF_NETWORK_ADDRESS);
    pong_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    pong_pkt->setRound(round);
    pong_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(pong_pkt, BROADCAST_MAC_ADDRESS);

}

void efmrp::storePong(efmrpPongPacket *pong_pkt) {
    trace()<<"[info] Entering efmrp::storePong(pong_pkt.source="<<pong_pkt->getSource()<<")";
    pong_table.insert({pong_pkt->getSource(),{pong_pkt->getSource(),0,0,false,0,0,false,pong_pkt->getRound()}});
}

void efmrp::clearPongTable() {
    pong_table.clear();
}

void efmrp::clearPongTable(int round) {
    trace()<<"[info] Entering efmrp::clearPongTable(round="<<round<<")";
    for(auto it=pong_table.begin() ; it != pong_table.end() ; ++it) {
        if(it->second.round < round ) {
            pong_table.erase(it);
        }
    }
}

int efmrp::getPongTableSize() const {
    return pong_table.size();
}

void efmrp::sendRinv(int round) {
    trace()<<"[info] Entering efmrp::sendRinv(round = "<<round<<")";
    efmrpRinvPacket *rinv_pkt=new efmrpRinvPacket("SHMRP RINV packet", NETWORK_LAYER_PACKET);
    rinv_pkt->setByteLength(netDataFrameOverhead);
    rinv_pkt->setEfmrpPacketKind(efmrpPacketDef::RINV_PACKET);
    rinv_pkt->setSource(SELF_NETWORK_ADDRESS);
    rinv_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    rinv_pkt->setRound(round);
    rinv_pkt->setHop(getHop());
    rinv_pkt->setInterf(getPongTableSize());
    rinv_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(rinv_pkt, BROADCAST_MAC_ADDRESS);
}

void efmrp::clearRinvTable() {
    trace()<<"[info] RINV table erased";
    rinv_table.clear();
}

void efmrp::addToRinvTable(efmrpRinvPacket *rinv_pkt) {
    trace()<<"[info] Add entry to RINV table - source: "<<rinv_pkt->getSource()<<" pathid: "<<rinv_pkt->getPathid()<<" hop: "<<rinv_pkt->getHop()<<" interf: "<<rinv_pkt->getInterf();
    node_entry ne;
    ne.nw_address.assign(rinv_pkt->getSource());
    ne.hop = rinv_pkt->getHop();
    ne.interf = rinv_pkt->getInterf();
    if(rinv_table.find(ne.nw_address) != rinv_table.end()) {
        trace()<<"[info] Entry already exists, overriding";
    }
    rinv_table.insert({ne.nw_address, ne});
}

int efmrp::getRinvTableSize() const {
    return rinv_table.size();
}

double efmrp::calculateCostFunction(node_entry ne) {
    double ret_val;
    switch (fp.cost_func) {
        case efmrpCostFuncDef::HOP: {
            ret_val=static_cast<double>(ne.hop);
            break;
        }
        case efmrpCostFuncDef::HOP_AND_INTERF: {
            ret_val=static_cast<double>(ne.hop) * pow(ne.interf,fp.cost_func_beta);
            break;
        }
        case efmrpCostFuncDef::HOP_EMERG_AND_INTERF: {
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


void efmrp::sendData(cPacket *pkt, std::string dest, int pathid) {
    trace()<<"[info] Sending DATA to "<<dest<<" via pathid "<<pathid;
    efmrpDataPacket *data_pkt=new efmrpDataPacket("SHMRP DATA packet",NETWORK_LAYER_PACKET);
    data_pkt->setByteLength(netDataFrameOverhead);
    data_pkt->setEfmrpPacketKind(efmrpPacketDef::DATA_PACKET);
    data_pkt->setSource(SELF_NETWORK_ADDRESS);
    data_pkt->setOrigin(SELF_NETWORK_ADDRESS);
    data_pkt->setDestination(dest.c_str());
    data_pkt->setPathid(pathid);
    data_pkt->setSequenceNumber(currentSequenceNumber++);
    data_pkt->encapsulate(pkt);
    toMacLayer(data_pkt, resolveNetworkAddress(dest.c_str()));
}

void efmrp::forwardData(efmrpDataPacket *data_pkt, std::string dest) {
    trace()<<"[info] Entering forwardData(dest="<<dest<<")";
    forwardData(data_pkt, dest, data_pkt->getPathid());
}

void efmrp::forwardData(efmrpDataPacket *data_pkt, std::string dest, int pathid) {
    trace()<<"[info] Entering forwardData(dest="<<dest<<", pathid="<<pathid<<")";
    data_pkt->setSource(SELF_NETWORK_ADDRESS);
    data_pkt->setDestination(dest.c_str());
    data_pkt->setPathid(pathid);
    data_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(data_pkt, resolveNetworkAddress(dest.c_str()));
}

void efmrp::clearRoutingTable() {
    trace()<<"[info] Routing table erased";
    routing_table.clear();
}

bool efmrp::isRoutingTableEmpty() const {
    return routing_table.empty();
}

void efmrp::constructRoutingTable(bool rresp_req) {
    trace()<<"[info] Entering efmrp::constructRoutingTable(rresp_req="<<rresp_req<<")";
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

void efmrp::constructRoutingTable(bool rresp_req, bool app_cf) {
    trace()<<"[info] Entering efmrp::constructRoutingTable(rresp_req="<<rresp_req<<", app_cf="<<app_cf<<")";
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

int efmrp::selectPathid() {
    trace()<<"[info] Entering efmrp::selectPathid()";
    if(routing_table.empty()) {
        throw routing_table_empty("[error] Routing table empty");
    }
    auto i=getRNG(0)->intRand(routing_table.size());
    auto it=routing_table.begin();
    std::advance(it,i);
    trace()<<"[info] Selected pathid: "<<it->second.pathid;
    return it->second.pathid;
}

std::string efmrp::getNextHop(int pathid) {
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

std::string efmrp::getNextHop(int pathid, bool random_node) {
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

void efmrp::incPktCountInRoutingTable(std::string next_hop) {
    trace()<<"[info] Entering incPktCountInRoutingTable(next_hop="<<next_hop<<")";
    if(routing_table.find(next_hop) == routing_table.end()) {
        throw no_available_entry("[error] No available entry to update Pkt count"); 
    }

    routing_table[next_hop].pkt_count++;
}

void efmrp::incPktCountInRecvTable(std::string entry) {
    trace()<<"[info] Entering incPktCountInRecvTable("<<entry<<")";
    if(recv_table.find(entry) == recv_table.end()) {
        recv_table[entry] = {entry,0,0,false,0,0,false,0,1};
    } else {
        recv_table[entry].pkt_count++;
    }
}

void efmrp::sendHello() {
    trace()<<"[info] Entering sendHello()";
    sendHello(0, getClock().dbl());
}

void efmrp::sendHello(int hop, double timestamp) {
    trace()<<"[info] Entering sendHello(hop="<<hop<<", timestamp="<<timestamp<<")";
    auto *hello_pkt=new efmrpHelloPacket("EFMRP HELLO packet", NETWORK_LAYER_PACKET);
    hello_pkt->setByteLength(netDataFrameOverhead);
    hello_pkt->setEfmrpPacketKind(efmrpPacketDef::HELLO_PACKET);
    hello_pkt->setOrigin(SELF_NETWORK_ADDRESS);
    hello_pkt->setSource(SELF_NETWORK_ADDRESS);
    hello_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);

    hello_pkt->setHop(hop);
    hello_pkt->setTimestamp(timestamp);

    toMacLayer(hello_pkt, BROADCAST_MAC_ADDRESS);
}

void efmrp::updateHelloTable(efmrpHelloPacket *hello_pkt) {
    trace()<<"[info] Entering updateHelloTable(..)";
    node_entry ne;
    ne.nw_address=hello_pkt->getSource();
    ne.hop=hello_pkt->getHop();
    trace()<<"[info] Adding entry NW address: "<<ne.nw_address<<" hop: "<<ne.hop<<" to hello_table";
    hello_table[hello_pkt->getSource()]=ne;
}


void efmrp::timerFiredCallback(int index) {
    switch (index) {
        case efmrpTimerDef::SINK_START: {
            trace()<<"[timer] SINK_START timer expired";
            sendHello();
            break;
        }
        case efmrpTimerDef::T_ESTABLISH: {
            trace()<<"[timer] T_ESTABLISH timer expired";
            


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
                setState(efmrpStateDef::MEASURE);
                setTimer(efmrpTimerDef::T_MEASURE,getTmeas());
                if(fp.round_keep_pong) {
                    clearPongTable(getRound());
                } else {
                    clearPongTable();
                }
                sendPing(getRound());

            } else {
                trace()<<"[info] Establishment done, transitioning to WORK state";
                setState(efmrpStateDef::WORK);
            }
            break;
        }
        case efmrpTimerDef::T_MEASURE: {
            trace()<<"[timer] T_MEASURE timer expired, PONG table size: "<<getPongTableSize();
            // What if it is in ESTABLISH state? What if new round? how to handle RINV table?
            setState(efmrpStateDef::WORK);
            break;
        }
        default: {
            trace()<<"[error] Unknown timer expired: "<<index;
            break;
        }
    }
}

void efmrp::fromApplicationLayer(cPacket * pkt, const char *destination) {
    if(0!=std::strcmp(destination,getSinkAddress().c_str())) {
        trace()<<"[error] Packet's destination not sink: "<<destination;
        return;
    }

    switch (getState()) {
        case efmrpStateDef::LEARN: {
            trace()<<"[error] In LEARNING state, can't route packet";
            return;
        }
        case efmrpStateDef::MEASURE: {
            trace()<<"[info] In MEASURE state, routing round-robin";
        }

    }


    // Shouldn't we buffer?
    if(isRoutingTableEmpty()) {
        trace()<<"[error] Routing table empty, can't route packet";
        return;
    }

    auto pathid=selectPathid();
    std::string next_hop;
    
    if(efmrpRingDef::EXTERNAL==getRingStatus()) {
        next_hop=getNextHop(pathid);
    } else {
        next_hop=getNextHop(pathid,fp.rand_ring_hop);
    }
    
    incPktCountInRoutingTable(next_hop);
    sendData(pkt,next_hop,pathid);

}

void efmrp::fromMacLayer(cPacket * pkt, int srcMacAddress, double rssi, double lqi) {
    efmrpPacket *efmrp_pkt=dynamic_cast<efmrpPacket *>(pkt);
    if(!efmrp_pkt) {
        trace()<<"[error] Dynamic cast of packet failed";
    }

    trace()<<"[info] EFMRP packet received from MAC: "<<srcMacAddress<<" NW: "<<efmrp_pkt->getSource();

    switch (efmrp_pkt->getEfmrpPacketKind()) {
        case efmrpPacketDef::HELLO_PACKET: {
            trace()<<"[info] HELLO_PACKET received";
            efmrpHelloPacket *hello_pkt=dynamic_cast<efmrpHelloPacket *>(pkt);
            if(getClock().dbl() - hello_pkt->getTimestamp() > fp.ttl) {
                trace()<<"[info] HELLO_PACKET expired, timestamp: "<<hello_pkt->getTimestamp()<<" clock: "<<getClock().dbl();
                break;
            }

            if(hello_pkt->getHop()<getHop()) {
                setHop(hello_pkt->getHop()+1);
            }

            updateHelloTable(hello_pkt);


        }

        case efmrpPacketDef::DATA_PACKET: {
            trace()<<"[info] DATA_PACKET received";
            efmrpDataPacket *data_pkt=dynamic_cast<efmrpDataPacket *>(pkt);
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
                    if(efmrpRingDef::EXTERNAL==getRingStatus()) {
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
        case efmrpPacketDef::UNDEF_PACKET: {
            trace()<<"[error] UNDEF_PACKET received";
            break;
        }
        default: {
            trace()<<"[error] Unknown packet received with efmrpPacketKind value: "<<efmrp_pkt->getEfmrpPacketKind();
            break;
        }
    }
}

map<int,string> efmrp::getPathsAndHops() {
    map<int,string> ret;
    if(0==routing_table.size()) {
        throw std::length_error("Routing table empty");
    }
    for(auto it=routing_table.begin(); it != routing_table.end(); ++it) {
        ret.insert(std::pair<int,string>(it->second.pathid,it->second.nw_address));
    }
    return ret;
}

efmrpRingDef efmrp::getRingStatus() const {
    if(0 == getHop()) {
        return efmrpRingDef::CENTRAL;
    } else if(fp.ring_radius > getHop()) {
        return efmrpRingDef::INTERNAL;
    } else if(fp.ring_radius == getHop()) {
        return efmrpRingDef::BORDER;
    }
    return efmrpRingDef::EXTERNAL;
}

std::string efmrp::ringToStr(efmrpRingDef pos) const {
    switch (pos) {
        case efmrpRingDef::CENTRAL: {
            return string("central");
        }
        case efmrpRingDef::INTERNAL: {
            return string("internal");
        }
        case efmrpRingDef::BORDER: {
            return string("border");
        }
        case efmrpRingDef::EXTERNAL: {
            return string("external");
        }
    }
    return string("unkown");
}

void efmrp::serializeRoutingTable() {
    serializeRoutingTable(routing_table);
}
void efmrp::serializeRoutingTable(std::map<std::string,node_entry> table) {
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

void efmrp::serializeRecvTable() {
    serializeRecvTable(recv_table);
}

void efmrp::serializeRecvTable(std::map<std::string,node_entry> table) {
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


void efmrp::finishSpecific() {
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
            efmrp *efmrp_instance = dynamic_cast<efmrp*>
                (topo->getNode(i)->getModule()->getSubmodule("Communication")->getSubmodule("Routing"));
            auto mob_mgr=dynamic_cast<VirtualMobilityManager *>(topo->getNode(i)->getModule()->getSubmodule("MobilityManager"));
           
             y_out<<YAML::BeginMap;
             y_out<<YAML::Key<<"node";
             y_out<<YAML::Value<<i;
             try {
                auto table=efmrp_instance->getRecvTable();
                 y_out<<YAML::Key<<"recv_table";
                 y_out<<YAML::Value;
                 serializeRecvTable(table);
             } catch (exception &e) {
                 trace()<<"[error] No recv table: "<<e.what();
             }
             try {
                auto table=efmrp_instance->getRoutingTable();
                 y_out<<YAML::Key<<"routing_table";
                 y_out<<YAML::Value;
                 serializeRoutingTable(table);
             } catch (exception &e) {
                 trace()<<"[error] No routing table: "<<e.what();
             }
               y_out<<YAML::Key<<"state";
               y_out<<YAML::Value<<stateToStr(efmrp_instance->getState());

  

            map<int,string> routes;
            try {
                routes=efmrp_instance->getPathsAndHops();
            } catch(std::length_error &e) {
                trace()<<"[error] Can't retrieve paths and hops for node "<<i<<": "<<e.what();
            }

            auto res_mgr=dynamic_cast<ResourceManager *>(topo->getNode(i)->getModule()->getSubmodule("ResourceManager"));
            if(res_mgr->isDead()) {
                r_out<<"'"<<i<<"':'"<<"dead',";
            } else {
                r_out<<"'"<<i<<"':'"<<ringToStr(efmrp_instance->getRingStatus())<<"',";
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
            y_out<<(res_mgr->isDead()?"dead":ringToStr(efmrp_instance->getRingStatus()));
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

void efmrp::handleMacControlMessage(cMessage *msg) {
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
