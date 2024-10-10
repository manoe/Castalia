/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2023                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/

#include "node/communication/routing/smrp/smrp.h"

Define_Module(smrp);

void smrp::startup() {
    trace()<<"[info] Startup initialized.";
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
        setHop(resolveNetworkAddress(SELF_NETWORK_ADDRESS),0);
        setTimer(smrpTimerDef::SINK_START,par("t_start"));
        setState(smrpStateDef::INIT);
    } else {
        setState(smrpStateDef::INIT);
    }

    fp.ttl              = par("t_ttl");
    fp.field            = par("t_field");
    fp.query            = par("t_query");
    fp.env_c            = par("t_env_c");
    fp.d_update         = par("t_d_update");
    fp.restart          = par("t_restart");
    fp.alpha            = par("p_alpha");
    fp.beta             = par("p_beta");
    fp.pnum             = par("p_pnum");
    fp.gamma            = par("p_gamma");
    fp.n_lim            = par("p_n_lim");
    fp.periodic_restart = par("p_periodic_restart");
    fp.a_paths          = par("p_a_paths");
    fp.c_dead           = par("p_c_dead");

    ff_app = dynamic_cast<ForestFire *>(appModule);

    env_val=1.0;
    forw_pkt_count = 0;
}

bool smrp::isSink() const {
    return g_is_sink;
}

void smrp::setSinkAddress(const char *p_sink_addr) {
    g_sink_addr.assign(p_sink_addr);
}

std::string smrp::getSinkAddress() const {
    return g_sink_addr;
}

void smrp::setHop(int sink, int hop) {
    if(g_hop.find(sink) != g_hop.end()) {
        trace()<<"[info] Update hop "<<g_hop[sink]<<" to "<<hop<<" for sink "<<sink;
    } else {
        trace()<<"[info] Initialising hop to "<<hop<<" for sink "<<sink;
    }
    g_hop[sink]=hop;
}

std::map<int, int> smrp::getHop() {
    return g_hop;
}

int smrp::getHop(int sink) {
    if(g_hop.find(sink) != g_hop.end()) {
        return g_hop[sink];
    }
    throw std::runtime_error("Sink-specific entry not present.");
}

bool smrp::updateHop(int sink,int hop) {
    if(g_hop.find(sink) != g_hop.end()) {
        return g_hop[sink] > hop+1;
    }
    return true;
}


void smrp::setState(smrpStateDef state) {
    trace()<<"[info] State change from "<<stateToStr(g_state)<<" to "<<stateToStr(state);
    cTopology *topo = new cTopology("topo");
    topo->extractByNedTypeName(cStringTokenizer("node.Node").asVector());
    auto *smrp_instance = dynamic_cast<smrp*>
                (topo->getNode(atoi(getSinkAddress().c_str()))->getModule()->getSubmodule("Communication")->getSubmodule("Routing"));
    auto *res_mgr = dynamic_cast<ResourceManager *>(getParentModule()->getParentModule()->getSubmodule("ResourceManager"));
    smrp_instance->writeState(atoi(SELF_NETWORK_ADDRESS), simTime().dbl(), state, res_mgr->getSpentEnergy());
    delete topo;
    g_state=state;
}

void smrp::writeState(int node, double timestamp, smrpStateDef state, double energy) {
    int numNodes = getParentModule()->getParentModule()->getParentModule()->par("numNodes");
    cTopology *topo;
    topo = new cTopology("topo");
    topo->extractByNedTypeName(cStringTokenizer("node.Node").asVector());
    double nrg=0.0;
    for (int i = 0; i < numNodes; i++) {
        auto *rm = dynamic_cast<ResourceManager*>
                        (topo->getNode(i)->getModule()->getSubmodule("ResourceManager"));
        nrg+=rm->getSpentEnergy();
    }
    delete topo;
    state_chng_log.push_back({node,timestamp,state,energy,nrg});
}


string smrp::stateToStr(smrpStateDef state) const {
    switch (state) {
        case smrpStateDef::UNDEF: {
            return "UNDEF";
        }
        case smrpStateDef::WORK: {
            return "WORK";
        }
        case smrpStateDef::BUILD: {
            return "BUILD";
        }
        case smrpStateDef::LEARN: {
            return "LEARN";
        }
        case smrpStateDef::INIT: {
            return "INIT";
        }
    }
    return "UNKNOWN";
}

string smrp::pathStatusToStr(smrpPathStatus pstatus) const {
    switch (pstatus) {
       case smrpPathStatus::UNKNOWN: {
           return "UNKNOWN";
       }
       case smrpPathStatus::AVAILABLE: {
           return "AVAILABLE";
       }
       case smrpPathStatus::USED: {
           return "USED";
       }
       case smrpPathStatus::DEAD: {
           return "DEAD";
       }
       case smrpPathStatus::UNDER_QUERY: {
           return "UNDER_QUERY";
       }
    }
    return "NONDEF";
} 

smrpStateDef smrp::getState() const {
    return g_state;
}

void smrp::sendHello() {
    trace()<<"[info] Entering sendHello()";
    sendHello(resolveNetworkAddress(SELF_NETWORK_ADDRESS),0,1,1,getClock().dbl());
}

void smrp::sendHello(int sink, int hop, double env, double nrg, double timestamp) {
    trace()<<"[info] Entering sendHello(hop="<<hop<<", env="<<env<<", nrg="<<nrg<<", timestamp="<<timestamp<<")";
    auto *hello_pkt=new smrpHelloPacket("SMRP HELLO packet", NETWORK_LAYER_PACKET);
    hello_pkt->setByteLength(netDataFrameOverhead);
    hello_pkt->setSmrpPacketKind(smrpPacketDef::HELLO_PACKET);
    hello_pkt->setOrigin(SELF_NETWORK_ADDRESS);
    hello_pkt->setSource(SELF_NETWORK_ADDRESS);
    hello_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    hello_pkt->setSink(sink);
    hello_pkt->setHop(hop);
    hello_pkt->setEnv(env);
    hello_pkt->setNrg(nrg);
    hello_pkt->setTimestamp(timestamp);

    toMacLayer(hello_pkt, BROADCAST_MAC_ADDRESS);
}

void smrp::sendField(std::map<int, int> hop, double nrg, double env, double trg) {
    // hop logging missing
    trace()<<"[info] Entering sendField(nrg="<<nrg<<", env="<<env<<", trg="<<trg<<")";
    auto *field_pkt=new smrpFieldPacket("SMRP FIELD packet",NETWORK_LAYER_PACKET);
    field_pkt->setByteLength(netDataFrameOverhead);
    field_pkt->setSmrpPacketKind(smrpPacketDef::FIELD_PACKET);
    field_pkt->setOrigin(SELF_NETWORK_ADDRESS);
    field_pkt->setSource(SELF_NETWORK_ADDRESS);
    field_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);

    field_pkt->setHopArraySize(hop.size());
    auto it=hop.begin();
    for(int i=0 ; i < hop.size() ; ++i) {
        smrpHop he;
        he.sink = it->first;
        he.hop  = it->second;
        field_pkt->setHop(i,he);
        ++it;
    }

    field_pkt->setNrg(nrg);
    field_pkt->setEnv(env);
    field_pkt->setTrg(trg);

    toMacLayer(field_pkt, BROADCAST_MAC_ADDRESS);
}

void smrp::updateHelloTable(smrpHelloPacket *hello_pkt) {
    trace()<<"[info] Entering updateHelloTable(..)";
    sm_node_entry ne;
    ne.nw_address=hello_pkt->getSource();
    ne.env=hello_pkt->getEnv();
    ne.nrg=hello_pkt->getNrg();
    ne.hop[hello_pkt->getSink()]=hello_pkt->getHop();
    trace()<<"[info] Adding entry NW address: "<<ne.nw_address<<" sink: "<<hello_pkt->getSink()<<" hop: "<<hello_pkt->getHop()<<" env: "<<ne.env<<" nrg: "<<ne.nrg<<" to hello_table";
    if(hello_table.find(ne.nw_address) != hello_table.end()) {
            trace()<<"[warn] Overriding record";
            ne.hop.insert(hello_table[ne.nw_address].hop.begin(),hello_table[ne.nw_address].hop.end() );
    }
    hello_table[hello_pkt->getSource()]=ne;
}

void smrp::updateHelloTable(smrpFieldPacket *field_pkt) {
    trace()<<"[info] Entering updateHelloTable(..)";
    sm_node_entry ne;
    ne.nw_address=field_pkt->getSource();
    ne.env=field_pkt->getEnv();
    ne.nrg=field_pkt->getNrg();
    for(unsigned int i=0 ; i < field_pkt->getHopArraySize() ; ++i) {
        auto he = field_pkt->getHop(i);
        if(he.hop > getHop(he.sink)) {
            trace()<<"[error] Hop count "<<he.hop<<" for sink "<<he.sink<<" is greater than self hop: "<<getHop(he.sink);
        } else {
            trace()<<"[info] Add hop: "<<he.hop<<" associeted with sink: "<<he.sink;
            ne.hop[he.sink]=he.hop;
        }
    }
    if(ne.hop.empty()) {
        throw std::runtime_error("[error] No hop information added");
    }
    trace()<<"[info] Adding entry NW address: "<<ne.nw_address<<" env: "<<ne.env<<" nrg: "<<ne.nrg<<" to hello_table";
    if(hello_table.find(ne.nw_address) != hello_table.end()) {
            trace()<<"[warn] Overriding record";
    }
    hello_table[ne.nw_address]=ne;
}

bool smrp::checkHelloTable(std::string nw_address) {
    trace()<<"[info] Entering checkHelloTable(nw_address="<<nw_address<<")";
    if(hello_table.find(nw_address) != hello_table.end()) {
        return true;
    }
    return false;
}

void smrp::removeHelloEntry(std::string nw_address) {
    trace()<<"[info] Entering removeHelloEntry(nw_address="<<nw_address<<")";
    if(hello_table.find(nw_address) != hello_table.end()) {
        hello_table.erase(nw_address);
    }
    throw std::runtime_error("No hello table entry present");
}


void smrp::removeSinkFromHelloTable(int sink) {
}


void smrp::initHelloTable() {
    trace()<<"[info] Entering initHelloTable()";
    hello_table.clear();
}


void smrp::updateFieldTable(smrpFieldPacket *field_pkt) {
    trace()<<"[info] Entering updateFieldTable(..)";
    sm_node_entry ne;
    ne.nw_address = field_pkt->getSource();
    for(unsigned int i=0 ; i < field_pkt->getHopArraySize() ; ++i) {
        auto he = field_pkt->getHop(i);
        ne.hop[he.sink]=he.hop;
    }
    ne.nrg        = field_pkt->getNrg();
    ne.env        = field_pkt->getEnv();
    ne.trg        = field_pkt->getTrg();

    if(field_table.find(ne.nw_address) != field_table.end()) {
        trace()<<"[info] Overriding existing record of node "<<ne.nw_address;
        field_table[ne.nw_address] = ne;
    } else {
        trace()<<"[info] New record of node "<<ne.nw_address;
        field_table.insert({ne.nw_address,ne});
    }
    // hop logging missing
    trace()<<"[info] nrg: "<<ne.nrg<<" env: "<<ne.env<<" trg: "<<ne.trg;
}

void smrp::updateFieldTableEntry(std::string ne, double env, double nrg, double trg) {
    trace()<<"[info] Entering updateFieldTableEntry(ne="<<ne<<", env="<<env<<", nrg="<<nrg<<", trg="<<trg<<")";
    if(field_table.find(ne) == field_table.end()) {
        throw std::runtime_error("[error] Entry not present");
    }
    field_table[ne].env=env;
    field_table[ne].nrg=nrg;
    field_table[ne].trg=trg;
}

bool smrp::checkFieldEntry(std::string ne) {
    trace()<<"[info] checkFieldEntry(ne="<<ne<<")";
    return field_table.find(ne) == field_table.end() ? false : true;
}


void smrp::initRouting() {
    trace()<<"[info] Entering initRouting()";
    trace()<<"[info] Clearing routing table, construct primary path";
    initRoutingTable();
    auto ne=getNthTargetValueEntry(1, {});
    addRoutingEntry(std::string(SELF_NETWORK_ADDRESS),ne,1);
    updateFieldTableWithPE(ne.nw_address,SELF_NETWORK_ADDRESS,smrpPathStatus::USED);
    setTimer(smrpTimerDef::ENV_CHK, fp.env_c+getRNG(0)->doubleRand());
}

void smrp::initRoutingTable() {
    trace()<<"[info] Entering initRoutingTable()";
    routing_table.clear();
}

int smrp::calcNextPriFromRtTable(std::string ne, bool a_paths) {
    trace()<<"[info] Entering calcNextPriFromRtTable(ne="<<ne<<", a_paths="<<a_paths<<")";
    if(routing_table.empty()) {
        trace()<<"[info] Routing table empty.";
        return 0;
    }
    vector<int> pv;
    for(auto re: routing_table) {
        if(re.nw_address == ne && (smrpPathStatus::AVAILABLE == re.status || !a_paths)) {
            pv.push_back(re.prio);
        }
    }
    for(int i=1; i <= fp.pnum ; ++i) {
        if(std::find(pv.begin(),pv.end(),i) == pv.end()) {
            trace()<<"[info] Priority "<<i<<" missing, comes next.";
            return i;
        }
    }
    throw std::runtime_error("Max prio met");
}


void smrp::cleanRouting(std::string ne) {
    trace()<<"[info] Entering cleanRouting(ne="<<ne<<")";
    removeRoutingEntry(ne,1,true);
}

void smrp::logRouting() {
    trace()<<"[info] Entering logRouting()";
    for(auto re: routing_table) {
        trace()<<"[info] nw_address: "<<re.nw_address<<" next_hop: "<<re.next_hop<<" status: "<<pathStatusToStr(re.status)<<" prio: "<<re.prio;
    }
}

void smrp::logField() {
    trace()<<"[info] Entering logField()";
    for(auto ne: field_table) {
        trace()<<"[info] nw_address: "<<ne.second.nw_address;
    }
}

void smrp::addRoutingEntry(std::string nw_address, sm_node_entry ne, int prio) {
    addRoutingEntry(nw_address, ne, prio, smrpPathStatus::AVAILABLE, 0.0);
}

void smrp::addRoutingEntry(std::string nw_address, sm_node_entry ne, int prio, smrpPathStatus status, double timestamp) {
    trace()<<"[info] Entering addRoutingEntry(nw_address="<<nw_address<<", sm_node_entry.nw_address="<<ne.nw_address<<", prio="<<prio<<", status="<<pathStatusToStr(status)<<", timestamp="<<timestamp;
    for(auto it=routing_table.begin() ; it != routing_table.end() ; ++it) {
        if(it->nw_address == nw_address && it->prio == prio) {
            std::string("[error] record with prio already exists"); // throw?
        }
    }
    sm_routing_entry re;
    re.nw_address=nw_address;
    re.next_hop=ne.nw_address;
    re.target_value=targetFunction(ne);
    re.status=status;
    re.prio=prio;
    re.query_timestamp=timestamp;
    routing_table.push_back(re);
}

sm_node_entry smrp::getSinkFieldTableEntry() {
    trace()<<"[info] Entering getSinkFieldTableEntry()";
    if(field_table.find(getSinkAddress()) == field_table.end()) {
        trace()<<"[error] Sink address not found.";
        throw std::runtime_error("[error] No sink entry");
    }
    return field_table[getSinkAddress()];
}

void smrp::updateRoutingEntry(std::string nw_address, sm_node_entry ne, int prio, smrpPathStatus status) {
    trace()<<"[info] Entering updateRoutingEntry(nw_address="<<nw_address<<", sm_node_entry.nw_address="<<ne.nw_address<<", prio="<<prio<<", status="<<pathStatusToStr(status);
    for(auto &&re: routing_table) {
        trace()<<"[info] Entry: "<<re.nw_address<<", prio: "<<re.prio;
        if(re.nw_address == nw_address && re.prio==prio) {
            trace()<<"[info] record found";
            re.next_hop=ne.nw_address;
            re.status=status;
            re.target_value=targetFunction(ne);
            re.prio=prio;
            return;
        }
    }
    throw std::runtime_error("[error] No record found.");
}


bool smrp::checkRoutingEntryWithOtherPrio(std::string ne, int prio) {
    trace()<<"[info] Entering checkRoutingEntry(ne="<<ne<<", prio="<<prio<<")";
    for(auto re: routing_table) {
        if(re.nw_address == ne && re.prio!=prio && re.status==smrpPathStatus::AVAILABLE) {
            trace()<<"[info] Entry with other prio ("<<re.prio<<") exists";
            return true;
        }
    }
    return false;
}


bool smrp::checkRoutingEntry(std::string ne, int prio) {
    trace()<<"[info] Entering checkRoutingEntry(ne="<<ne<<", prio="<<prio<<")";
    for(auto re: routing_table) {
        if(re.nw_address == ne && re.prio==prio && re.status==smrpPathStatus::AVAILABLE) {
            trace()<<"[info] Entry exists";
            return true;
        }
    }
    return false;
}

sm_routing_entry smrp::getRoutingEntry(std::string ne, int prio) {
    trace()<<"[info] Entering getRoutingEntry(ne="<<ne<<", prio="<<prio<<")";
    for(auto re: routing_table) {
        if(re.nw_address == ne && re.prio==prio && re.status==smrpPathStatus::AVAILABLE) {
            trace()<<"[info] Entry exists";
            return re;
        }
    }
    throw std::runtime_error("[error] Entry not found.");
}

void smrp::removeRoutingEntry(std::string ne, int prio, bool noprio=false) {
    trace()<<"[info] Entering removeRoutingEntry(ne="<<ne<<", prio="<<prio<<", noprio="<<noprio<<")";
    for(auto it=routing_table.begin() ; it != routing_table.end();) {
        if(it->nw_address==ne && (it->prio==prio || noprio )) {
            trace()<<"[info] Record present in routing table, erasing - ne: "<<ne<<" next_hop: "<<it->next_hop;
            it=routing_table.erase(it);
        } else {
            ++it;
        }
    }
}

double smrp::calculateTargetValue() {
    trace()<<"[info] Entering calculateTargetValue()";
    sm_node_entry min_ne;
    min_ne=hello_table.begin()->second;
    for(auto ne: hello_table) {
        if(ne.second.env < min_ne.env) {
            min_ne=ne.second;
        }
    }
    trace()<<"[info] min ne: "<<min_ne.nw_address<<"  min env: "<<min_ne.env<<" own env: "<<ff_app->getEmergencyValue();
    return (min_ne.env+ff_app->getEmergencyValue())/2;
}

void smrp::initFieldTable() {
    trace()<<"[info] initFieldTable()";
    field_table.clear();
}

sm_node_entry smrp::getNthTargetValueEntry(int order, std::vector<std::string> ne_lst) {
    trace()<<"[info] Entering getNthTargetValueEntry(order="<<order<<")";
    return getNthTargetValueEntry(order,ne_lst,false);
}

sm_node_entry smrp::getNthTargetValueEntry(int order, std::vector<std::string> ne_lst, bool use_pe) {
    trace()<<"[info] Entering getNthTargetValueEntry(order="<<order<<", use_pe="<<use_pe<<")";
    if(field_table.size() < order) {
        throw std::runtime_error("[error] Less record than order");
    }
    trace()<<"[info] Field table size: "<<field_table.size();

    std::vector<sm_node_entry> fv;
    for(auto it = field_table.begin() ; it != field_table.end() ; ++it) {
        bool found=false;
        for(auto ne: ne_lst) {
            if(ne==it->second.nw_address) {
                trace()<<"[info] Record "<<it->second.nw_address<<" filtered out due to ne_lst.";
                found=true;
            }
        }
        if(use_pe) {
            for(auto pe: it->second.pe) {
                if(pe.status==smrpPathStatus::DEAD && pe.origin==SELF_NETWORK_ADDRESS) {
                    trace()<<"[info] Record "<<it->second.nw_address<<" filtered out due to DEAD.";
                    found=true;
                }
            }
        }
        if(!found) {
            trace()<<"[info] Adding record "<<it->second.nw_address;
            for(auto it2 = it->second.hop.begin() ; it2 != it->second.hop.end() ; ++it2) {
                trace()<<"[info] Adding with sink: "<<it2->first<<" hop: "<<it2->second;
                auto ne = it->second;
                ne.hop.clear();
                ne.hop.insert({it2->first, it2->second});
                fv.push_back(ne);
            }
        }
    }
    if(fv.size()<order) {
        throw std::runtime_error("[error] No record left");
    }

    std::sort(fv.begin(), fv.end(), [this](sm_node_entry a, sm_node_entry b) { return targetFunction(a) > targetFunction(b);  });
    
    return fv[order-1];
}

double smrp::targetFunction(sm_node_entry a) {
    trace()<<"[info] Entering targetFunction(a)";
    double ret_val = (1.0 - fp.alpha - fp.beta) * 1.0/(a.hop.begin()->second + 1) +
                     fp.alpha * a.trg + fp.beta * a.nrg;
    trace()<<"[info] targetFunction value for node "<<a.nw_address <<": "<<ret_val;
    return ret_val;
}


int smrp::numOfAvailPaths(std::string ne, bool only_available=true, bool count_dead=false) {
    trace()<<"[info] Entering numOfAvailPaths(ne="<<ne<<")";
    int ret_val=0;
    for(auto re: routing_table) {
        if(ne == re.nw_address) {
           if( (!only_available || re.status==smrpPathStatus::AVAILABLE)
            || (!count_dead && re.status==smrpPathStatus::DEAD) ) {
            ++ret_val;
           }
        }
    }
    trace()<<"[info] Number of available paths: "<<ret_val;
    return ret_val;
}

void smrp::sendQuery(std::string ne) {
    trace()<<"[info] Entering sendQuery(ne="<<ne<<")";
    smrpQueryPacket *query_pkt=new smrpQueryPacket("SMRP QUERY packet", NETWORK_LAYER_PACKET);
    query_pkt->setByteLength(netDataFrameOverhead);
    query_pkt->setSmrpPacketKind(smrpPacketDef::QUERY_PACKET);
    query_pkt->setOrigin(ne.c_str());
    query_pkt->setSource(SELF_NETWORK_ADDRESS);
    query_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);

    toMacLayer(query_pkt, BROADCAST_MAC_ADDRESS);

}

void smrp::sendQueryAck(std::string origin, std::string dst, bool used) {
    trace()<<"[info] Entering sendQueryAck(origin="<<origin<<", dst="<<dst<<", used="<<used<<")";
    smrpQueryAckPacket *query_ack_pkt=new smrpQueryAckPacket("SMRP QUERY_ACK packet", NETWORK_LAYER_PACKET);
    query_ack_pkt->setByteLength(netDataFrameOverhead);
    query_ack_pkt->setSmrpPacketKind(smrpPacketDef::QUERY_ACK_PACKET);
    query_ack_pkt->setOrigin(origin.c_str());
    query_ack_pkt->setSource(SELF_NETWORK_ADDRESS);
    query_ack_pkt->setDestination(dst.c_str());

    query_ack_pkt->setUsed(used);

    toMacLayer(query_ack_pkt, resolveNetworkAddress(dst.c_str()));

}

bool smrp::queryStarted(std::string ne, int prio=0) {
    trace()<<"[info] Entering queryStarted(ne="<<ne<<")";
    for(auto re: routing_table) {
        if(re.nw_address==ne && re.status==smrpPathStatus::UNDER_QUERY && (re.prio == prio &&re.prio>1 || prio==0 )) {
            trace()<<"[info] Query started";
            return true;
        }
    }
    trace()<<"[info] Query not started";
    return false;
}

bool smrp::queryCompleted(std::string ne, int prio=0) {
    trace()<<"[info] Entering queryCompleted(ne="<<ne<<")";
    for(auto re: routing_table) {
        if(re.nw_address==ne && re.status==smrpPathStatus::UNDER_QUERY && re.query_timestamp+fp.query < getClock().dbl() && (re.prio==prio && prio > 0 || prio==0)) {
            trace()<<"[info] Query completed";
            return true;
        }
    }
    trace()<<"[info] Query not completed";
    return false;
}


void smrp::sendData(sm_routing_entry re, cPacket *pkt) {
    trace()<<"[info] Entering sendData(re.next_hop="<<re.next_hop<<")";
    smrpDataPacket *data_pkt=new smrpDataPacket("SMRP DATA packet", NETWORK_LAYER_PACKET);

    data_pkt->setByteLength(netDataFrameOverhead);
    data_pkt->setSmrpPacketKind(smrpPacketDef::DATA_PACKET);
    data_pkt->setOrigin(re.nw_address.c_str());
    data_pkt->setSource(SELF_NETWORK_ADDRESS);
    data_pkt->setDestination(re.next_hop.c_str());

    data_pkt->setPri(re.prio);

    data_pkt->encapsulate(pkt);

    toMacLayer(data_pkt, resolveNetworkAddress(re.next_hop.c_str()));
}

void smrp::sendData(std::string ne, cPacket *pkt) {
    trace()<<"[info] Entering sendData(ne="<<ne<<")";
    smrpDataPacket *data_pkt=new smrpDataPacket("SMRP DATA packet", NETWORK_LAYER_PACKET);

    data_pkt->setByteLength(netDataFrameOverhead);
    data_pkt->setSmrpPacketKind(smrpPacketDef::DATA_PACKET);
    data_pkt->setOrigin(SELF_NETWORK_ADDRESS);
    data_pkt->setSource(SELF_NETWORK_ADDRESS);
    data_pkt->setDestination(ne.c_str());

    data_pkt->setPri(1);

    data_pkt->encapsulate(pkt);

    toMacLayer(data_pkt, resolveNetworkAddress(ne.c_str()));
}

void smrp::forwardData(smrpDataPacket *data_pkt) {
    trace()<<"[info] Entering forwardData()";
    sm_routing_entry re;
    try {
        re=getPath(data_pkt->getOrigin(),data_pkt->getPri());
    } catch(std::runtime_error &e) {
        trace()<<"[error] "<<e.what();
        return;
    }
    trace()<<"[info] Next hop: "<<re.next_hop;
    data_pkt->setDestination(re.next_hop.c_str());
    data_pkt->setSource(SELF_NETWORK_ADDRESS);

    toMacLayer(data_pkt, resolveNetworkAddress(re.next_hop.c_str()));
    ++forw_pkt_count;
}

void smrp::sendRetreat(smrpDataPacket *data_pkt) {
    trace()<<"[info] Entering sendRetreat()";
    smrpRetreatPacket *retreat_pkt=new smrpRetreatPacket("SMRP RETREAT packet",NETWORK_LAYER_PACKET);

    retreat_pkt->setByteLength(netDataFrameOverhead);
    retreat_pkt->setSmrpPacketKind(smrpPacketDef::RETREAT_PACKET);
    retreat_pkt->setOrigin(data_pkt->getOrigin());
    retreat_pkt->setSource(SELF_NETWORK_ADDRESS);
    retreat_pkt->setDestination(data_pkt->getSource());

    retreat_pkt->setPri(data_pkt->getPri());

    toMacLayer(retreat_pkt, resolveNetworkAddress(data_pkt->getSource()));

}

void smrp::sendAlarm(smrpAlarmDef alarm_kind, double env_val, double nrg_val, double trg_val) {
    trace()<<"[info] Entering sendAlarm(alarm_kind="<<alarm_kind<<", env_val="<<env_val<<", nrg_val="<<nrg_val<<", trg_val="<<trg_val<<")";
    smrpAlarmPacket *alarm_pkt=new smrpAlarmPacket("SMRP ALARM packet",NETWORK_LAYER_PACKET);

    alarm_pkt->setByteLength(netDataFrameOverhead);
    alarm_pkt->setSmrpPacketKind(smrpPacketDef::ALARM_PACKET);
    alarm_pkt->setOrigin(SELF_NETWORK_ADDRESS);
    alarm_pkt->setSource(SELF_NETWORK_ADDRESS);
    alarm_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);

    alarm_pkt->setSmrpAlarmKind(alarm_kind);

    if(alarm_kind==smrpAlarmDef::ENVIRONMENT_ALARM) {
        alarm_pkt->setEnv(env_val);
        alarm_pkt->setNrg(nrg_val);
        alarm_pkt->setTrg(trg_val);
    } 

    toMacLayer(alarm_pkt, BROADCAST_MAC_ADDRESS);
}  

bool smrp::checkPath(std::string ne) {
    trace()<<"[info] Entering checkPath(ne="<<ne<<")";
    for(auto entry: routing_table) {
        if(entry.nw_address == ne) {
            trace()<<"[info] Entry found with state "<<pathStatusToStr(entry.status);
            return true;
        }
    }
    trace()<<"[info] Entry not found.";
    return false;
}


bool smrp::checkNextHop(std::string ne) {
    trace()<<"[info] checkNextHop(ne="<<ne<<")";
    for(auto re: routing_table) {
        if(re.next_hop==ne) {
            return true;
        }
    }
    return false;
}

bool smrp::checkNextHop(std::string ne, int prio) {
    trace()<<"[info] checkNextHop(ne="<<ne<<", prio="<<prio<<")";
    for(auto re: routing_table) {
        if(re.next_hop==ne && re.nw_address==SELF_NETWORK_ADDRESS && re.prio==prio) {
            return true;
        }
    }
    return false;
}


sm_routing_entry smrp::getPath(std::string ne, int prio) {
    trace()<<"[info] Entering getPath(ne="<<ne<<", prio="<<prio<<")";
    for(auto entry: routing_table) {
        if(entry.nw_address==ne && entry.prio==prio && entry.status==smrpPathStatus::AVAILABLE) {
            return entry;
        }
    }
    throw std::runtime_error("[error] No path found");
}

void smrp::updateEntries(std::string ne, double env, double nrg, double trg) {
    trace()<<"[info] Entering updateEntries(ne="<<ne<<")";
    auto it=field_table.find(ne);
    if(it!=field_table.end()) {
        trace()<<"[info] Record present in field table, updating.";
        it->second.env=env;
        it->second.trg=trg;
    }
    for(auto &&e: routing_table) {
        if(e.next_hop==ne) {
            trace()<<"[info] Record present in routing table, updating.";
            e.target_value=targetFunction(it->second);
        }
    }
}

void smrp::removeEntries(std::string ne) {
    trace()<<"[info] Entering removeEntries(ne="<<ne<<")";
    if(field_table.find(ne) != field_table.end()) {
        trace()<<"[info] Record present in field table, erasing.";
        field_table.erase(ne);
    }
    for(auto it=routing_table.begin() ; it != routing_table.end();) {
        if(it->next_hop==ne) {
            trace()<<"[info] Record present in routing table, erasing.";
            it=routing_table.erase(it);
        } else {
            ++it;
        }
    }
}

sm_routing_entry smrp::getPath(std::string ne) {
    trace()<<"[info] Entering getPath(ne="<<ne<<")";
    std::vector<sm_routing_entry> rv;
    double tv_sum=0;
    double tv=0;
    double rnd=getRNG(0)->doubleRand();

    for(auto re: routing_table) {
        if(re.nw_address==ne && re.status==smrpPathStatus::AVAILABLE) {
            rv.push_back(re);
            tv_sum+=re.target_value;
            trace()<<"[info] Adding entry ne: "<<re.nw_address<<" next hop: "<<re.next_hop<<" tv: "<<re.target_value<<" prio: "<<re.prio;
        }
    }
    if(rv.size()==0) {
        throw std::runtime_error("[error] No routing entry");
    }

    trace()<<"[info] rand: "<<rnd<<" tv_sum: "<<tv_sum;

    for(auto re: rv) {
        tv+=re.target_value/tv_sum;
        if(rnd<tv) {
            trace()<<"[info] Selected entry: "<<re.next_hop;
            return re;
        }
    }

    trace()<<"[error] No route selected based on probability. Selecting first one next_hop: "<<rv[0].next_hop;
    return rv[0];
}

sm_node_entry smrp::findPath(std::string ne, std::vector<std::string> ne_lst) {
    trace()<<"[info] Entering findPath(ne="<<ne<<")";
    std::vector<sm_node_entry> nv;
    for(auto entry: field_table) {
        for(auto pe: entry.second.pe) {
            if(pe.origin == ne && pe.status == smrpPathStatus::AVAILABLE) {
                bool found=false;
                trace()<<"[info] Entry found: "<<entry.second.nw_address<<", pe.origin="<<pe.origin<<", pe.status="<<pathStatusToStr(pe.status);
                for(auto xe: ne_lst) {
                    if(xe==entry.second.nw_address) {
                        found=true;
                        trace()<<"[info] Exluded entry found: "<<xe;
                    }
                }
                if(!found) {
                    trace()<<"[info] Entry matches: "<<entry.second.nw_address;
                    for(auto it2 = entry.second.hop.begin() ; it2 != entry.second.hop.end() ; ++it2) {
                        trace()<<"[info] Adding with sink: "<<it2->first<<" hop: "<<it2->second;
                        auto ne = entry.second;
                        ne.hop.clear();
                        ne.hop.insert({it2->first, it2->second});
                        nv.push_back(ne);
                    }
                }
            }
        }
    }
    if(0==nv.size()) {
        trace()<<"[error] No path candidate, giving up.";
        throw std::runtime_error("[error] No path candidate");
    }

    trace()<<"[info] Removing routing entries from candidates";
    for(auto re: routing_table) {
        for(auto it=nv.begin() ; it != nv.end();) {
            if(re.next_hop == it->nw_address && re.nw_address == ne) {
                trace()<<"[info] "<<it->nw_address<<" already present in routing table, removing";
                it=nv.erase(it);
            } else {
                ++it;
            }
        }
    }

    if(0==nv.size()) {
        trace()<<"[error] No path candidate present after routing table based filtering, giving up.";
        throw std::runtime_error("[error] No path candidate after routing table based filtering");
    }

    std::sort(nv.begin(), nv.end(), [this](sm_node_entry a, sm_node_entry b) { return targetFunction(a) > targetFunction(b);  });

    trace()<<"[info] Entry selected node: "<<nv[0].nw_address<<" sink: "<<nv[0].hop.begin()->first<<" hop: "<<nv[0].hop.begin()->second;
    return nv[0];
}

void smrp::updateFieldTableWithQA(smrpQueryAckPacket *query_ack_pkt) {
    trace()<<"[info] Entering updateFieldTableWithQA(source="<<query_ack_pkt->getSource()<<", origin: "<<query_ack_pkt->getOrigin() <<", used: "<<query_ack_pkt->getUsed()<<")";
    smrpPathStatus status = query_ack_pkt->getUsed() ? smrpPathStatus::USED : smrpPathStatus::AVAILABLE;
    if(field_table.find(query_ack_pkt->getSource()) != field_table.end()) {
        trace()<<"[info] Record exists";
        for(auto &&pe: field_table[query_ack_pkt->getSource()].pe) {
            if(pe.origin==std::string(query_ack_pkt->getOrigin())) {
                trace()<<"[info] QUERY_ACK record exists: "<<pe.origin<<" status: "<<pe.status;
                pe.status=status;
                return;
            }
        }
        trace()<<"[info] QUERY_ACK record does not exists yet";
        field_table[query_ack_pkt->getSource()].pe.push_back({query_ack_pkt->getOrigin(),status });
    } else {
        trace()<<"[warn] Query responder does not exists in field_table";
    }
}

void smrp::updateFieldTableWithPE(std::string ne, std::string pe, smrpPathStatus status) {
    trace()<<"[info] Entering updateFieldTableWithPE(ne="<<ne<<", pe="<<pe<<", status="<<pathStatusToStr(status)<<")";
    if(field_table.find(ne) != field_table.end()) {
        trace()<<"[info] Record found";
        for(auto &&e: field_table[ne].pe) {
            if(e.origin==pe) {
                trace()<<"[info] Path entry found";
                e.status=status;
            }
        }
    } else {
        trace()<<"[error] Record not found, where is it gone?";
    }
}


void smrp::timerFiredCallback(int index) {
    trace()<<"[info] Entering timerFiredCallback(index="<<index<<")";
    switch (index) {
        case smrpTimerDef::SINK_START: {
            trace()<<"[timer] SINK_START timer expired";
            sendHello();
            setTimer(smrpTimerDef::BUILD_START, fp.ttl + getRNG(0)->doubleRand());
            if(fp.periodic_restart) {
                trace()<<"[info] Periodic restart active";
                cancelTimer(smrpTimerDef::RESTART);
                setTimer(smrpTimerDef::RESTART,fp.restart);
            }
            setState(smrpStateDef::LEARN);
            break;
        }
        case smrpTimerDef::RESTART: {
            trace()<<"[timer] RESTART timer expired";
            sendHello();
            setTimer(smrpTimerDef::RESTART,fp.restart);
            setState(smrpStateDef::LEARN);
            setTimer(smrpTimerDef::BUILD_START, fp.ttl + getRNG(0)->doubleRand());
            break;
        }
        case smrpTimerDef::BUILD_START: {
            trace()<<"[timer] BUILD_START expired"; 
            sendField(getHop(), 1.0, 1.0, 1.0);
            setState(smrpStateDef::WORK);
            break;
        }
        case smrpTimerDef::TTL: {
            trace()<<"[timer] TTL timer expired";
            setState(smrpStateDef::BUILD);
            sendField(getHop(), ff_app->getEnergyValue(), ff_app->getEmergencyValue(), calculateTargetValue());
            setTimer(smrpTimerDef::FIELD, fp.field + getRNG(0)->doubleRand());
            break;
        }
        case smrpTimerDef::FIELD: {
            trace()<<"[timer] FIELD timer expired";
            setState(smrpStateDef::WORK);
            try { 
                 initRouting();
                 setState(smrpStateDef::WORK);
             } catch (std::runtime_error &e) {
                trace()<<"[error] "<<e.what();
                setState(smrpStateDef::INIT);
            }
            break;
        }
        case smrpTimerDef::ENV_CHK: {
            trace()<<"[timer] ENV_CHK timer expired";
            double nrg_val=ff_app->getEnergyValue();
            double new_env_val=ff_app->getEmergencyValue();
            if(nrg_val<fp.n_lim) {
                trace()<<"[info] Energy below limit.";
                sendAlarm(smrpAlarmDef::ENERGY_ALARM,0.0,0.0,0.0);
                break;
            }
            setTimer(smrpTimerDef::ENV_CHK, fp.env_c+getRNG(0)->doubleRand());
            trace()<<"[info] Sensor difference - abs("<<new_env_val<<"-"<<env_val<<")="<<abs(new_env_val-env_val);
            if(abs(new_env_val-env_val)>fp.gamma) {
                trace()<<"[info] Sensor reading difference exceeds gamma - new_env_val: "<<new_env_val<<" env_val: "<<env_val;
                sendAlarm(smrpAlarmDef::ENVIRONMENT_ALARM,new_env_val,nrg_val, calculateTargetValue());
                env_val=new_env_val;
            }
            break;
        }
        default: {
            trace()<<"[error] Unknown timer expired: "<<index;
            break;
        }
    }
}

void smrp::fromApplicationLayer(cPacket * pkt, const char *destination) {
    trace()<<"[info] Entering fromApplicationLayer(..)";

    if(0==std::strcmp(destination,BROADCAST_NETWORK_ADDRESS)) {
        trace()<<"[info] Broadcast network address";
        sendData(std::string(destination),pkt);
        return;
    }

    switch (getState()) {
        case smrpStateDef::INIT: {
            trace()<<"[info] In INIT state, can't route packet";
            break;
        }
        case smrpStateDef::LEARN: {
            trace()<<"[error] In LEARN state, can't route packet";
            break;
        }
        case smrpStateDef::BUILD: {
            trace()<<"[info] In BUILD state, can't route packet";
            break;
        }
        case smrpStateDef::WORK: {
            trace()<<"[info] In WORK state, routing";
            if(numOfAvailPaths(SELF_NETWORK_ADDRESS,fp.a_paths,fp.c_dead)==0) {
                trace()<<"[error] No route available";
                break;
            }
            else if(numOfAvailPaths(SELF_NETWORK_ADDRESS,fp.a_paths,fp.c_dead) < fp.pnum) {
                trace()<<"[info] Path number not met.";
                auto pri=calcNextPriFromRtTable(SELF_NETWORK_ADDRESS,fp.a_paths);
                trace()<<"[info] Establishing path with priority "<<pri;
                if(checkRoutingEntry(SELF_NETWORK_ADDRESS, pri)) {
                    trace()<<"[error] Path with priority "<<pri<<" exists.";
                    throw runtime_error("What?");
                    break;
                }
                trace()<<"[info] No path present with priority "<<pri<<".";
                trace()<<"[info] Check query status";
                if(queryCompleted(SELF_NETWORK_ADDRESS)) {
                    trace()<<"[info] Query completed";
                    sm_node_entry ne;
                    try {
                        ne=findPath(SELF_NETWORK_ADDRESS,{SELF_NETWORK_ADDRESS});
                        updateFieldTableWithPE(ne.nw_address,SELF_NETWORK_ADDRESS,smrpPathStatus::USED);
                    } catch (std::runtime_error &e) {
                        trace()<<"[error] "<<e.what();
                        trace()<<"[error] How do we got here?";
                        removeRoutingEntry(SELF_NETWORK_ADDRESS, pri);
                        break;
                    }
                    updateRoutingEntry(SELF_NETWORK_ADDRESS,ne,pri,smrpPathStatus::AVAILABLE);
                } else if(queryStarted(SELF_NETWORK_ADDRESS)) {
                    trace()<<"[info] Query ongoing.";
                } else {
                    trace()<<"[info] No path with priority "<<pri<<" available.";
                    sm_node_entry ne;
                    ne.nw_address="0";
                    ne.hop[0]=0;
                    ne.nrg=0;
                    ne.env=0;
                    ne.trg=0;
                    ne.pe.push_back({"0",UNDER_QUERY});

                    addRoutingEntry(SELF_NETWORK_ADDRESS, ne, pri, smrpPathStatus::UNDER_QUERY,getClock().dbl());
                    sendQuery(SELF_NETWORK_ADDRESS);
                }

            }
            sendData(getPath(SELF_NETWORK_ADDRESS),pkt);
            break;
        }
        case smrpStateDef::MOBILITY: {
            trace()<<"[info] In MOBILITY state, can't route packet";
            break;
        }
        case smrpStateDef::RE_LEARN: {
            trace()<<"[info] In RE_LEARN state, can't route packet";
            break;
        }
        default: {
            trace()<<"[error] In unknown state, can't route packet";
            break;
        }
    }
}


void smrp::fromMacLayer(cPacket * pkt, int srcMacAddress, double rssi, double lqi) {
    trace()<<"[info] Entering fromMacLayer(..srcMacAddress="<<srcMacAddress<<")";
    smrpPacket *smrp_pkt=dynamic_cast<smrpPacket *>(pkt);
    if(!smrp_pkt) {
        trace()<<"[error] Dynamic cast of packet failed";
    }

    trace()<<"[info] SMRP packet received from MAC: "<<srcMacAddress<<" NW: "<<smrp_pkt->getSource();

    logRouting();

    switch (smrp_pkt->getSmrpPacketKind()) {
        case smrpPacketDef::HELLO_PACKET: {
            trace()<<"[info] HELLO_PACKET received";
            smrpHelloPacket *hello_pkt=dynamic_cast<smrpHelloPacket *>(pkt);
            if(getClock().dbl() - hello_pkt->getTimestamp() > fp.ttl) {
                trace()<<"[info] HELLO_PACKET expired, timestamp: "<<hello_pkt->getTimestamp()<<" clock: "<<getClock().dbl();
                break;
            }

            if(isSink()) {
                trace()<<"[info] Node is sink, HELLO_PACKET discarded";
                break;
            }

            if(getState()==smrpStateDef::BUILD) {
                trace()<<"[info] Node in BUILD state, discarding HELLO_PACKET";
                break;
            }

            if(getState()==smrpStateDef::WORK) {
                trace()<<"[info] Node in WORK state, re-learn starts";
                setState(smrpStateDef::LEARN);
                initHelloTable();
                initFieldTable();
                setTimer(smrpTimerDef::TTL, fp.ttl + getRNG(0)->doubleRand());
            }

            if(getState()==smrpStateDef::INIT) {
                trace()<<"[info] Node in INIT state, transitioning to LEARN and arming TTL timer";
                setState(smrpStateDef::LEARN);
                initHelloTable();
                initFieldTable();
                // Add some random to TTL to compensate propagation delay
                setTimer(smrpTimerDef::TTL, fp.ttl + getRNG(0)->doubleRand());
            }

            if(updateHop(hello_pkt->getSink(),hello_pkt->getHop())) {
                trace()<<"[info] Updating hop status";
                setHop(hello_pkt->getSink(),hello_pkt->getHop()+1);
                sendHello(hello_pkt->getSink(),getHop(hello_pkt->getSink()),ff_app->getEmergencyValue(), ff_app->getEnergyValue(), hello_pkt->getTimestamp());
            }
            updateHelloTable(hello_pkt);

            break;
        }
        case smrpPacketDef::FIELD_PACKET: {
            trace()<<"[info] FIELD_PACKET received";
            smrpFieldPacket *field_pkt=dynamic_cast<smrpFieldPacket *>(smrp_pkt);
            if(isSink()) {
                trace()<<"[info] Node is sink, FIELD_PACKET discarded";
                break;
            }
            if(getState()==smrpStateDef::INIT) {
                trace()<<"[info] Node in INIT state, doing nothing.";
            }

            if(getState()==smrpStateDef::LEARN) {
                trace()<<"[info] Node in LEARN state, still updating Field table";
                updateFieldTable(field_pkt);
            }
            if(getState()==smrpStateDef::BUILD) {
                trace()<<"[info] Node in BUILD state, updating Field table, if possible";
                if(checkHelloTable(std::string(field_pkt->getSource()))) {
                    updateFieldTable(field_pkt);
                } else {
                    trace()<<"[info] Entry not found in hello table, discarding";
                }
            }
            if(getState()==smrpStateDef::WORK) {
                trace()<<"[info] Node in WORK state, discarding FIELD_PACKET";
            }
            if(getState()==smrpStateDef::MOBILITY) {
                trace()<<"[info] Node in MOBILITY state, discarding FIELD_PACKET";
            }
            if(getState()==smrpStateDef::RE_LEARN) {
                trace()<<"[info] Node in RE_LEARN state, processing FIELD_PACKET";
                try {
                    updateHelloTable(field_pkt);
                } catch (std::runtime_error &e) {
                    trace()<<e.what();
                    trace()<<"[error] Cannot add any entry to hello table";
                    break;
                }
                updateFieldTable(field_pkt);
            }
            break;
        }
        case smrpPacketDef::QUERY_PACKET: {
            trace()<<"[info] QUERY_PACKET received";
            smrpQueryPacket *query_pkt=dynamic_cast<smrpQueryPacket *>(smrp_pkt);
            sendQueryAck(query_pkt->getOrigin(),query_pkt->getSource(), checkPath(query_pkt->getOrigin()));
            break;
        }
        case smrpPacketDef::QUERY_ACK_PACKET: {
            trace()<<"[info] QUERY_ACK_PACKET received";
            smrpQueryAckPacket *query_ack_pkt=dynamic_cast<smrpQueryAckPacket *>(smrp_pkt);
            updateFieldTableWithQA(query_ack_pkt);
            break;
        }
        case smrpPacketDef::DATA_PACKET: {
            trace()<<"[info] DATA_PACKET received";
            smrpDataPacket *data_pkt=dynamic_cast<smrpDataPacket *>(smrp_pkt);
            trace()<<"[info] origin: "<<data_pkt->getOrigin()<<" pri: "<<data_pkt->getPri();
            if(std::string(data_pkt->getDestination()) == BROADCAST_NETWORK_ADDRESS) {
                trace()<<"[info] Broadcast pkt received, forwarding to application layer";
                toApplicationLayer(decapsulatePacket(data_pkt));
                break;
            }
            if(std::string(data_pkt->getOrigin()) == SELF_NETWORK_ADDRESS) {
                trace()<<"[error] Loop detected, origin equals destination";
                break;
            }

            if(isSink() && SELF_NETWORK_ADDRESS == std::string(data_pkt->getDestination())) {
                trace()<<"[info] Data packet arrived at sink";
                data_pkt->setSource(data_pkt->getOrigin());
                toApplicationLayer(decapsulatePacket(data_pkt));
                break;
            }

            auto pri=data_pkt->getPri();

            if(checkRoutingEntry(data_pkt->getOrigin(), pri)) {
                trace()<<"[info] Path with priority "<<pri<<" exists.";
                forwardData(data_pkt->dup());
                break;
            }
            trace()<<"[info] No path present with priority "<<pri<<".";
            trace()<<"[info] Check query status";
            if(checkRoutingEntryWithOtherPrio(data_pkt->getOrigin(), pri)) {
                trace()<<"[info] Path present with other priority, time to retreat.";
                sendRetreat(data_pkt);
                break;
            }
            if(queryCompleted(data_pkt->getOrigin(),data_pkt->getPri())) {
                trace()<<"[info] Query completed";
                sm_node_entry ne;
                try {
                    ne=findPath(data_pkt->getOrigin(),{data_pkt->getSource() } );
                    updateFieldTableWithPE(ne.nw_address,data_pkt->getOrigin(),smrpPathStatus::USED);
                } catch (std::runtime_error &e) {
                    trace()<<"[error] "<<e.what();
                    removeRoutingEntry(data_pkt->getOrigin(), data_pkt->getPri());
                    sendRetreat(data_pkt);
                    break;
                }
                updateRoutingEntry(data_pkt->getOrigin(),ne,pri,smrpPathStatus::AVAILABLE);
                forwardData(data_pkt->dup());
            } else if(queryStarted(data_pkt->getOrigin(), data_pkt->getPri())) {
                trace()<<"[info] Query ongoing, dropping packet.";
                break;
            } else {
                trace()<<"[info] No path with priority "<<pri<<" available.";
                sm_node_entry ne;
                addRoutingEntry(data_pkt->getOrigin(), ne, pri, smrpPathStatus::UNDER_QUERY,getClock().dbl());
                sendQuery(data_pkt->getOrigin());
                break;
            }
            break;
        }
        case smrpPacketDef::RETREAT_PACKET: {
            trace()<<"[info] RETREAT_PACKET received";
            smrpRetreatPacket *retreat_pkt=dynamic_cast<smrpRetreatPacket*>(smrp_pkt);
            sm_routing_entry re;
            try {
                re=getRoutingEntry(retreat_pkt->getOrigin(),retreat_pkt->getPri());
            } catch (std::runtime_error &e) {
                trace()<<"[error] "<<e.what();
                break;
            }
            if(re.next_hop == retreat_pkt->getSource()) {
                if(0==std::strcmp(retreat_pkt->getOrigin(),SELF_NETWORK_ADDRESS)) {
                    trace()<<"[info] Node is the origin";
                    sm_node_entry ne;
                    updateFieldTableWithPE(retreat_pkt->getSource(), retreat_pkt->getOrigin(), smrpPathStatus::DEAD);
                    updateRoutingEntry(SELF_NETWORK_ADDRESS, ne, retreat_pkt->getPri(), smrpPathStatus::DEAD);
                    // this is going to cause an inifity loop
                } else {
                    trace()<<"[info] Node is interim";
                    sm_node_entry ne;
                    updateRoutingEntry(retreat_pkt->getOrigin(), ne, retreat_pkt->getPri(), smrpPathStatus::UNDER_QUERY);
                    updateFieldTableWithPE(retreat_pkt->getSource(), retreat_pkt->getOrigin(), smrpPathStatus::DEAD);
                }

            } else {
                trace()<<"[error] Retreat sender not next hop";
            }
            break;
        }
        case smrpPacketDef::ALARM_PACKET: {
            trace()<<"[info] ALARM_PACKET received";
            smrpAlarmPacket *alarm_pkt=dynamic_cast<smrpAlarmPacket*>(smrp_pkt);
            switch (alarm_pkt->getSmrpAlarmKind()) {
                case smrpAlarmDef::ENERGY_ALARM: {
                    trace()<<"[info] ENERGY_ALARM received, removing node";
                    logRouting();
                    logField();
                    removeEntries(alarm_pkt->getSource());
                    break;
                }
                case smrpAlarmDef::ENVIRONMENT_ALARM: {
                    trace()<<"[info] ENVIRONMENT_ALARM received, updating tables";
                    if(checkFieldEntry(alarm_pkt->getSource())) {
                        trace()<<"[info] Sender present in field table";
                        updateFieldTableEntry(alarm_pkt->getSource(),alarm_pkt->getEnv(), alarm_pkt->getNrg(), alarm_pkt->getTrg());
                        // Maybe we shouldn't remove the routing entry
                        // that says: I'm routing the source's packet
                        // We should remove entries, where the node
                        // in question is the next hop
                        cleanRouting(alarm_pkt->getSource());
                    }
                    break;
                }
                case smrpAlarmDef::MOBILITY_ALARM: {
                    trace()<<"[info] MOBILITY_ALARM received, removing corresponding entries";
                    if(checkNextHop(alarm_pkt->getSource())) {
                        trace()<<"[info] Entry in routing table exists with "<<alarm_pkt->getSource()<<" as next hop";
                        removeEntries(alarm_pkt->getSource());
                        if(checkHelloTable(alarm_pkt->getSource())) {
                            removeHelloEntry(alarm_pkt->getSource());
                        }
                        // cleanRouting??
                    }
                    break;
                }
                case smrpAlarmDef::RELEARN_ALARM: {
                    trace()<<"[info] RELERN_ALARM received, sending FIELD packet";
                    sendField(getHop(), ff_app->getEnergyValue(), ff_app->getEmergencyValue(), calculateTargetValue());
                    break;
                }
                default: {
                    trace()<<"[error] Unknown, "<<alarm_pkt->getSmrpAlarmKind()<<" ALARM received";
                    break;
                }
            }
            break;
        }

        case smrpPacketDef::UNDEF_PACKET: {
            trace()<<"[error] UNDEF_PACKET received";
            break;
        }
        default: {
            trace()<<"[error] Unknown packet received with smrpPacketKind value: "<<smrp_pkt->getSmrpPacketKind();
            break;
        }
    }
}


void smrp::handleNetworkControlCommand(cMessage *msg) {
    trace()<<"[info] Entering handleNetworkControlCommand()";
    EmergencyMessage *app_msg=check_and_cast<EmergencyMessage *>(msg);
    if(!msg) {
        trace()<<"[error] Unknown Network Control Command Message";
    }
    switch(app_msg->getEvent()) {
        case MsgType::EMERGENCY: {
            trace()<<"[info] Application in Emergency state";
            break;
        }
        case MsgType::PREP_MOBILITY: {
            trace()<<"[info] Application preparing for mobility";
            setState(smrpStateDef::MOBILITY);
            sendAlarm(smrpAlarmDef::MOBILITY_ALARM,0.0,0.0,0.0);
            initHelloTable();
            initRoutingTable();
            initFieldTable();
            break;
        }
        case MsgType::RELEARN: {
            if(isSink()) {
                trace()<<"[info] Application finished mobility, restart network.";
                setTimer(smrpTimerDef::SINK_START,getRNG(0)->doubleRand());
            } else {
                trace()<<"[info] Application finished mobility, relearn.";
                setState(smrpStateDef::RE_LEARN);
                sendAlarm(smrpAlarmDef::RELEARN_ALARM,0.0,0.0,0.0);
                setTimer(smrpTimerDef::FIELD, fp.field + getRNG(0)->doubleRand());
            }
            break;
        }
        default: {
            trace()<<"[info] Unknown network control command";
            break;
        }
    }
}   



void smrp::finishSpecific() {
    trace()<<"[info] Entering finishSpecific()";
    trace()<<"[info] Routing table";
    for(auto re: routing_table) {
        trace()<<"[info] Origin="<<re.nw_address<<", next hop="<<re.next_hop<<", status="<<pathStatusToStr(re.status)<<", prio="<<re.prio;

    }
    trace()<<"[info] Field table";
    for(auto ne: field_table) {
        trace()<<"[info] Addr="<<ne.second.nw_address;
       for(auto pe: ne.second.pe) {
           trace()<<"[info] Path entry - origin: "<<pe.origin<<" status: "<<pathStatusToStr(pe.status);
       }
    }
    if(!std::strcmp(SELF_NETWORK_ADDRESS,"0")) {
        generateYaml();
    }
}

void smrp::generateYaml() {
    cTopology *topo;        // temp variable to access energy spent by other nodes
    topo = new cTopology("topo");
    topo->extractByNedTypeName(cStringTokenizer("node.Node").asVector());
    y_out<<YAML::BeginSeq;
    for(int i=0 ; i < topo->getNumNodes() ; ++i) {
        y_out<<YAML::BeginMap;
        auto pos=dynamic_cast<VirtualMobilityManager *>(topo->getNode(i)->getModule()->getSubmodule("MobilityManager"))->getLocation();
        y_out<<YAML::Key<<"node";
        y_out<<YAML::Value<<i;
        y_out<<YAML::Key<<"x";
        y_out<<YAML::Value<<pos.x;
        y_out<<YAML::Key<<"y";
        y_out<<YAML::Value<<pos.y;
        auto *smrp_instance = dynamic_cast<smrp*>
                (topo->getNode(i)->getModule()->getSubmodule("Communication")->getSubmodule("Routing"));

        serializeRoutingTable(smrp_instance->getRoutingTable());
        y_out<<YAML::Key<<"forw_data_pkt_count";
        y_out<<YAML::Value<<smrp_instance->getForwDataPkt();
        y_out<<YAML::Key<<"hop";
        y_out<<YAML::Value<<smrp_instance->getHop();
        y_out<<YAML::Key<<"state";
        auto res_mgr=dynamic_cast<ResourceManager *>(topo->getNode(i)->getModule()->getSubmodule("ResourceManager"));
        y_out<<(res_mgr->isDead()?"DEAD": stateToStr(smrp_instance->getState()));
        y_out<<YAML::EndMap;
    }
    y_out<<YAML::EndSeq;
    
    ofstream loc_pdr_file("loc_pdr.yaml");
    loc_pdr_file<<y_out.c_str();
    loc_pdr_file.close();

    YAML::Emitter ys_out;
    ys_out<<YAML::BeginSeq;
    for(auto se: state_chng_log) {
        ys_out<<YAML::BeginMap;
        ys_out<<YAML::Key<<"node";
        ys_out<<YAML::Value<<se.node;
        ys_out<<YAML::Key<<"timestamp";
        ys_out<<YAML::Value<<se.timestamp;
        ys_out<<YAML::Key<<"state";
        ys_out<<YAML::Value<<stateToStr(se.state);
        ys_out<<YAML::Key<<"energy";
        ys_out<<YAML::Value<<se.energy;
        ys_out<<YAML::Key<<"total_energy";
        ys_out<<YAML::Value<<se.total_energy;
        ys_out<<YAML::EndMap;

    }
    ys_out<<YAML::EndSeq;
    ofstream state_file("state_chng.yaml");
    state_file<<ys_out.c_str();
    state_file.close();

    delete(topo);
}

void smrp::serializeRoutingTable(std::vector<sm_routing_entry> rt) {
    y_out<<YAML::Key<<"routing_table";
    y_out<<YAML::Value;
    y_out<<YAML::BeginSeq;
    for(auto re: rt) {
        y_out<<YAML::BeginMap;
        y_out<<YAML::Key<<"origin";
        y_out<<YAML::Value<<re.nw_address;
        y_out<<YAML::Key<<"next_hop";
        y_out<<YAML::Value<<re.next_hop;
        y_out<<YAML::Key<<"target_value";
        y_out<<YAML::Value<<re.target_value;
        y_out<<YAML::Key<<"status";
        y_out<<YAML::Value<<pathStatusToStr(re.status);
        y_out<<YAML::Key<<"prio";
        y_out<<YAML::Value<<re.prio;
        y_out<<YAML::EndMap;
    }

    y_out<<YAML::EndSeq;
}

