/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2022                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/

#include "node/communication/routing/hdmrp/hdmrp.h"

Define_Module(hdmrp);

void hdmrp::startup() {
    cModule *appModule = getParentModule()->getParentModule()->getSubmodule("Application");

    // Define role
    if(appModule->hasPar("isSink")) {
        appModule->par("isSink")?initRole(hdmrpRoleDef::SINK):initRole(hdmrpRoleDef::NON_ROOT);
    } else {
        role=hdmrpRoleDef::NON_ROOT;
        throw cRuntimeError("\nHDMRP nodes require the parameter role");
    }

    no_role_change=appModule->hasPar("noRoleChange")?appModule->par("noRoleChange"):false;

    if(no_role_change) {
       if(appModule->hasPar("Role")) {
           if(0==appModule->par("Role").stdstringValue().compare("Root")) {
               initRole(hdmrpRoleDef::ROOT);
           }
           else if(0==appModule->par("Role").stdstringValue().compare("SubRoot")) {
            initRole(hdmrpRoleDef::SUB_ROOT);
           } else {
               throw cRuntimeError("Role change not allowed without a valid role");
           }
       } else {
           throw cRuntimeError("Role change not allowed without a role");
       }
    }

    min_rreq_rssi=hasPar("min_rreq_rssi")?par("min_rreq_rssi").doubleValue():-100.0;

    if(appModule->hasPar("isMaster")) {
       setMaster(appModule->par("isMaster"));
    } else {
        setMaster(false);
    }
    auto yes_no = [](auto value) { return value?"yes":"no";};

    trace()<<"Sink: "<<yes_no(isSink())<<" Master: "<<yes_no(isMaster());;

    // T_l timer
    t_l=par("t_l");

    t_rreq=par("t_rreq");

    t_relay=par("t_relay");

    t_start=par("t_start");

    t_rsnd=par("t_rsnd");

    t_pkt_hist=par("t_pkt_hist");

    rep_limit=par("rep_limit");

    send_path_failure=par("send_path_failure");

    store_all_paths=par("store_all_paths");

    event_ack_req_period=par("event_ack_req_period");
    event_pkt_counter=0;
    // Set round to 0
    initRound();
    initMinorRound();
    initState(hdmrpStateDef::WORK);

    if(isSink()) {
        setTimer(hdmrpTimerDef::SINK_START,t_start);
        //Trigger 1st RREQ, timer should be also here
    }
    setTimer(hdmrpTimerDef::ACK_HIST_PURGE,t_pkt_hist);
    d_pkt_seq=8;
    sent_data_pkt=0;
    recv_pkt=0;
    s_rssi=0.0;
    s_lqi=0.0;
    declareOutput("Data packets");

}

bool hdmrp::isSink() const {
    return hdmrpRoleDef::SINK==role;
}

bool hdmrp::isRoot() const {
    return hdmrpRoleDef::ROOT==role;
}

bool hdmrp::isSubRoot() const {
    return hdmrpRoleDef::SUB_ROOT==role;
}

bool hdmrp::isNonRoot() const {
    return hdmrpRoleDef::NON_ROOT==role;
}

void hdmrp::initRole(hdmrpRoleDef new_role) {
    trace()<<"Role initialized to: "<<new_role;
    role=new_role;
}

void hdmrp::setRole(hdmrpRoleDef new_role) {
    trace()<<"Role change: "<<role<<"->"<<new_role;
    role=new_role;
}

bool hdmrp::isMaster() const {
    return master; 
}

void hdmrp::setMaster(bool master) {
    this->master=master;
}

bool hdmrp::isWorkingState() const {
    return hdmrpStateDef::WORK==state;
}

bool hdmrp::isLearningState() const {
    return hdmrpStateDef::LEARN==state;
}

void hdmrp::setState(hdmrpStateDef new_state) {
    trace()<<"State change: "<<state<<"->"<<new_state;
    state=new_state;
}

void hdmrp::initState(hdmrpStateDef new_state) {
    trace()<<"State initializied to: "<<new_state;
    state=new_state;
}


void hdmrp::initRound() {
    round = 0;
}

void hdmrp::newRound() {
    ++round;
}

bool hdmrp::isNewRound(hdmrpPacket* pkt) const {
    return pkt->getRound()>round;
}

bool hdmrp::isSameRound(hdmrpPacket* pkt) const {
    return pkt->getRound()==round;
}


int hdmrp::getRound() const {
    return round;
}

void hdmrp::setRound(int new_round) {
    round=new_round;
    minor_round=0;
}

void hdmrp::initMinorRound() {
    minor_round = 0;
}

void hdmrp::newMinorRound() {
    ++minor_round;
}

bool hdmrp::isNewMinorRound(hdmrpPacket* pkt) const {
    return pkt->getMinor_round()>minor_round;
}

void hdmrp::setMinorRound(int new_minor_round) {
    minor_round=new_minor_round;
}

int  hdmrp::getMinorRound() const {
    return minor_round;
}

std::vector<int> hdmrp::getPath_filter_array(hdmrpPacket *pkt) {
    vector<int> path_array;
    for(int i=0 ; i < pkt->getPath_filterArraySize() ; ++i) {
        path_array.push_back(pkt->getPath_filter(i));
    }
    return path_array;
}

void hdmrp::setPath_filter(hdmrpPacket *pkt, std::vector<int> path_array) {
    pkt->setPath_filterArraySize(path_array.size());
    for(int i=0 ; i < path_array.size() ; ++i) {
        pkt->setPath_filter(i,path_array[i]);
    }
}

bool hdmrp::matchPathFilter(hdmrpPacket *pkt) {
    auto paths=getPath_filter_array(pkt);
    trace()<<"matchPathFilter called";
    for(int i=0 ; i < paths.size() ; ++i) {
        try {
            getRoute(paths[i]);
            trace()<<"matched";
            return true;
        } catch (std::exception e) {}
    }
    return false;
}

void hdmrp::sendMinorRREQ(std::vector<int> path_array) {
    sendMinorRREQ(getRound(),getMinorRound(),0,isMaster()?1:0,1,path_array);
}

void hdmrp::sendMinorRREQ(int round, int minor_round, hdmrp_path path) {
    sendMinorRREQ(round, minor_round, path.path_id, path.nmas+isMaster()?1:0, path.len+1, path.path_filter);
}

void hdmrp::sendMinorRREQ(int round, int minor_round, hdmrp_path path, std::vector<int> path_array) {
    sendMinorRREQ(round, minor_round, path.path_id, path.nmas+isMaster()?1:0, path.len+1, path_array);
}


void hdmrp::sendMinorRREQ(int round, int minor_round, int path_id, int nmas, int len, vector<int> path_array) {
    trace()<<"sendMinorRREQ() called";
    hdmrpPacket *RREQPkt=new hdmrpPacket("HDMRP RREQ packet", NETWORK_LAYER_PACKET);
    RREQPkt->setHdmrpPacketKind(hdmrpPacketDef::RREQ_PACKET);
    RREQPkt->setSource(SELF_NETWORK_ADDRESS);
    RREQPkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    RREQPkt->setRound(round);
    RREQPkt->setMinor_round(minor_round);
    RREQPkt->setPath_id(path_id);
    RREQPkt->setNmas(nmas);
    RREQPkt->setLen(len);
    setPath_filter(RREQPkt, path_array);

    RREQPkt->setByteLength(7);

    toMacLayer(RREQPkt, BROADCAST_MAC_ADDRESS);

    trace()<<"Round #: "<<getRound();

}

void hdmrp::sendMinorSRREQ(vector<int> path_array) {
    trace()<<"sendMinorSRRQ() called";
    hdmrpPacket *SRREQPkt=new hdmrpPacket("HDMRP Sink RREQ packet", NETWORK_LAYER_PACKET);
    SRREQPkt->setHdmrpPacketKind(hdmrpPacketDef::SINK_RREQ_PACKET);
    SRREQPkt->setSource(SELF_NETWORK_ADDRESS);
    SRREQPkt->setDestination(BROADCAST_NETWORK_ADDRESS);

    // Per HDMRP, increment round number to trigger new RREQ round

    SRREQPkt->setRound(getRound());
    SRREQPkt->setMinor_round(getMinorRound());
    SRREQPkt->setPath_id(0);
    SRREQPkt->setNmas(0);
    SRREQPkt->setLen(0);
    SRREQPkt->setByteLength(7);
    setPath_filter(SRREQPkt, path_array);

    toMacLayer(SRREQPkt, BROADCAST_MAC_ADDRESS);

    trace()<<"Round #: "<<getRound();
}



void hdmrp::sendRREQ() {
    sendRREQ(getRound(),0,isMaster()?1:0,1);
}

void hdmrp::sendRREQ(int round, hdmrp_path path) {
    sendRREQ(round, path.path_id, path.nmas+isMaster()?1:0, path.len+1);
}

void hdmrp::sendRREQ(int round, int path_id, int nmas, int len) {
    trace()<<"sendRREQ() called";
    hdmrpPacket *RREQPkt=new hdmrpPacket("HDMRP RREQ packet", NETWORK_LAYER_PACKET);
    RREQPkt->setHdmrpPacketKind(hdmrpPacketDef::RREQ_PACKET);
    RREQPkt->setSource(SELF_NETWORK_ADDRESS);
    RREQPkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    RREQPkt->setRound(round);
    RREQPkt->setPath_id(path_id);
    RREQPkt->setNmas(nmas);
    RREQPkt->setLen(len);

    RREQPkt->setByteLength(7);

    toMacLayer(RREQPkt, BROADCAST_MAC_ADDRESS);

    trace()<<"Round #: "<<getRound();

}

void hdmrp::sendSRREQ() {
    trace()<<"sendSRRQ() called";
    hdmrpPacket *SRREQPkt=new hdmrpPacket("HDMRP Sink RREQ packet", NETWORK_LAYER_PACKET);
    SRREQPkt->setHdmrpPacketKind(hdmrpPacketDef::SINK_RREQ_PACKET);
    SRREQPkt->setSource(SELF_NETWORK_ADDRESS);
    SRREQPkt->setDestination(BROADCAST_NETWORK_ADDRESS);

    // Per HDMRP, increment round number to trigger new RREQ round
    newRound();

    SRREQPkt->setRound(getRound());
    SRREQPkt->setPath_id(0);
    SRREQPkt->setNmas(0);
    SRREQPkt->setLen(0);
    SRREQPkt->setByteLength(7);

    toMacLayer(SRREQPkt, BROADCAST_MAC_ADDRESS);

    trace()<<"Round #: "<<getRound();
}

void hdmrp::storeRREQ(hdmrpPacket *pkt) {
    trace()<<"New RREQ stored";
    hdmrp_path candidate_path;
    candidate_path.path_id=pkt->getPath_id();
    candidate_path.len=pkt->getLen();
    candidate_path.nmas=pkt->getNmas();
    candidate_path.next_hop=string(pkt->getSource());
    candidate_path.path_filter=getPath_filter_array(pkt);

    if(rreq_table.find(candidate_path.path_id) == rreq_table.end()) {
        rreq_table[candidate_path.path_id]=candidate_path;
    } else {
        if(calculateCost(candidate_path)<calculateCost(rreq_table[candidate_path.path_id])) {
                rreq_table[candidate_path.path_id]=candidate_path;
        }
    }
    return;
}

hdmrp_path hdmrp::selectRREQ() {
    std::random_device rd;
    uniform_int_distribution<int> dist(0, rreq_table.size()-1);
    auto it=rreq_table.begin();

    for(int i=dist(rd); i > 0 ; --i, ++it);
    trace()<<"RREQ table size: "<<rreq_table.size();
    return it->second;
}

void hdmrp::removeRREQ(hdmrp_path path) {
    rreq_table.erase(path.path_id);
}

void hdmrp::clearRREQ() {
    rreq_table.clear();
}

bool hdmrp::isRREQempty() const {
    return rreq_table.empty();
}

void hdmrp::confirmPaths() {
    for(auto path: routing_table) {
        trace()<<"Confirm path "<<path.second.path_id<<" to node "<<path.second.next_hop;
        hdmrpPacket *PathPkt=new hdmrpPacket("HDMRP PATH_CONFIRM Packet", NETWORK_LAYER_PACKET);
        PathPkt->setHdmrpPacketKind(hdmrpPacketDef::PATH_CONFIRM_PACKET);
        PathPkt->setSource(SELF_NETWORK_ADDRESS);
        PathPkt->setDestination(path.second.next_hop.c_str());
        PathPkt->setOrig(resolveNetworkAddress(path.second.next_hop.c_str()));
        PathPkt->setPath_id(path.second.path_id);
        PathPkt->setAck_req(true);
        PathPkt->setSequenceNumber(currentSequenceNumber++);
        PathPkt->setL_seq(d_pkt_seq);
        bufferForAck(PathPkt->dup());
        setTimer(d_pkt_seq,t_rsnd);
        incrementSeqNum();
        toMacLayer(PathPkt, resolveNetworkAddress(path.second.next_hop.c_str()));
    }
}

float hdmrp::calculateCost(hdmrp_path path) const {
    return (float)path.len/(float)path.nmas;
}

void hdmrp::addRoute(hdmrp_path path) {
    trace()<<"New route with Path ID: "<<path.path_id<<" Next hop: "<<path.next_hop;
    routing_table[path.path_id]=path;
}

bool hdmrp::RouteExists() const {
    return routing_table.size()>0;
}

void hdmrp::clearRoutes() {
    routing_table.clear();
}

void hdmrp::removeRoutes(vector<int> paths) {
    for(auto i: paths) {
        if(routing_table.find(i) != routing_table.end()) {
            routing_table.erase(i);
        }
    }
}

hdmrp_path hdmrp::getRoute(const int path_id) {
    if(!routing_table.size()) {
        throw std::length_error("Routing table empty");
    }
    if(routing_table.find(path_id) == routing_table.end()) {
        trace()<<"[Error] Path ID not available in routing table: "<<path_id;
        throw std::length_error("Path ID not available in routing table");
    }
    return routing_table.find(path_id)->second;
}

hdmrp_path hdmrp::getRoute() {
    if(!routing_table.size()) {
        throw std::length_error("Routing table empty");
    }
    std::random_device rd;
    uniform_int_distribution<int> dist(0, routing_table.size()-1);
    auto it=routing_table.begin();

    for(int i=dist(rd); i > 0 ; --i, ++it);
    trace()<<"Path selected: "<<it->second.path_id;
    return it->second;
}

set<int> hdmrp::getPaths() {
    std::set<int> ret;
    if(routing_table.size()>0) {
        for(auto it=routing_table.begin(); it != routing_table.end(); ++it) {
            ret.insert(it->first);
        }
    }
    return ret;
}

void hdmrp::bufferForAck(hdmrpPacket *pkt) {
    trace()<<"Buffered packet: "<<pkt->getL_seq()<<" Total of: "<<wf_ack_buffer.size();
    wf_ack_buffer[pkt->getL_seq()]=pkt;
}

bool hdmrp::bufferedPktExists(int index) {
    if(wf_ack_buffer.find(index) == wf_ack_buffer.end()) {
        return false;
    }
    return true;
}

hdmrpPacket* hdmrp::getBufferedPkt(int index) {
    if(!bufferedPktExists(index)) {
        throw cRuntimeError("Packet with l_seq %d is not buffered",index);
    }
    return wf_ack_buffer[index];
}

void hdmrp::removeBufferedPkt(int index) {
    if(!bufferedPktExists(index)) {
        throw cRuntimeError("Packet with l_seq %d is not buffered",index);
    }
    hdmrpPacket *orig_pkt=wf_ack_buffer[index];
    wf_ack_buffer.erase(index);
    delete orig_pkt;

}

void hdmrp::incrementSeqNum() {
   if(d_pkt_seq == TIMER_MAX_SIZE-1) {
        d_pkt_seq=hdmrpTimerDef::PACKET_TIMER_1;
    } else {
        ++d_pkt_seq;
    }
}

bool hdmrp::findPktHistEntry(pkt_hist_entry pkt_entry) {
    for(auto i: pkt_hist) {
        if(i.orig==pkt_entry.orig && i.seq == pkt_entry.seq && i.l_seq == pkt_entry.l_seq) {
            return true;
        }
    }
    return false;
}

void hdmrp::addPktHistEntry(pkt_hist_entry pkt_entry) {
    pkt_hist.push_back(pkt_entry);
}

void hdmrp::updatePktHistEntryRepCount(pkt_hist_entry pkt_entry) {
    for(auto &i: pkt_hist) {
        if(i.orig==pkt_entry.orig && i.seq == pkt_entry.seq && i.l_seq == pkt_entry.l_seq) {
            ++i.rep_count;
        }
    }
}

void hdmrp::sendAck(hdmrpPacket *orig_pkt, int dest) {
    hdmrpPacket *ack_pkt=new hdmrpPacket("HDMRP ACK packet", NETWORK_LAYER_PACKET);
    ack_pkt->setSource(SELF_NETWORK_ADDRESS);
    ack_pkt->setHdmrpPacketKind(hdmrpPacketDef::ACK_PACKET);
    char buffer[10];
    sprintf(buffer,"%d",dest);
    ack_pkt->setDestination(buffer);
    ack_pkt->setPath_id(orig_pkt->getPath_id());
    ack_pkt->setOrig(orig_pkt->getOrig());
    ack_pkt->setSequenceNumber(orig_pkt->getSequenceNumber());
    ack_pkt->setL_seq(orig_pkt->getL_seq());
    trace()<<"Constructing acknowledgement of packet with sequence number: "<<ack_pkt->getL_seq()<<" from Orig: "<<orig_pkt->getOrig();
    toMacLayer(ack_pkt, dest);
}

void hdmrp::sendPathFailure(int path) {
    hdmrpPacket *fail_pkt=new hdmrpPacket("HDMRP PATH_FAILURE packet",NETWORK_LAYER_PACKET);
    fail_pkt->setSource(SELF_NETWORK_ADDRESS);
    fail_pkt->setHdmrpPacketKind(hdmrpPacketDef::PATH_FAILURE_PACKET);
    fail_pkt->setOrig(resolveNetworkAddress(SELF_NETWORK_ADDRESS));
    fail_pkt->setSequenceNumber(d_pkt_seq);
    fail_pkt->setL_seq(d_pkt_seq);
    fail_pkt->setPath_id(0);
    fail_pkt->setAck_req(true);
    auto dest=getBackupDestination(path);
    char buffer[10];
    sprintf(buffer,"%d",dest);
    fail_pkt->setDestination(buffer);
    setPath_filter(fail_pkt,collectPath_filter());
    setTimer(d_pkt_seq,t_rsnd);
    fail_pkt->setRep_count(0);
    bufferForAck(fail_pkt->dup());
    incrementSeqNum();
    toMacLayer(fail_pkt,dest);
}

int hdmrp::getBackupDestination(int failing_path) {
    trace()<<"Find backup destination for failing path: "<<failing_path;
    neigh_entry n,m;
    n.rssi=-100.0;
    bool found=false;
    bool confirmed_found=false;

    if(neigh_list.size() == 0) {
        throw cRuntimeError("No neighbor");
    }
    for(auto i: neigh_list) {
        if(i.second.rssi>n.rssi) {
            if(std::find(i.second.paths.begin(),i.second.paths.end(),failing_path)==i.second.paths.end()) {
                n=i.second;
                found=true;
            } else {
                if(i.second.paths.size()>1) {
                    n=i.second;
                    found=true;
                }
            }
        }
        if(i.second.confirmed) {
            m=i.second;
            confirmed_found=true;
        }
    }
    if(found) {
        trace()<<"Backup destination: "<<n.address;
        return n.address;
    }
    if(confirmed_found) {
        trace()<<"Backup destination (backward): "<<m.address;
        return m.address;
    }
    throw cRuntimeError("No backup destination");

}

vector<int> hdmrp::collectPath_filter() {
    map<int,int> paths;
    for(auto it=neigh_list.begin() ; it!=neigh_list.end(); ++it) {
        for(auto i: it->second.paths) {
            paths[i]=i;
        }
    }
    vector<int> path_filter;
    for(auto i: paths) {
        path_filter.push_back(i.first);
    }
    return path_filter;
}


// Timer handling
void hdmrp::timerFiredCallback(int index) {
    switch (index) {
        case hdmrpTimerDef::NEW_ROUND:
        case hdmrpTimerDef::SINK_START: {
            trace()<<"SINK_START timer expired";
            sendSRREQ();
            setTimer(hdmrpTimerDef::NEW_ROUND,t_rreq);
            break;
        }
        case hdmrpTimerDef::T_L: {
            trace()<<"T_L expired";
            // select path
            auto path=selectRREQ();
            if(path.path_filter.size() > 0) {
                sendMinorRREQ(getRound(),getMinorRound(),path);
            } else {
                sendRREQ(getRound(),path);
                clearRoutes();
            }
            addRoute(path);
            if(isMaster()) {
                removeRREQ(path);
                if(isRREQempty()) {
                    confirmPaths();
                    setState(hdmrpStateDef::WORK);
                } else {
                    setState(hdmrpStateDef::RELAY);
                    setTimer(hdmrpTimerDef::T_RELAY,t_relay);
                }
            } else {
                while(!isRREQempty() && store_all_paths) {
                    addRoute(selectRREQ());
                }
                clearRREQ();
                confirmPaths();
                setState(hdmrpStateDef::WORK);
            }
            break;
        }
        case hdmrpTimerDef::T_RELAY: {
            trace()<<"T_RELAY expired";
            auto path=selectRREQ();
            if(path.path_filter.size() > 0) {
                sendMinorRREQ(getRound(),getMinorRound(),path);
            } else {
                sendRREQ(getRound(),path);
            }
            addRoute(path);
            removeRREQ(path);
            if(isRREQempty()) {
                confirmPaths();
                setState(hdmrpStateDef::WORK);
            } else {
                setState(hdmrpStateDef::RELAY);
                setTimer(hdmrpTimerDef::T_RELAY,t_relay);
            }
            break;
        }
        case hdmrpTimerDef::ACK_HIST_PURGE: {
            simtime_t now=getClock();
            for(auto it = pkt_hist.begin() ; it != pkt_hist.end() ;) {
                trace()<<"Orig: "<<it->orig<<" seq: "<<it->seq<<"l_seq: "<<it->l_seq;
                if(it->ts+static_cast<double>(t_pkt_hist) < now ) {
                    trace()<<"Erase orig: "<<it->orig<<" seq: "<<it->seq<<"l_seq: "<<it->l_seq;
                    it=pkt_hist.erase(it);
                } else {
                    ++it;
                }
            }
            setTimer(hdmrpTimerDef::ACK_HIST_PURGE,t_pkt_hist);
            break;
        }
        default: {
            trace()<<"Most probably packet timer: "<<index;
            hdmrpPacket *pkt=nullptr;

            try{
                pkt=getBufferedPkt(index);
            } catch (cRuntimeError &e) {
                trace()<<"[error]"<<e.what();
                break;
            
            }

            if(pkt->getRep_count() >= rep_limit) {
                trace()<<"Path failure";
                if(send_path_failure) {
                    sendPathFailure(pkt->getPath_id());
                    removeBufferedPkt(index);
                }
                break;
            } else {
               pkt->setRep_count(1+pkt->getRep_count());
            } 

            switch (pkt->getHdmrpPacketKind()) {
                case hdmrpPacketDef::DATA_PACKET: {
                    trace()<<"Resend data packet on path: "<<pkt->getPath_id()<<" with l_seq: "<<index;
                    hdmrp_path path;
                    if(isRoot()) {
                        path=getRoute(0);
                    } else {
                        try {
                        path=getRoute(pkt->getPath_id());
                        } catch (exception &e) {
                            trace()<<e.what();
                            break;
                        }
                    }
                    toMacLayer(pkt->dup(), resolveNetworkAddress(path.next_hop.c_str()));
                    collectOutput("Data packets","Forw",1);
                    collectOutput("Data packets","Repeat",1);
                    break;
                }
                case hdmrpPacketDef::PATH_CONFIRM_PACKET: {
                    trace()<<"Resend path confirm packet to: "<<pkt->getDestination()<<" with l_seq: "<<index;
                    toMacLayer(pkt->dup(),resolveNetworkAddress(pkt->getDestination()));
                    break;
                }
            }

            trace()<<"Start timer: "<<index;
            setTimer(index,t_rsnd); // should be a parameter
            break;
        }
    }
}


/* Application layer sends a packet together with a dest network layer address.
 * Network layer is responsible to route that packet by selecting an appropriate
 * MAC address. With BypassRouting we do not perform any routing function. We
 * just encapsulate the app packet and translate the net address to a MAC address
 * (e.g. "3" becomes 3, or a BROADCAST_NETWORK_ADDRESS becomes BROADCAST_MAC_ADDRESS)
 * If the destination is a 1-hop neighbor it will receive the packet.
 */
void hdmrp::fromApplicationLayer(cPacket * pkt, const char *destination) {
    hdmrp_path path;
    bool ack_req=false;


    if(0==std::strcmp(destination,BROADCAST_NETWORK_ADDRESS)) {
        hdmrpPacket *netPacket = new hdmrpPacket("HDMRP DATA packet", NETWORK_LAYER_PACKET);
        netPacket->setSource(SELF_NETWORK_ADDRESS);
        netPacket->setHdmrpPacketKind(hdmrpPacketDef::DATA_PACKET);
        netPacket->setDestination(BROADCAST_NETWORK_ADDRESS);
        encapsulatePacket(netPacket, pkt);
        toMacLayer(netPacket, BROADCAST_MAC_ADDRESS);
        return;
    }
    
    try {
        path=getRoute();
    } catch (std::length_error &e) {
        trace()<<"No route available: "<<e.what();
        return;
    }


    hdmrpPacket *netPacket = new hdmrpPacket("HDMRP DATA packet", NETWORK_LAYER_PACKET);
    ApplicationPacket *app_pkt=dynamic_cast<ApplicationPacket *>(pkt);
    if(0==std::strcmp(app_pkt->getApplicationID(),"ForestFire")) {
        ForestFirePacket *ff_pkt=dynamic_cast<ForestFirePacket *>(app_pkt);
        if(ForestFirePacketDef::PERIODIC_REPORT_PACKET == ff_pkt->getForestFirePacketKind()) {
            ack_req=true;
            trace()<<"Periodic report packet, ack required set";
        }
        if(ForestFirePacketDef::EVENT_REPORT_PACKET == ff_pkt->getForestFirePacketKind()) {
            ++event_pkt_counter;
            ack_req=0==event_pkt_counter%event_ack_req_period;
        }
    }

    netPacket->setSource(SELF_NETWORK_ADDRESS);
    netPacket->setHdmrpPacketKind(hdmrpPacketDef::DATA_PACKET);
    netPacket->setDestination(path.next_hop.c_str());
    netPacket->setPath_id(path.path_id);
    netPacket->setAck_req(ack_req);
    netPacket->setOrig(resolveNetworkAddress(SELF_NETWORK_ADDRESS)); 
    trace()<<"Destination "<<path.next_hop<<" Path: "<<path.path_id;
    encapsulatePacket(netPacket, pkt);
    // UGLY HACK, sequence number should not be handled like so, in two places
    netPacket->setSequenceNumber(d_pkt_seq);

    trace()<<"Sequence number: "<<netPacket->getSequenceNumber();
    if(ack_req) {
        netPacket->setRep_count(1);
        setTimer(d_pkt_seq,t_rsnd);
        netPacket->setL_seq(d_pkt_seq);
        bufferForAck(netPacket->dup());
        incrementSeqNum();
    }
    toMacLayer(netPacket, resolveNetworkAddress(path.next_hop.c_str()));


    ++sent_data_pkt;
    collectOutput("Data packets","Orig sent",1);

}

/* MAC layer sends a packet together with the source MAC address and other info.
 * With BypassMAC we just filter the packet by the NET address. If this
 * node is the right destination decapsulatePacket will extract the APP packet.
 * If not, there is no need to do anything. The whole net packet (including
 * the encapsulated apppacket will be deleted by the virtual routing code
 */
void hdmrp::fromMacLayer(cPacket * pkt, int srcMacAddress, double rssi, double lqi)
{
    hdmrpPacket *netPacket = dynamic_cast <hdmrpPacket*>(pkt);
  
    if (!netPacket) {
        trace()<<"dynamic_cast of packet failed";
        return;
    }

    trace()<<"RSSI: "<<rssi<<" LQI: "<<lqi<<" From: "<<srcMacAddress;
    if(neigh_list.find(srcMacAddress) != neigh_list.end()) {
        neigh_list[srcMacAddress].paths.push_back(netPacket->getPath_id());
    } else {
        neigh_list[srcMacAddress]={srcMacAddress,false,{netPacket->getPath_id()},rssi,lqi};
    }
    ++recv_pkt;
    s_rssi+=rssi;
    s_lqi+=lqi;


    switch(netPacket->getHdmrpPacketKind()) {
        case hdmrpPacketDef::PATH_CONFIRM_PACKET: {
            trace()<<"Path confirm received from: "<<netPacket->getSource()<<" for path: "<<netPacket->getPath_id();
            neigh_list[srcMacAddress].confirmed;
            if(netPacket->getAck_req()) {
                sendAck(netPacket,srcMacAddress);
                if(findPktHistEntry({netPacket->getOrig(),netPacket->getSequenceNumber(),netPacket->getL_seq(),0,0})) {
                    updatePktHistEntryRepCount({netPacket->getOrig(),netPacket->getSequenceNumber(),netPacket->getL_seq(),0,0});
                    trace()<<"Confirm Pkt already received";
                    break;
                } else {
                    addPktHistEntry({netPacket->getOrig(),netPacket->getSequenceNumber(),netPacket->getL_seq(),0,getClock()});
                }
            }
            break;
        }
        case hdmrpPacketDef::ACK_PACKET: {
            try {
                trace()<<"Ack packet received for Orig: "<<netPacket->getOrig()<<" L_seq: "<<netPacket->getL_seq();
                cancelTimer(netPacket->getL_seq());
                if(wf_ack_buffer.find(netPacket->getL_seq()) == wf_ack_buffer.end()) {
                    trace()<<"No such packet waiting for ack";
                    break;
                }
                removeBufferedPkt(netPacket->getL_seq());
                collectOutput("Data packets","Ack",1);

            } catch (cRuntimeError &e) {
                trace()<<"Error: "<<e.what();
            }
            break;
        }
        case hdmrpPacketDef::PATH_FAILURE_PACKET: {
            trace()<<"Failure packet seq: "<<netPacket->getSequenceNumber()<<" l_seq: "<<netPacket->getL_seq()<<" Orig: "<<netPacket->getOrig();
            if(netPacket->getAck_req()) {
                sendAck(netPacket,srcMacAddress);
                if(findPktHistEntry({netPacket->getOrig(),netPacket->getSequenceNumber(),netPacket->getL_seq(),0,0})) {
                    trace()<<"Pkt already received";
                    updatePktHistEntryRepCount({netPacket->getOrig(),netPacket->getSequenceNumber(),netPacket->getL_seq(),0,0});
                    break;
                } else {
//                    collectOutput("Data packets","Recv first",1);
                    addPktHistEntry({netPacket->getOrig(),netPacket->getSequenceNumber(),netPacket->getL_seq(),0,getClock()});
                }
            }
            if(0==strcmp(netPacket->getDestination(),SELF_NETWORK_ADDRESS)) {
                trace()<<"Path failure packet";
                hdmrp_path path;
                bool backward=false;
                int dest;
                if(isSink()) {
                    auto pf=getPath_filter_array(netPacket);
                    trace()<<"Path filter received: ";
                    for(auto p: pf) {
                        trace()<<"Path: "<<p;
                    }
                    setMinorRound(getMinorRound()+1);
                    sendMinorSRREQ(getPath_filter_array(netPacket));
                    break;
                }
                else if(isRoot()) {

                    path=getRoute(0);
                }
                else if(isSubRoot() || isNonRoot()) {
                    try {
                    if(0 != netPacket->getPath_id()) {
                        path=getRoute(netPacket->getPath_id());
                    } else {
                        path=getRoute();
                    } } catch (exception &e) {
                        trace()<<"FUUCK"<<e.what();
                        break;
                    }
                    // If the packet would go back
                    if(0==strcmp(path.next_hop.c_str(),netPacket->getSource())) {
                        dest=getBackupDestination(netPacket->getPath_id());
                        netPacket->setPath_id(0);
                        backward=true;
                    }
                }
                netPacket->setSource(SELF_NETWORK_ADDRESS);
                if(backward) {
                    char buffer[10];
                    sprintf(buffer,"%d",dest);
                    netPacket->setDestination(buffer);
                } else {
                    netPacket->setDestination(path.next_hop.c_str());
                }
                if(netPacket->getAck_req()) {
                    netPacket->setL_seq(d_pkt_seq);
                    netPacket->setRep_count(0);
                    bufferForAck(netPacket->dup());
                    trace()<<"t_rsnd started for l_seq: "<<d_pkt_seq;
                    setTimer(d_pkt_seq,t_rsnd);
                    incrementSeqNum();
                }
                toMacLayer(netPacket->dup(), backward?dest:resolveNetworkAddress(path.next_hop.c_str()));
            } else {
                trace()<<"Packet's destination network address does not match node's network address: "<<netPacket->getDestination()<<" "<<SELF_NETWORK_ADDRESS;
            }
            break;
        }
        case hdmrpPacketDef::DATA_PACKET: {
            collectOutput("Data packets","Recv",1);
            trace()<<"Packet seq: "<<netPacket->getSequenceNumber()<<" l_seq: "<<netPacket->getL_seq()<<" Orig: "<<netPacket->getOrig();

            if(netPacket->getAck_req()) {
                sendAck(netPacket,srcMacAddress);
                if(findPktHistEntry({netPacket->getOrig(),netPacket->getSequenceNumber(),netPacket->getL_seq(),0,0})) {
                    trace()<<"Pkt already received";
                    updatePktHistEntryRepCount({netPacket->getOrig(),netPacket->getSequenceNumber(),netPacket->getL_seq(),0,0});
                    break;
                } else {
                    collectOutput("Data packets","Recv first",1);
                    addPktHistEntry({netPacket->getOrig(),netPacket->getSequenceNumber(),netPacket->getL_seq(),0,getClock()});
                }
            }
            

            if(0==strcmp(netPacket->getSource(),SELF_NETWORK_ADDRESS)) {
                trace()<<"Data packet from the same node. Should not happen.";
            }
            else if(0==strcmp(netPacket->getSource(),BROADCAST_NETWORK_ADDRESS)) {
                trace()<<"Data packet with broadcast as source. Should not happen.";
            }
            else if(0==strcmp(netPacket->getDestination(),BROADCAST_NETWORK_ADDRESS)) {
                toApplicationLayer(decapsulatePacket(netPacket));
                trace()<<"Broadcast data packet. Forwarding to application layer.";
            }
            else if(0==strcmp(netPacket->getDestination(),SELF_NETWORK_ADDRESS)) {
                if(isSink()) {
                    trace()<<"Packet arrived on path: "<<netPacket->getPath_id()<<" From: "<<netPacket->getSource();
                    trace()<<"Orig source: "<<netPacket->getOrig()<<" Sequence number: "<<netPacket->getSequenceNumber();
toApplicationLayer(decapsulatePacket(netPacket));

                } else {
                    hdmrp_path path;
                    trace()<<"Packet received, routing forward.";
                    if(isRoot()) {
                        try {
                            path=getRoute(0);
                        } catch(std::length_error &e) {
                            trace()<<"Error: "<<e.what();
                            return;
                        }
                        trace()<<"Root|Path: "<<netPacket->getPath_id()<<"|Next hop: "<<path.next_hop<<"|Seq: "<<netPacket->getSequenceNumber();
                    }
                    else if(isSubRoot() || isNonRoot()) {
                        try {
                            path=getRoute(netPacket->getPath_id());
                        } catch(std::length_error &e) {
                            trace()<<"Error: "<<e.what();
                            return;
                        }
                        trace()<<(isSubRoot()?"SubRoot|":"NonRoot|")<<"Path: "<<netPacket->getPath_id()<<" Next hop: "<<path.next_hop<<"|Seq: "<<netPacket->getSequenceNumber();

                    }
                    netPacket->setSource(SELF_NETWORK_ADDRESS);
                    netPacket->setDestination(path.next_hop.c_str());
                    if(netPacket->getAck_req()) {
                        netPacket->setL_seq(d_pkt_seq);
                        netPacket->setRep_count(0);
                        bufferForAck(netPacket->dup());
                        trace()<<"t_rsnd started for l_seq: "<<d_pkt_seq;
                        setTimer(d_pkt_seq,t_rsnd); // should be a parameter
                        incrementSeqNum();
                    }

                    toMacLayer(netPacket->dup(), resolveNetworkAddress(path.next_hop.c_str()));
                    collectOutput("Data packets","Forw first",1);
                    collectOutput("Data packets","Forw",1);

                }
            } else {
                trace()<<"Wrong address: "<<netPacket->getDestination();
            }
            break;
        }
        case hdmrpPacketDef::RREQ_PACKET: {
            if(rssi<min_rreq_rssi) {
                break;
            }
            if(isSink()) {
                trace()<<"RREQ discarded by sink";
            }
            else if(isRoot()) {
                trace()<<"RREQ discarded by root";
            }
            else if(isSubRoot()) {
                trace()<<"RREQ received by sub-root";
                if(0 == netPacket->getPath_id()) {
                    trace()<<"RREQ with 0 Path_id";
                    // start guard timer, send message with RREQ, learn next hop?
                    if(isNewRound(netPacket)) {
                       trace()<<"New Round";
                       setRound(netPacket->getRound());
                       clearRREQ();
                       clearRoutes();
                       addRoute(hdmrp_path{resolveNetworkAddress(SELF_NETWORK_ADDRESS), string(netPacket->getSource()), 0, 0}  );
                       sendRREQ(getRound(),getRoute());
                    }
                    else if(isSameRound(netPacket) && isNewMinorRound(netPacket) && matchPathFilter(netPacket)) {
                        trace()<<"New minor round";
                        setMinorRound(netPacket->getMinor_round());
                        clearRREQ();
                        clearRoutes();
                        addRoute(hdmrp_path{resolveNetworkAddress(SELF_NETWORK_ADDRESS), string(netPacket->getSource()), 0, 0}  );
                        sendMinorRREQ(getRound(),getMinorRound(),getRoute(),getPath_filter_array(netPacket));
                    } 
                     else {
                        trace()<<"Old round. Current round: "<<getRound()<<" Received round: "<<netPacket->getRound();
                    }
                } else {
                    trace()<<"RREQ with valid Path_id discarded by sub-root";
                }
            }
            else if(isNonRoot()) {
                trace()<<"RREQ received by non-root";
                if(isWorkingState()) {
                    if(0 != netPacket->getPath_id() ) {
                        if(isNewRound(netPacket)) {
                            trace()<<"New round";
                            setState(hdmrpStateDef::LEARN);
                            setRound(netPacket->getRound());
                            setTimer(hdmrpTimerDef::T_L,t_l);
                            clearRREQ();
                            storeRREQ(netPacket);
                        }
                        else if(isSameRound(netPacket) && isNewMinorRound(netPacket) && matchPathFilter(netPacket)) {
                            trace()<<"New minor round";
                            setState(hdmrpStateDef::LEARN);
                            setMinorRound(netPacket->getMinor_round());
                            setTimer(hdmrpTimerDef::T_L,t_l);
                            clearRREQ();
                            removeRoutes(getPath_filter_array(netPacket));
                            storeRREQ(netPacket);
                        }
                        else {
                            trace()<<"Old round. Current round: "<<getRound()<<" Received round: "<<netPacket->getRound();
                        }
                    } else {
                        // once switched to sub-root, some guard timer is needed??
                        trace()<<"RREQ with 0 Path_id received";
                        if(netPacket->getRound() > getRound()) {
                            trace()<<"New Round";
                            setRole(hdmrpRoleDef::SUB_ROOT);
                            setRound(netPacket->getRound());
                            clearRoutes();
                            clearRREQ();
                            addRoute( hdmrp_path{resolveNetworkAddress(SELF_NETWORK_ADDRESS),
                                                 string(netPacket->getSource()),
                                                 0,
                                                 0} );
                            sendRREQ(getRound(),getRoute());
                        } else {
                            trace()<<"Old round. Current round: "<<getRound()<<" Received round: "<<netPacket->getRound();
                        }
                    }
                }
                else if(isLearningState()) { // What about new rounds?
                    if(0 != netPacket->getPath_id()) {
                        if(netPacket->getRound() == getRound() && netPacket->getMinor_round() == getMinorRound() ) {
                            storeRREQ(netPacket);
                        } else {
                            trace()<<"RREQ received with invalid round during learning state";
                        }
                    } else {
                        trace()<<"RREQ with 0 Path_id discarded during learning state";
                    }
                }
            }
            break;
        }
        case hdmrpPacketDef::SINK_RREQ_PACKET: {
            if(rssi<min_rreq_rssi) {
                break;
            }
            if(isSink()) {
                trace()<<"SRREQ discarded by sink";
            }
            else if(isNonRoot() || isRoot()) {
                trace()<<"SRREQ received by non-root or root";
                if(isNonRoot()) {
                    if(getTimer(hdmrpTimerDef::T_L) != -1) {
                        cancelTimer(hdmrpTimerDef::T_L);
                    }
                    setRole(hdmrpRoleDef::ROOT);
                }

                if(netPacket->getRound() > getRound()) {
                    trace()<<"Old round: "<<getRound()<<" new round: "<<netPacket->getRound();
                    trace()<<"Source: "<<netPacket->getSource();
                    setRound(netPacket->getRound());
                    sendRREQ();
                    clearRREQ();
                    clearRoutes();
                    addRoute(hdmrp_path{0, string(netPacket->getSource()),0,0});
                }
                else if(isSameRound(netPacket) && isNewMinorRound(netPacket)) {
                    trace()<<"New minor round";
                    setMinorRound(netPacket->getMinor_round());
                    sendMinorRREQ(getPath_filter_array(netPacket));
                    clearRREQ();
                    clearRoutes();
                    addRoute(hdmrp_path{0, string(netPacket->getSource()),0,0});

                } else {
                    trace()<<"Outdated SRREQ. Current round: "<<getRound()<<" Received round: "<<netPacket->getRound();
                }
            }
            else if(isSubRoot()) {
                trace()<<"SRREQ received by sub-root";
            }
            break;
        }
    }
}

void hdmrp::finishSpecific() {
    declareOutput("Paths");
    collectOutput("Paths","",routing_table.size());

    declareOutput("Role");
    collectOutput("Role","",role);



    // Only the sink calculates individual paths
    if (getParentModule()->getIndex() == 0) {
        declareOutput("Number of Paths");

        cTopology *topo;        // temp variable to access energy spent by other nodes
        topo = new cTopology("topo");
        topo->extractByNedTypeName(cStringTokenizer("node.Node").asVector());
        
        set<int> paths;

        for (int i = 1; i < topo->getNumNodes(); ++i) {
            hdmrp *hdmrp_instance = dynamic_cast<hdmrp*>
                (topo->getNode(i)->getModule()->getSubmodule("Communication")->getSubmodule("Routing"));
            set<int> tmp_paths=hdmrp_instance->getPaths();
            trace()<<"Node: "<<i;
            trace()<<"Number of paths at node: "<<tmp_paths.size();
            for(auto it=tmp_paths.begin() ; it != tmp_paths.end() ; ++it) {
                if(paths.find(*it)==paths.end()) {
                    paths.insert(*it);
                    trace()<<"Path: "<<*it;
                }
            }
        }
        
        delete(topo);
        collectOutput("Number of Paths", "", paths.size());
    }
    declareOutput("Recv Pkt ind");
    collectOutput("Recv Pkt ind","RSSI",0==recv_pkt?0.0:s_rssi/static_cast<double>(recv_pkt) );
    collectOutput("Recv Pkt ind","LQI", 0==recv_pkt?0.0:s_lqi/static_cast<double>(recv_pkt) );

    declareOutput("Constructed paths");
    for(auto path: routing_table) {
        collectOutput("Constructed paths", path.second.path_id,path.second.next_hop.c_str());
    }
    return;
}
