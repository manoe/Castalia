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

    if(0==strcmp(appModule->getClassName(),"ForesFire")) {
        trace()<<"[info] ForestFire app present";
        ff_app = check_and_cast<ForestFire *>(appModule);
    } else {
        trace()<<"[info] ForestFire app not present";
        ff_app = NULL;
    }


    if(appModule->hasPar("isMaster")) {
        g_is_master=appModule->par("isMaster");
    } else {
        g_is_master=false;
    }

    if(hasPar("sink_address")) {
        g_sink_addr.assign(par("sink_address").stringValue());
    } else {
        trace()<<"[error] No sink address provisioned";
    }

    fp.t_l               = par("t_l");
    fp.ring_radius       = par("ring_radius");
    fp.t_est             = par("t_est");
    fp.t_meas            = par("t_meas");
    fp.rresp_req         = par("f_rresp_required");
    fp.rst_learn         = par("f_restart_learning");
    fp.replay_rinv       = par("f_replay_rinv");
    fp.cost_func         = strToCostFunc(par("f_cost_function").stringValue());
    fp.cost_func_epsilon = par("f_cost_func_epsilon");
    fp.cost_func_iota    = par("f_cost_func_iota");
    fp.cost_func_pi      = par("f_cost_func_pi");
    fp.cost_func_phi     = par("f_cost_func_phi");
    fp.cost_func_mu      = par("f_cost_func_mu");
    fp.cost_func_eta     = par("f_cost_func_eta");
    fp.cf_after_rresp    = par("f_cf_after_rresp");
    fp.random_t_l        = par("f_random_t_l");
    fp.random_t_l_sigma  = par("f_random_t_l_sigma");
    fp.rinv_tbl_admin    = strToRinvTblAdmin(par("f_rinv_table_admin").stringValue());
    fp.interf_ping       = par("f_interf_ping");
    fp.round_keep_pong   = par("f_round_keep_pong");
    fp.rand_ring_hop     = par("f_rand_ring_hop");
    fp.static_routing    = par("f_static_routing");
    fp.measure_w_rreq    = par("f_measure_w_rreq");
    fp.meas_rreq_count   = par("f_meas_rreq_count");
    fp.calc_max_hop      = par("f_calc_max_hop");
    fp.qos_pdr           = par("f_qos_pdr");
    fp.rt_recalc_warn    = par("f_rt_recalc_warn");
    fp.reroute_pkt       = par("f_reroute_pkt");
    fp.second_learn      = strToSecLPar(par("f_second_learn").stringValue());
    fp.t_sec_l           = par("f_t_sec_l");
    fp.t_sec_l_repeat    = par("f_t_sec_l_repeat");
    fp.t_sec_l_timeout   = par("f_t_sec_l_timeout");
    fp.t_sec_l_start     = par("f_t_sec_l_start");
    fp.detect_link_fail  = par("f_detect_link_fail");
    fp.rt_fallb_wo_qos   = par("f_rt_fallb_wo_qos");
    fp.send_pfail_rwarn  = par("f_send_pfail_rwarn");
    fp.fail_count        = par("f_fail_count");
    fp.path_sel          = par("f_path_sel");
    fp.e2e_qos_pdr       = par("f_e2e_qos_pdr");
    fp.t_send_pkt        = par("f_t_send_pkt");
    fp.rep_m_pdr         = par("f_rep_m_pdr");
    fp.drop_1st_rt_c     = par("f_drop_1st_rt_c");
    fp.drop_prob         = par("f_drop_prob");
    fp.e2e_cost          = par("f_e2e_cost");

    if(fp.static_routing) {
        parseRouting(par("f_routing_file").stringValue());
        setState(shmrpStateDef::WORK);
        setHop(0); // probably wrong
    } else {
        if(isSink()) {
            setHop(0);
            initPongTableSize();
            setTimer(shmrpTimerDef::SINK_START,par("t_start"));
            setState(shmrpStateDef::WORK);
            if(fp.second_learn != shmrpSecLParDef::OFF) {
                setTimer(shmrpTimerDef::T_SEC_L_START,fp.t_sec_l_start);
            }
        } else {
            setHop(std::numeric_limits<int>::max());
            setState(shmrpStateDef::INIT);
        }
    }
    setRound(0);
}

void shmrp::parseRouting(std::string file) {
    YAML::Node nodes;

    try {
        nodes = YAML::LoadFile(file);
    }
    catch (std::exception &e) {
        trace()<<e.what();
        throw e;
    }
    if(!nodes.IsSequence()) {
        throw std::runtime_error("[error] Routing file - format error - node sequence missing");
    }
    for(auto i = 0 ; i < nodes.size() ; ++i) {
        if(0 == std::strcmp(nodes[i]["node"].as<std::string>().c_str(), SELF_NETWORK_ADDRESS)) {
            if(!nodes[i]["routes"].IsSequence()) {
                throw std::runtime_error("[error] Routing file - format error - route sequence missing");
            }
            for(auto j = 0 ; j < nodes[i]["routes"].size() ; ++j) {
                trace()<<"[info] "<<nodes[i]["routes"][j]["node"].as<std::string>()<<" node via pathid "<<nodes[i]["routes"][j]["pathid"].as<int>();
                addRoute(nodes[i]["routes"][j]["node"].as<std::string>(), nodes[i]["routes"][j]["pathid"].as<int>());
            }
        } 
    }
}

bool shmrp::isSink() const {
    return g_is_sink;
}

bool shmrp::isMaster() const {
    return g_is_master;
}

bool shmrp::getSecL() {
    trace()<<"[info] Entering getSecL()";
    for(auto ne: routing_table) {
        for(auto p: ne.second.pathid) {
            if(true==p.secl_performed) {
                return true;
            }
        }
    }
    return false;
}

bool shmrp::getSecL(int pathid) {
    trace()<<"[info] Entering getSecL(pathid="<<pathid<<")";
    for(auto ne: routing_table) {
        for(auto p: ne.second.pathid) {
            if(p.pathid == pathid && p.secl_performed == true) {
                return true;
            }
        }
    }
    return false;
}


void shmrp::setSecL(bool flag) {
    trace()<<"[info] Entering setSecL(flag="<<flag<<")";
    if(false==flag) {
        for(auto &&i: routing_table) {
            for(auto &&p: i.second.pathid) {
                p.secl_performed=false;
            }
        }
    } else {
        trace()<<"[info] set to true not implemented";
    }
}

void shmrp::setSecL(int pathid, bool flag) {
    trace()<<"Entering setSecL(pathid="<<pathid<<", flag="<<flag<<")";
    for(auto &&ne: routing_table) {
        for(auto &&p: ne.second.pathid) {
            if(p.pathid == pathid) {
                p.secl_performed = true;
            }
        }
    }
}

void shmrp::pushSecLPathid(int pathid) {
    trace()<<"Entering pushSecLPathid(pathid="<<pathid<<")";
    g_sec_l_pathid.push(pathid);
}

int  shmrp::popSecLPathid() {
   trace()<<"Entering popSecLPathid()";
   auto ret_val=g_sec_l_pathid.front();
   g_sec_l_pathid.pop();
   return ret_val;
}

int  shmrp::getSecLPathid() {
    return g_sec_l_pathid.front();

}
bool shmrp::isSecLPathidEmpty() {
    return g_sec_l_pathid.empty();
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

void shmrp::handleTSecLTimer() {
    trace()<<"[info] Entering handleTSecLTimer()";
    if(fp.second_learn != shmrpSecLParDef::OFF) {
        trace()<<"[info] Second learn active";
        if(getTimer(shmrpTimerDef::T_SEC_L) != -1) {
            trace()<<"[info] T_SEC_L timer active, restarting";
            cancelTimer(shmrpTimerDef::T_SEC_L);
            setTimer(shmrpTimerDef::T_SEC_L,fp.t_sec_l);
        }
    }
}


shmrpCostFuncDef shmrp::strToCostFunc(string str) const {
    if("hop" == str) {
        return shmrpCostFuncDef::HOP;
    } else if("hop_and_interf" == str) {
        return shmrpCostFuncDef::HOP_AND_INTERF;
    } else if("hop_emerg_and_interf" == str) {
        return shmrpCostFuncDef::HOP_EMERG_AND_INTERF;
    } else if("hop_and_pdr" == str) {
        return shmrpCostFuncDef::HOP_AND_PDR;
    } else if("hop_pdr_and_interf" == str) {
        return shmrpCostFuncDef::HOP_PDR_AND_INTERF;
    } else if("hop_emerg_pdr_and_interf" == str) {
        return shmrpCostFuncDef::HOP_EMERG_PDR_AND_INTERF;
    } else if("hop_enrgy_emerg_pdr_and_interf" == str) {
        return shmrpCostFuncDef::HOP_ENRGY_EMERG_PDR_AND_INTERF;
    } else if("hop_enrgy_emerg_and_pdr" == str) {
        return shmrpCostFuncDef::HOP_ENRGY_EMERG_AND_PDR;
    } else if("hop_enrgy_and_pdr" == str) {
        return shmrpCostFuncDef::HOP_ENRGY_AND_PDR;
    } else if("xpr_interf" == str) {
        return shmrpCostFuncDef::XPR_INTERF;
    } else if("xpr_hop_and_pdr" == str) {
        return shmrpCostFuncDef::XPR_HOP_AND_PDR;
    } else if("xpr_hop_pdr_and_interf" == str) {
        return shmrpCostFuncDef::XPR_HOP_PDR_AND_INTERF;
    } else if("sum_hop_enrgy_emerg_pdr_and_interf" == str) {
        return shmrpCostFuncDef::SUM_HOP_ENERGY_EMERG_PDR_AND_INTERF;
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

shmrpSecLParDef shmrp::strToSecLPar(std::string str) const {
    if("off" == str) {
        return shmrpSecLParDef::OFF;
    } else if("broadcast" == str) {
        return shmrpSecLParDef::BROADCAST;
    } else if("unicast" == str) {
        return shmrpSecLParDef::UNICAST;
    }
    throw std::invalid_argument("[error] Unknown second_learn parameter");
    return shmrpSecLParDef::OFF;
}


void shmrp::setHop(int hop) {
    trace()<<"[info] Entering shmrp::setHop(hop="<<hop<<")";  
    trace()<<"[info] Update hop "<<g_hop<<" to "<<hop;
    g_hop=hop;
}

int shmrp::getHop() const {
    return g_hop;
}

int shmrp::getHop(int pathid) {
    trace()<<"[info] Entering shmrp::getHop(pathid="<<pathid<<")";
    int hop=getHop();
    bool fail=true;
    for(auto ne: routing_table) {
        for(auto p: ne.second.pathid) {
            if(p.pathid==pathid) {
                hop = ne.second.hop;
                fail=false;
            }
        }
    }
    if(fail) {
        throw no_available_entry("No route available for the path"); 
    }
    trace()<<"[info] Hop count for pathid "<<pathid<<" is "<<hop;
    return hop;
}

int shmrp::calculateHop(bool max_hop=false) {
    trace()<<"[info] Entering shmrp::calculateHop(max_hop="<<max_hop<<")";
    if(rinv_table.empty()) {
        throw rinv_table_empty("[error] RINV table empty");
    }

    if(std::all_of(rinv_table.begin(), rinv_table.end(),[](std::pair<std::string,node_entry> ne){return ne.second.used; } )) {
        throw no_available_entry("[error] All RINV table entry used");
    }

    int hop=std::numeric_limits<int>::max();
    bool (*gt_lt)(int,int)=[](int lh, int rh) { return lh > rh; };

    if(max_hop) {
        hop=std::numeric_limits<int>::min();
        gt_lt=[](int lh, int rh) { return lh < rh; };
    }

    for(auto ne: rinv_table) {
        trace()<<"[info] Hop level for node "<<ne.second.nw_address<<" is "<<ne.second.hop;
        if(!ne.second.used && gt_lt(hop,ne.second.hop)) {
            hop=ne.second.hop;
        }
    }
    return hop+1;
}

int shmrp::calculateHopFromRoutingTable() {
    trace()<<"[info] Entering shmrp::calculateHopFromRoutingTable()";
    int hop=std::numeric_limits<int>::max();
    for(auto i: routing_table) {
        if(i.second.hop < hop) {
            hop=i.second.hop;
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
    cTopology *topo = new cTopology("topo");
    topo->extractByNedTypeName(cStringTokenizer("node.Node").asVector());
    auto *shmrp_instance = dynamic_cast<shmrp*>
                (topo->getNode(atoi(getSinkAddress().c_str()))->getModule()->getSubmodule("Communication")->getSubmodule("Routing"));
    auto *res_mgr = dynamic_cast<ResourceManager *>(getParentModule()->getParentModule()->getSubmodule("ResourceManager"));
    shmrp_instance->writeState(atoi(SELF_NETWORK_ADDRESS), simTime().dbl(), state, res_mgr->getSpentEnergy());
    delete topo;

    g_state=state;
}

void shmrp::writeState(int node, double timestamp, shmrpStateDef state, double energy) {
    state_chng_log.push_back({node,timestamp,state,energy});
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
        case shmrpStateDef::LOCAL_LEARN: {
            return "LOCAL_LEARN";
        }
        case shmrpStateDef::DEAD: {
            return "DEAD";
        }
        case shmrpStateDef::S_ESTABLISH: {
            return "S_ESTABLISH";
        }
    }
    return "UNKNOWN";

}

shmrpStateDef shmrp::getState() const {
    if(disabled) {
        return shmrpStateDef::DEAD;
    }
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
    pong_table.insert({pong_pkt->getSource(),{pong_pkt->getSource(),std::vector<pathid_entry>(1,{0,0}),0,false,0,0,false,pong_pkt->getRound()}});
}

void shmrp::clearPongTable() {
    pong_table.clear();
}

void shmrp::clearPongTable(int round) {
    trace()<<"[info] Entering shmrp::clearPongTable(round="<<round<<")";
    if(pong_table.empty()) {
        return;
    }
    for(auto it=pong_table.begin() ; it != pong_table.end() ; ++it) {
        if(it->second.round < round ) {
            pong_table.erase(it);
        }
    }
}

int shmrp::getPongTableSize() const {
    return pong_table.size();
}

void shmrp::initPongTableSize() {
    trace()<<"Entering setPongTableSize()";
    node_entry ne;
    ne.nw_address=SELF_NETWORK_ADDRESS;
//    pong_table.insert({std::string(SELF_NETWORK_ADDRESS),ne});
}


void shmrp::sendRinv(int round, std::vector<pathid_entry> pathid, bool local=false, int local_id=0, int nmas=0) {
    trace()<<"[info] Entering shmrp::sendRinv(round = "<<round<<", pathid = "<<pathidToStr(pathid)<<")";
    shmrpRinvPacket *rinv_pkt=new shmrpRinvPacket("SHMRP RINV packet", NETWORK_LAYER_PACKET);
    rinv_pkt->setByteLength(netDataFrameOverhead);
    rinv_pkt->setShmrpPacketKind(shmrpPacketDef::RINV_PACKET);
    rinv_pkt->setSource(SELF_NETWORK_ADDRESS);
    rinv_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    rinv_pkt->setRound(round);
    rinv_pkt->setPathidArraySize(pathid.size());
    for(int i=0 ; i < pathid.size() ; ++i) {
        shmrpPathDef p_id;
        p_id.pathid = pathid[i].pathid;
        p_id.nmas   = pathid[i].nmas;
        p_id.enrgy  = pathid[i].enrgy;
        p_id.emerg  = pathid[i].emerg;
        p_id.pdr    = pathid[i].pdr;
        rinv_pkt->setPathid(i, p_id);
    }
    try {
        // This is wrong, hop should be moved to pathid entry, but whatever...
        rinv_pkt->setHop(getHop(pathid[0].pathid)+1);
    } catch (exception &e) {
        trace()<<"[info] "<<e.what()<<" fall back to normal hop";
        rinv_pkt->setHop(getHop());
    }
    rinv_pkt->setInterf(getPongTableSize());
    rinv_pkt->setNmas(nmas);
    rinv_pkt->setLocal(local);
    rinv_pkt->setLocalid(local_id);
    rinv_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(rinv_pkt, BROADCAST_MAC_ADDRESS);
}

void shmrp::sendRinv(int round, bool local=false, int localid=0, int nmas=0) {
    trace()<<"[info] Entering shmrp::sendRinv(round = "<<round<<")";
    std::vector<pathid_entry> pathid;
    pathid_entry pe;
    pe.pathid         = 0;
    pe.nmas           = 0;
    pe.secl           = 0;
    pe.secl_performed = 0;
    pe.used           = 0;
    pe.enrgy          = getEnergyValue(); 
    pe.emerg          = getEmergencyValue();
    pe.pdr            = 1.0;
    pathid.push_back(pe);
    sendRinv(round,pathid,local,localid,nmas);
}

void shmrp::sendRinvBasedOnHop(bool local=false, int localid=0, int nmas=0) {
    trace()<<"[info] Entering sendRinvBasedOnHop()";
    if(getHop() < fp.ring_radius) {
        trace()<<"[info] Node inside mesh ring";
        sendRinv(getRound(), local, localid, nmas);
    } else if(getHop() == fp.ring_radius) {
        trace()<<"[info] Node at mesh ring border";
        // With this the master/sensor node capabilities inside the ring won't matter
        sendRinv(getRound(), std::vector<pathid_entry>(1,{resolveNetworkAddress(SELF_NETWORK_ADDRESS),static_cast<int>(isMaster()),false,false,false,  getEnergyValue(), getEmergencyValue(),1.0}), local, nmas);
    } else {
        trace()<<"[info] Node outside mesh ring";
        std::vector<pathid_entry> pathid;
        try {
            if(isMaster()) {
                trace()<<"[info] Node is master node, selecting all pathids";
                trace()<<"[info] Master hack, let's consume 1 random number, to keep simulation in-sync: "<<getRNG(0)->intRand(pathid.size());
                for(auto ne: routing_table) {
                    for(auto p: ne.second.pathid) {
                        bool found=false;
                        for(auto pp: pathid) {
                            if(p.pathid==pp.pathid) {
                                found=true;
                            }
                        }
                        if(!found) {
                            pathid_entry pe;
                            pe.pathid = p.pathid;
                            pe.nmas   = p.nmas+1;
                            pe.enrgy  = getEnergyValue(); // select min
                            pe.emerg  = getEmergencyValue(); // select min
                            pe.pdr    = static_cast<double>(rreq_table[ne.first].ack_count)/static_cast<double>(rreq_table[ne.first].pkt_count);

                            pathid.push_back(pe);
                        }
                    }
                }
            } else {
                trace()<<"[info] Node is sensor node, selecting random pathid";
                auto pe = selectPathid(false);
                pe.enrgy = getEnergyValue();
                pe.emerg = getEmergencyValue();

                // PDR not working yet.
                pe.pdr    = 0;


                pathid.push_back(pe);
            }
        } catch (std::exception &e) {
            trace()<<e.what();
            throw e;
        }
        sendRinv(getRound(), pathid, local, localid,nmas);
    }
}


void shmrp::clearRinvTable() {
    trace()<<"[info] RINV table erased";
    rinv_table.clear();
}

void shmrp::addToRinvTable(shmrpRinvPacket *rinv_pkt) {
    trace()<<"[info] Add entry to RINV table - source: "<<rinv_pkt->getSource()<<" pathid size: "<<rinv_pkt->getPathidArraySize()<<" hop: "<<rinv_pkt->getHop()<<" interf: "<<rinv_pkt->getInterf();
    node_entry ne;
    ne.nw_address = std::string(rinv_pkt->getSource());
    ne.pathid.clear(); 
    for(int i=0 ; i < rinv_pkt->getPathidArraySize() ; ++i) {
        pathid_entry pe;
        pe.pathid         = rinv_pkt->getPathid(i).pathid;
        pe.nmas           = rinv_pkt->getPathid(i).nmas;
        pe.secl           = false;
        pe.secl_performed = false;
        pe.used           = false;
        pe.enrgy          = rinv_pkt->getPathid(i).enrgy;
        pe.emerg          = rinv_pkt->getPathid(i).emerg;
        pe.pdr            = rinv_pkt->getPathid(i).pdr;
        ne.pathid.push_back(pe);
    }
    trace()<<"[info] Pathid added: "<<pathidToStr(ne.pathid);
    ne.hop = rinv_pkt->getHop();
    ne.interf = rinv_pkt->getInterf();
    ne.nmas = rinv_pkt->getNmas();
    ne.used = false;
    if(rinv_table.find(ne.nw_address) != rinv_table.end()) {
        trace()<<"[info] Entry already exists, overriding";

        bool used = rinv_table.find(ne.nw_address)->second.used;
        ne.used=used;
        rinv_table[ne.nw_address]=ne;
    } else {
        trace()<<"[info] Adding new entry";
        rinv_table[ne.nw_address]=ne;
    }
}

int shmrp::getRinvTableSize() const {
    return rinv_table.size();
}

void shmrp::clearRinvTableLocalFlags() {
    trace()<<"[info] entering clearRinvTableLocalFlags()";
    for(auto i=rinv_table.begin() ; i != rinv_table.end() ; ++i) {
        i->second.local=false;
    }
}

void shmrp::removeRinvEntry(std::string ne) {
    trace()<<"[info] Entering removeRinvEntry(ne="<<ne<<")";
    rinv_table.erase(ne);
}


void shmrp::markRinvEntryLocal(std::string id) {
    trace()<<"[info] Entering markRinvEntryLocal("<<id<<")";
    if(rinv_table.find(id) == rinv_table.end()) {
        throw no_available_entry("[error] No entry available in RINV table");
    }
    rinv_table[id].local = true;
}

void shmrp::markRinvEntryFail(std::string id) {
    trace()<<"[info] Entering markRinvEntryLocal(id="<<id<<")";
    if(!checkRinvEntry(id)) {
        throw no_available_entry("[error] No entry available in RINV table");
    }
    rinv_table[id].fail=true; 
}

void shmrp::clearRreqTable() {
    trace()<<"[info] RREQ table erased";
    rreq_table.clear();
}

void shmrp::saveRreqTable() {
    trace()<<"[info] Entering saveRreqTable()";
    backup_rreq_table=rreq_table;
    for(auto ne: rreq_table) {
        trace()<<"[info] ne: "<<ne.first<<" pathid: "<<pathidToStr(ne.second.pathid);
    }
}

void shmrp::retrieveRreqTable() {
    trace()<<"[info] Entering retrieveRreqTable()";
    rreq_table=backup_rreq_table;
}

// Merge pathid array...
void shmrp::retrieveAndMergeRreqTable() {
    trace()<<"[info] Entering retrieveAndMergeRreqTable()";
    for(auto ne: backup_rreq_table) {
        if(rreq_table.find(ne.first) != rreq_table.end()) {
            trace()<<"[warn] Duplicate record:"<<ne.first<<" merging pathid "<<pathidToStr(ne.second.pathid)<<" with "<<pathidToStr(rreq_table[ne.first].pathid);
            mergePathids(rreq_table[ne.first].pathid,ne.second.pathid);
        } else {
            trace()<<"[info] Merging element: "<<ne.second.nw_address<<" with pathid: "<<pathidToStr(ne.second.pathid)<<" to RREQ table";
            rreq_table[ne.first]=ne.second;
        }
    }
}

void shmrp::mergePathids(std::vector<pathid_entry> &p1, std::vector<pathid_entry> &p2) {
    trace()<<"[info] Entering mergePathids(p1="<<pathidToStr(p1)<<", p2="<<pathidToStr(p2)<<")";
    for(auto pe: p1) {
        for(auto pe2: p2) {
            if(pe.pathid == pe2.pathid) {
                trace()<<"[error] Matching pathid entries";
                throw state_not_permitted("[error] Matching pathid entries");
            } 
        }
    }
    p1.insert( p1.end(), p2.begin(), p2.end() );
}

bool shmrp::isRreqTableEmpty() const {
    return rreq_table.empty();
}

double shmrp::calculateCostFunction(node_entry ne) {
    double ret_val;

    // FIXME
    if(fp.e2e_cost) {
        ne.pathid[0].pdr;
    }

    switch (fp.cost_func) {
        case shmrpCostFuncDef::HOP: {
            ret_val=pow(static_cast<double>(ne.hop),fp.cost_func_phi) ;
            break;
        }
        case shmrpCostFuncDef::HOP_AND_PDR: {
            ret_val=pow(static_cast<double>(ne.hop),fp.cost_func_phi) * pow(static_cast<double>(ne.pkt_count)/static_cast<double>(ne.ack_count),fp.cost_func_pi);
            break;
        }
        case shmrpCostFuncDef::HOP_AND_INTERF: {
            ret_val=pow(static_cast<double>(ne.hop),fp.cost_func_phi) * log10(pow(ne.interf,fp.cost_func_iota));
            break;
        }
        case shmrpCostFuncDef::HOP_PDR_AND_INTERF: {
            ret_val=pow(static_cast<double>(ne.hop),fp.cost_func_phi) * log10(pow(ne.interf,fp.cost_func_iota)) * pow(static_cast<double>(ne.pkt_count)/static_cast<double>(ne.ack_count),fp.cost_func_pi);
            break;
        }
        case shmrpCostFuncDef::HOP_EMERG_AND_INTERF: {
            ret_val=pow(static_cast<double>(ne.hop),fp.cost_func_phi) * pow(ne.emerg+1,fp.cost_func_epsilon) * log10(pow(ne.interf,fp.cost_func_iota));
            break;
        }
        case shmrpCostFuncDef::HOP_EMERG_PDR_AND_INTERF: {
            ret_val=pow(static_cast<double>(ne.hop),fp.cost_func_phi) * pow(ne.emerg+1,fp.cost_func_epsilon) * log10(pow(ne.interf,fp.cost_func_iota)) * pow(static_cast<double>(ne.pkt_count)/static_cast<double>(ne.ack_count),fp.cost_func_pi);
            break;
        }
        case shmrpCostFuncDef::XPR_INTERF: {
            ret_val=log10(pow(ne.interf,fp.cost_func_iota));
            break;
        }
        case shmrpCostFuncDef::XPR_HOP_AND_PDR: {
            ret_val=fp.cost_func_phi * log10(static_cast<double>(ne.hop)) + fp.cost_func_pi * log10(static_cast<double>(ne.pkt_count)/static_cast<double>(ne.ack_count));
            break;
        }
        case shmrpCostFuncDef::XPR_HOP_PDR_AND_INTERF: {
            ret_val=fp.cost_func_phi * log10(static_cast<double>(ne.hop)) + fp.cost_func_pi * log10(static_cast<double>(ne.pkt_count)/static_cast<double>(ne.ack_count)) + fp.cost_func_iota * log10(ne.interf);
            break;
        }
        case shmrpCostFuncDef::SUM_HOP_ENERGY_EMERG_PDR_AND_INTERF: {
            // CostFunction = 1 - ObjectiveFunction
            // CostFunction = 1 - [ (1-Pi-Epsilon-Iota-Eta-Mu)*1/(1+hop)  ]
            ret_val = 1 - ( 
                    (1-fp.cost_func_pi-fp.cost_func_epsilon-fp.cost_func_iota-fp.cost_func_eta-fp.cost_func_mu)*(1/(1+ne.hop))
                    + fp.cost_func_pi*static_cast<double>(ne.ack_count)/static_cast<double>(ne.pkt_count)
                    + fp.cost_func_epsilon*ne.pathid[0].emerg
                    + fp.cost_func_iota*ne.interf
                    + fp.cost_func_eta*ne.pathid[0].enrgy
                    + fp.cost_func_mu*ne.pathid[0].nmas
                    );

            break;
        }
        default: {
            trace()<<"[error] Unknown cost function: "<<fp.cost_func;
            throw unknown_cost_function("[error] Unknown cost function");
        }
    }
    if(ne.pathid.size() != 1) {
        trace()<<"[error] Ambigous pathid array";
        throw state_not_permitted("[error] pathid array size not 1");
    }
    if(shmrpCostFuncDef::SUM_HOP_ENERGY_EMERG_PDR_AND_INTERF != fp.cost_func) {
        ret_val=ret_val/pow(static_cast<double>(ne.pathid[0].nmas+1), fp.cost_func_mu);
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

    setHop(calculateHop(fp.calc_max_hop));

    if(getHop() <= fp.ring_radius) {
        trace()<<"[info] Node inside mesh ring";
        for(auto ne: rinv_table) {
            if(ne.second.hop < getHop() && !ne.second.used) {
                trace()<<"[info] Adding entry address: "<<ne.second.nw_address<<" hop: "<<ne.second.hop<<" pathid: "<<pathidToStr(ne.second.pathid);
                rreq_table.insert(ne);
                rinv_table[ne.second.nw_address].used=true;
            }
        }
    } else {
        trace()<<"[info] Node outside mesh ring";
        std::map<int, std::vector<node_entry>> cl;
        std::for_each(rinv_table.begin(),rinv_table.end(),[&](std::pair<std::string,node_entry> ne){
                if(ne.second.hop < getHop() && !ne.second.used) {
                    for(auto p: ne.second.pathid) {
                        if(cl.find(p.pathid) == cl.end()) {
                            trace()<<"[info] Creating entry for pathid: "<<p.pathid<<" and adding entry address: "<<ne.second.nw_address<<" hop: "<<ne.second.hop<<" pathid: "<<pathidToStr(ne.second.pathid);
                            cl.insert({p.pathid,std::vector<node_entry>{ne.second}});
                        } else {
                            trace()<<"[info] Adding entry for pathid: "<<p.pathid<<" with address: "<<ne.second.nw_address<<" hop: "<<ne.second.hop<<" pathid: "<<pathidToStr(ne.second.pathid);
                            cl[p.pathid].push_back(ne.second);
                        }
                    }
                }
            });
        if(fp.cf_after_rresp) {
            trace()<<"[info] Selecting all RINV nodes to RREQ";
            for(auto l: cl) {
                for(auto n: l.second) {
                    if(n.hop < getHop()) {
                        rreq_table.insert({n.nw_address,n});
                        rinv_table[n.nw_address].used = true;
                    }
                }
            }
        } else {
            for(auto l: cl) {
                trace()<<"[info] Selecting nodes per pathid to RREQ";
                node_entry c_ne=l.second[0];
                trace()<<"[info] Candidate list entries for pathid "<<l.first<<" is " <<l.second.size();
                for(auto ne: l.second) {
                    if(calculateCostFunction(ne) < calculateCostFunction(c_ne)) {
                        trace()<<"[info] Node "<<ne.nw_address<<" preferred over node "<<c_ne.nw_address;
                        c_ne=ne;
                    }
                }
                trace()<<"[info] Selecting node "<<c_ne.nw_address<<" with pathid "<<pathidToStr(c_ne.pathid);
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

void shmrp::constructRreqTable(std::vector<int> path_filter) {
    trace()<<"[info] Entering constructRreqTable(path_filter="<<pathidToStr(path_filter)<<")";
    if(rinv_table.empty()) {
        throw rinv_table_empty("[error] RINV table empty");
    }
    if(!rreq_table.empty()) {
        throw rreq_table_non_empty("[error] RREQ table not empty");
    }

    rreq_table=rinv_table;

    for(auto it=rreq_table.begin() ; it != rreq_table.end(); ) {
        bool erase=false;
        for(auto i: path_filter) {
            for(auto p_it=it->second.pathid.begin() ; p_it != it->second.pathid.end() ;) {
                if(i==p_it->pathid) {
                    // torol+leptet
                    trace()<<"[info] Erasing path: "<<p_it->pathid;
                    trace()<<"ne: "<<it->second.nw_address;
                    p_it=it->second.pathid.erase(p_it);
                } else {
                    p_it->secl=true;
                    ++p_it;
                }
            }
        }
        if(0==it->second.pathid.size() || it->second.used) {
            trace()<<"[info] Erasing node: "<<it->second.nw_address<<" pathid: "<<pathidToStr(it->second.pathid);
            rreq_table.erase(it++);
        } else {
            rinv_table[it->first].used=true;
            ++it;
        }
    }

    if(rreq_table.empty()) {
        throw rreq_table_empty("[error] RREQ table empty after construction");
    }
    for(auto ne: rreq_table) {
        trace()<<"[info] ne: "<<ne.first<<" pathid: "<<pathidToStr(ne.second.pathid);
    }

}


bool shmrp::rreqEntryExists(const char *addr, int pathid) {
    if(rreq_table.find(string(addr)) != rreq_table.end()) {
        for(auto p: rreq_table[string(addr)].pathid) {
            if(p.pathid==pathid) {
                return true;
            }
        }
    }
    return false;
}

bool shmrp::rreqEntryExists(std::string ne) {
    if(rreq_table.find(ne) != rreq_table.end()) {
        return true;
    }
    return false;
}

void shmrp::updateRreqTableWithRresp(const char *addr, int pathid) {
    trace()<<"[info] Entering shmrp::updateRreqTableWithRresp(addr="<<addr<<", pathid="<<pathid;
    if(rreqEntryExists(addr,pathid)) {
        trace()<<"[info] RRESP received flag set to true";
        rreq_table[string(addr)].rresp=true;
        rreq_table[string(addr)].ack_count++;
    } else {
        throw std::length_error("[error] Entry not found");
    }
}


int  shmrp::getRreqPktCount() {
    if(rreq_table.empty()) {
        throw std::length_error("[error] RREQ table empty");
    }
    trace()<<"[info] Rreq Pkt count: "<<rreq_table.begin()->second.pkt_count;
    return rreq_table.begin()->second.pkt_count;
}


bool shmrp::rrespReceived() const {
    if(std::any_of(rreq_table.begin(), rreq_table.end(),[](std::pair<std::string,node_entry> ne){return ne.second.rresp; } )) {
        return true;
    }
    return false;
}


void shmrp::updateRreqEntryWithEmergency(const char *addr) {
    trace()<<"Entering shmrp::updateRreqEntryWithEmergency(addr="<<addr<<")";
    if(rreq_table.find(string(addr)) == rreq_table.end()) {
        throw no_available_entry("[error] Entry not present in routing table"); 
    }
    rreq_table[string(addr)].emerg++;
}


void shmrp::removeRreqEntry(std::string ne) {
    trace()<<"[info] Entering removeRreqEntry(ne="<<ne<<")";
    if(rreq_table.find(ne) != rreq_table.end()) {
        rreq_table.erase(ne);
    } else {
        throw no_available_entry("[error] Entry not available in RREQ table"); 
    }
}

void shmrp::sendRreqs(int count) {
    trace()<<"[info] Entering shmrp::sendRreqs(count="<<count<<")";

    if(rreq_table.empty()) {
        throw std::length_error("[error] RREQ table empty");
    }
    for(auto &&ne: rreq_table) {
        for(int i = 0 ; i < count ; ++i) {
            shmrpRreqPacket* rreq_pkt=new shmrpRreqPacket("SHMRP RREQ packet",NETWORK_LAYER_PACKET);
            rreq_pkt->setByteLength(netDataFrameOverhead);
            rreq_pkt->setShmrpPacketKind(shmrpPacketDef::RREQ_PACKET);
            rreq_pkt->setSource(SELF_NETWORK_ADDRESS);
            rreq_pkt->setDestination(ne.second.nw_address.c_str());
            rreq_pkt->setRound(getRound());
            rreq_pkt->setPathid(ne.second.pathid[0].pathid);
            rreq_pkt->setSequenceNumber(currentSequenceNumber++);
            trace()<<"[info] Sending RREQ to "<<ne.second.nw_address<<" with pathid: "<<ne.second.pathid[0].pathid;
            ne.second.pkt_count++;
            toMacLayer(rreq_pkt, resolveNetworkAddress(ne.second.nw_address.c_str()));
        }
    }
}

void shmrp::sendRreqs() {
    trace()<<"[info] Entering shmrp::sendRreqs()";
    sendRreqs(1);
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


void shmrp::sendLreqBroadcast(int round, int pathid) {
    trace()<<"[info] Entering sendLReqBroadcast(round="<<round<<", pathid="<<pathid<<")";
    sendLreq(BROADCAST_NETWORK_ADDRESS,round,pathid);
}

void shmrp::sendLreq(const char *nw_address, int round, int pathid) {
    trace()<<"[info] Entering sendLReq(nw_address="<<nw_address<<", round="<<round<<", pathid="<<pathid<<")";
    shmrpLreqPacket *lreq_pkt=new shmrpLreqPacket("SHMRP LREQ packet",NETWORK_LAYER_PACKET);
    lreq_pkt->setByteLength(netDataFrameOverhead);
    lreq_pkt->setShmrpPacketKind(shmrpPacketDef::LREQ_PACKET);
    lreq_pkt->setSource(SELF_NETWORK_ADDRESS);
    lreq_pkt->setDestination(nw_address);
    lreq_pkt->setRound(round);
    lreq_pkt->setPathid(pathid);
    lreq_pkt->setHop(getHop());
    lreq_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(lreq_pkt, resolveNetworkAddress(nw_address));
}

void shmrp::sendLreqUnicast(int round, int pathid) {
    trace()<<"[info] Entering sendLReqUnicast(round="<<round<<", pathid="<<pathid<<")";
    for(auto ne: recv_table) {
        // pathid 0 means we are inside the ring
        bool send=false;
        for(auto p: ne.second.pathid) {
            if(p.pathid==pathid) {
                send=true;
            }
        }
        if((send || pathid == 0) && !ne.second.secl) {
            sendLreq(ne.second.nw_address.c_str(), round, pathid);
        }
    }
}

void shmrp::sendLresp(const char *nw_address, int round, int pathid) {
    trace()<<"[info] Entering sendLResp(nw_address="<<nw_address<<", round="<<round<<", pathid="<<pathid<<")";
    shmrpLrespPacket *lresp_pkt=new shmrpLrespPacket("SHMRP LRESP packet",NETWORK_LAYER_PACKET);
    lresp_pkt->setByteLength(netDataFrameOverhead);
    lresp_pkt->setShmrpPacketKind(shmrpPacketDef::LRESP_PACKET);
    lresp_pkt->setSource(SELF_NETWORK_ADDRESS);
    lresp_pkt->setDestination(nw_address);
    lresp_pkt->setRound(round);
    lresp_pkt->setPathid(pathid);
    lresp_pkt->setHop(getHop());
    lresp_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(lresp_pkt, resolveNetworkAddress(nw_address));
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
    data_pkt->setReroute(0);
    data_pkt->setSequenceNumber(currentSequenceNumber++);
    data_pkt->encapsulate(pkt);
    toMacLayer(data_pkt, resolveNetworkAddress(dest.c_str()));
}

void shmrp::schedulePkt(cPacket *pkt, std::string dest, int pathid) {
    trace()<<"[info] Entering schedulePkt(pkt, dest="<<dest<<", pathid="<<pathid<<")";
    shmrpDataPacket *data_pkt=new shmrpDataPacket("SHMRP DATA packet",NETWORK_LAYER_PACKET);
    data_pkt->setByteLength(netDataFrameOverhead);
    data_pkt->setShmrpPacketKind(shmrpPacketDef::DATA_PACKET);
    data_pkt->setSource(SELF_NETWORK_ADDRESS);
    data_pkt->setOrigin(SELF_NETWORK_ADDRESS);
    data_pkt->setDestination(dest.c_str());
    data_pkt->setPathid(pathid);
    data_pkt->setSequenceNumber(currentSequenceNumber++);
    data_pkt->setRepeat(0);
    data_pkt->setReroute(0);
    data_pkt->encapsulate(pkt);
    pkt_list.push_back(data_pkt);
}

void shmrp::forwardData(shmrpDataPacket *data_pkt, std::string dest, bool reroute=false) {
    trace()<<"[info] Entering forwardData(dest="<<dest<<")";
    forwardData(data_pkt, dest, data_pkt->getPathid(),reroute);
}

void shmrp::forwardData(shmrpDataPacket *data_pkt, std::string dest, int pathid, bool reroute) {
    trace()<<"[info] Entering forwardData(dest="<<dest<<", pathid="<<pathid<<", reroute="<<reroute<<")";
    data_pkt->setSource(SELF_NETWORK_ADDRESS);
    data_pkt->setDestination(dest.c_str());
    data_pkt->setPathid(pathid);
    if(reroute) {
        data_pkt->setReroute(data_pkt->getReroute()+1);
    }
    data_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(data_pkt, resolveNetworkAddress(dest.c_str()));
}

void shmrp::clearRoutingTable() {
    trace()<<"[info] Routing table erased";
    routing_table.clear();
}

void shmrp::addRoute(std::string next_hop, int pathid) {
    trace()<<"[info] Entering shmrp::addRoute(next_hop="<<next_hop<<", pathid="<<pathid<<")";
    if(routing_table.find(next_hop) == routing_table.end()) {
        routing_table[next_hop]={next_hop,std::vector<pathid_entry>(1,{pathid,0}) };
    } else {
        trace()<<"[error] Route already exists";
    }
}

bool shmrp::isRoutingTableEmpty() const {
    return routing_table.empty();
}

void shmrp::constructRoutingTableFromRinvTable() {
    trace()<<"[info] Entering constructRoutingTableFromRinvTable()";
    std::map<std::string,node_entry> filt_table;
    for(auto ne: rinv_table) {
        if(ne.second.local) {
            filt_table[ne.second.nw_address]=ne.second;
        }
    }

    //    for(auto 
}

void shmrp::removeRoute(std::string ne) {
    trace()<<"[info] Entering removeRoute(ne="<<ne<<")";
    routing_table.erase(ne);
}


void shmrp::constructRoutingTable(bool rresp_req) {
    trace()<<"[info] Entering shmrp::constructRoutingTable(rresp_req="<<rresp_req<<")";
    for(auto ne: rreq_table) {
        if(ne.second.rresp || !rresp_req) {
            trace()<<"[info] Adding node "<<ne.second.nw_address<<" with pathid "<<pathidToStr(ne.second.pathid);
            routing_table.insert(ne);
        }
    }
    if(routing_table.empty()) {
        throw routing_table_empty("[error] Routing table empty");
    }
}

void shmrp::constructRoutingTable(bool rresp_req, bool app_cf, double pdr=0.0, bool update=false) {
    trace()<<"[info] Entering shmrp::constructRoutingTable(rresp_req="<<rresp_req<<", app_cf="<<app_cf<<", pdr="<<pdr<<")";
    if(!app_cf) {
        constructRoutingTable(rresp_req);
        return;
    }

    auto calc_pdr=[&](node_entry n){ trace()<<"[info] PDR: "<<static_cast<double>(n.ack_count)/static_cast<double>(n.pkt_count);return static_cast<double>(n.ack_count)/static_cast<double>(n.pkt_count);};
    auto clean_pathid=[](node_entry n, int p){ for(int i=0 ; i < n.pathid.size() ; ++i) { if(n.pathid[i].pathid == p) { auto ret=n; ret.pathid=std::vector<pathid_entry>(1,n.pathid[i]); return ret; } } };

    if(getHop() <= fp.ring_radius) {
        if(update) {
            trace()<<"[error] No routing table update inside the ring";
            throw state_not_permitted("[error] No routing table update inside the ring");
            /* but why? */ 
        }
        trace()<<"[info] Node inside mesh ring";
        for(auto ne: rreq_table) {
            if(ne.second.hop < getHop() && (ne.second.rresp || !fp.rresp_req ) && calc_pdr(ne.second) >= pdr ) {
                trace()<<"[info] Adding entry address: "<<ne.second.nw_address<<" hop: "<<ne.second.hop<<" pathid: "<<pathidToStr(ne.second.pathid);
                routing_table.insert(ne);
                //                rinv_table[ne.second.nw_address].used=true;
            }
        }
        if(isRoutingTableEmpty()) {
            throw routing_table_empty("[error] routing table empty after constructRoutingTable()");
        }
        return;
    }

    std::map<int, std::vector<node_entry>> cl;
    std::for_each(rreq_table.begin(),rreq_table.end(),[&](std::pair<std::string,node_entry> ne){
            // Either select only hop-based, if rresp is not required or based on rresp received
            // During update do not consider hop
            if((update || ne.second.hop < getHop()) && (ne.second.rresp || !fp.rresp_req) && calc_pdr(ne.second) >= pdr ) {
                for(auto p: ne.second.pathid) {
                    if(cl.find(p.pathid) == cl.end()) {
                        trace()<<"[info] Creating pathid entry "<<p.pathid<<" and adding entry address: "<<ne.second.nw_address<<" hop: "<<ne.second.hop<<" pathid: "<<pathidToStr(ne.second.pathid);
                        cl.insert({p.pathid,std::vector<node_entry>{clean_pathid(ne.second,p.pathid)}});
                    } else {
                        trace()<<"[info] Adding to pathid entry "<<p.pathid<<" entry address: "<<ne.second.nw_address<<" hop: "<<ne.second.hop<<" pathid: "<<pathidToStr(ne.second.pathid);
                        cl[p.pathid].push_back(clean_pathid(ne.second,p.pathid));
                    }
                }
            }
       });



    for(auto l: cl) {
        trace()<<"[info] Selecting nodes per pathid to RREQ";
        node_entry c_ne=l.second[0];

        trace()<<"[info] Candidate list entries for pathid "<<l.first<<" is " <<l.second.size();
        if(fp.drop_1st_rt_c && getRNG(0)->doubleRand() > 1.0-fp.drop_prob) {
            trace()<<"[info] Drop 1st routing table candidate active";
            if(l.second.size()>1) {
                for(auto ne: l.second) {
                    trace()<<"[info] Node "<<ne.nw_address<<" pkt data - sent: "<<ne.pkt_count<<" ack: "<<ne.ack_count;
                    if(calculateCostFunction(ne) < calculateCostFunction(c_ne)) {
                        trace()<<"[info] Node "<<ne.nw_address<<" preferred over node "<<c_ne.nw_address;
                        c_ne=ne;
                    }
                }
                auto it=l.second.begin();
                while(it->nw_address != c_ne.nw_address) {
                    ++it;
                }
                l.second.erase(it);
                c_ne=l.second[0];
                for(auto ne: l.second) {
                    trace()<<"[info] Node "<<ne.nw_address<<" pkt data - sent: "<<ne.pkt_count<<" ack: "<<ne.ack_count;
                    if(calculateCostFunction(ne) < calculateCostFunction(c_ne)) {
                        trace()<<"[info] Node "<<ne.nw_address<<" preferred over node "<<c_ne.nw_address;
                        c_ne=ne;
                    }
                }
            }
        } else {
            for(auto ne: l.second) {
                trace()<<"[info] Node "<<ne.nw_address<<" pkt data - sent: "<<ne.pkt_count<<" ack: "<<ne.ack_count;
                if(calculateCostFunction(ne) < calculateCostFunction(c_ne)) {
                    trace()<<"[info] Node "<<ne.nw_address<<" preferred over node "<<c_ne.nw_address;
                    c_ne=ne;
                }
            }
        }
        trace()<<"[info] Selecting node "<<c_ne.nw_address<<" with pathid "<<pathidToStr(c_ne.pathid);
        c_ne.pkt_count = 0;
        c_ne.ack_count = 0;
       
        // quite dangerous 
        auto p=c_ne.pathid[0];
        p.secl=update;
        c_ne.pathid.clear();
        if(routing_table.find(c_ne.nw_address) == routing_table.end()) {
            routing_table.insert({c_ne.nw_address,c_ne});
            routing_table[c_ne.nw_address].secl=update; // Set secl parameter to true, if second learn
            rinv_table[c_ne.nw_address].used=true;
            routing_table[c_ne.nw_address].pathid.push_back(p);
        } else {
            routing_table[c_ne.nw_address].pathid.push_back(p);
        }
    }
    if(routing_table.empty()) {
        throw routing_table_empty("[error] routing table empty after constructRoutingTable()");
    }
}

pathid_entry shmrp::selectPathid(bool replay) {
    trace()<<"[info] Entering shmrp::selectPathid(replay="<<replay<<")";
    if(!replay) {
        g_pathid=selectPathid();
    }
    return g_pathid;
}


std::vector<int> shmrp::selectAllPathid() {
    return selectAllPathid(fp.path_sel);
}

std::vector<int> shmrp::selectAllPathid(int path_sel) {
    trace()<<"[info] Entering selectAllPathid()";
     if(routing_table.empty()) {
        throw routing_table_empty("[error] Routing table empty");
    }

    auto tmp_table = routing_table;

    if(0 != path_sel) {
        for(auto ne: routing_table) {
            // Prefer primary
            if(1 == path_sel && ne.second.secl) {
                tmp_table.erase(ne.first);
            }
            // Prefer secl
            if(2 == path_sel && !ne.second.secl) {
                tmp_table.erase(ne.first);
            }
        }
    }

    if(tmp_table.empty()) {
        trace()<<"[info] Cannot apply path selection policy"; 
        tmp_table = routing_table;
    }

    std::vector<int> pathid;
    for(auto ne: routing_table) {
        for(auto p: ne.second.pathid) {
            pathid.push_back(p.pathid);
        }
    }
    trace()<<"[info] Selected pathids: "<<pathidToStr(pathid);
    return pathid;
}

pathid_entry shmrp::selectPathid() {
    trace()<<"[info] Entering shmrp::selectPathid()";
    if(routing_table.empty()) {
        throw routing_table_empty("[error] Routing table empty");
    }

    auto tmp_table = routing_table;

    if(0 != fp.path_sel) {
        for(auto ne: routing_table) {
            // Prefer primary
            if(1 == fp.path_sel && ne.second.secl) {
                tmp_table.erase(ne.first);
            }
            // Prefer secl
            if(2 == fp.path_sel && !ne.second.secl) {
                tmp_table.erase(ne.first);
            }
        }
    }

    if(tmp_table.empty()) {
        trace()<<"[info] Cannot apply path selection policy"; 
        tmp_table = routing_table;
    }

    std::vector<pathid_entry> pathid;
    for(auto ne: tmp_table) {
        for(auto p: ne.second.pathid) {
            trace()<<"[info] Pathid collected: "<<p.pathid;
            pathid.push_back(p);
        }
    }

    auto i=getRNG(0)->intRand(pathid.size());
    trace()<<"[info] Random number: "<<i;
    auto it=pathid.begin();
    std::advance(it,i);
    trace()<<"[info] Selected pathid: "<<it->pathid;
    return *it;
}

std::string shmrp::pathidToStr(vector<pathid_entry> pathid) {
    std::string str;
    for(auto i: pathid) {
        str.append(std::to_string(i.pathid));
        str.append(" ");
    }
    return str;
}

std::string shmrp::pathidToStr(vector<int> pathid) {
    std::string str;
    for(auto i: pathid) {
        str.append(std::to_string(i));
        str.append(" ");
    }
    return str;
}

std::string shmrp::getNextHop(int pathid) {
    trace()<<"[info] Entering getNextHop(pathid="<<pathid<<")";
    node_entry next_hop;
    bool found=false;
    std::for_each(routing_table.begin(),routing_table.end(),
            [&](std::pair<std::string,node_entry> ne){
            for(auto p: ne.second.pathid) {
            if(p.pathid== pathid) {
            found=true;
            next_hop=ne.second;
            }
            }
            });
    if(!found) {
        throw no_available_entry("Next hop not available");
    }
    trace()<<"[info] Next hop: "<<next_hop.nw_address;
    return next_hop.nw_address;
}

std::string shmrp::getNextHop(int pathid, bool random_node) {
    trace()<<"[info] Entering getNextHop(pathid="<<pathid<<", random_node="<<random_node<<")";
    std::string next_hop;
    if(random_node) {
        std::vector<node_entry> nodes;
        for(auto a : routing_table) {
            for(auto p: a.second.pathid) {
                if(p.pathid==pathid) {
                    nodes.push_back(a.second);
                }
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

bool shmrp::checkPathid(int pathid) {
    trace()<<"[info] Entering checkPathid(pathid="<<pathid<<")";
    for(auto ne: routing_table) {
        for(auto p: ne.second.pathid) {
            if(p.pathid==pathid) {
                trace()<<"[info] Pathid found";
                return true;
            }
        }
    }
    trace()<<"[info] Pathid not found";
    return false;
}

bool shmrp::checkRoute(std::string ne) {
    trace()<<"Entering checkRoute(ne="<<ne<<")";
    if(routing_table.find(ne)!=routing_table.end()) {
        return true;
    }
    return false;
}


int shmrp::calculateRepeat(const char *dest) {
    trace()<<"[info] Entering shmrp::calculateRepeat(dest="<<dest<<")";

    auto entry_it = routing_table.find(std::string(dest));

    if(entry_it == routing_table.end()) {
        throw no_available_entry("[error] Entry not available in routing table");
    } 

    double hop = static_cast<double>(entry_it->second.hop) + 1;
    double rep = 0;
    if(fp.rep_m_pdr) {
        double m_pdr=static_cast<double>(entry_it->second.ack_count)/static_cast<double>(entry_it->second.pkt_count);
        rep = ceil( log(1.0 - fp.e2e_qos_pdr) / log(1.0 - pow(m_pdr,hop) ));
    } else {
        rep = ceil( log(1.0 - fp.e2e_qos_pdr) / log(1.0 - pow(fp.qos_pdr,hop) ));
    }

    trace()<<"[info] Repeat for destination "<<dest<<" with hop "<<hop<<" is "<<rep;

    return static_cast<int>(rep);

    //    return rep;
}


void shmrp::incPktCountInRecvTable(std::string entry, int pathid, int round) {
    trace()<<"[info] Entering incPktCountInRecvTable("<<entry<<")";
    if(recv_table.find(entry) == recv_table.end()) {
        recv_table[entry] = {entry,std::vector<pathid_entry>(1,{pathid,0}),0,false,0,0,false,round,1};
    } else {
        bool create=true;
        for(auto p: recv_table[entry].pathid) {
            if(p.pathid==pathid) {
                create=false;
            }
        }
        if(create) {
            recv_table[entry].pathid.push_back({pathid,0});
        }
        recv_table[entry].pkt_count++;
    }
}

void shmrp::updateRinvTableFromRreqTable() {
    trace()<<"[info] Entering updateRinvTableFromRreqTable()";
    for(auto ne: rreq_table) {
        if(rinv_table.find(ne.first) != rinv_table.end()) {
            trace()<<"[info] Updating node "<<ne.first<<" as used in RINV table";
            rinv_table[ne.first].used = true;
        }
    }
}

void shmrp::timerFiredCallback(int index) {
    switch (index) {
        case shmrpTimerDef::SINK_START: {
            trace()<<"[timer] SINK_START timer expired";
            setRound(1+getRound());
            sendRinv(getRound());
            setTimer(shmrpTimerDef::T_REPEAT,par("t_start").doubleValue()*10.0);
            break;
        }
        case shmrpTimerDef::T_SEC_L_START: {
            trace()<<"[timer] T_SEC_L_START timer expired, starting T_SEC_L timer";
            setTimer(shmrpTimerDef::T_SEC_L,fp.t_sec_l);
            break;
        }
        case shmrpTimerDef::T_SEC_L: {
            trace()<<"[timer] T_SEC_L timer expired";
            if(isSink()) {
                trace()<<"[info] Sink starting second learn";
                switch(fp.second_learn) {
                    case shmrpSecLParDef::OFF: {
                        trace()<<"[error] second learn off, while T_SEC_L timer expired";
                        throw std::runtime_error("[error] Invalid state");
                    }
                    case shmrpSecLParDef::BROADCAST: {
                        sendLreqBroadcast(getRound(),0);
                        break;
                    }
                    case shmrpSecLParDef::UNICAST: {
                        if(0 == recv_table.size()) {
                            trace()<<"[info] Recv table size condition not met (empty)";
                            setTimer(shmrpTimerDef::T_SEC_L,fp.t_sec_l);
                            break;
                        }
                        // most probably true...
                        if(!secLPerformed(getRound(), 0)) {
                            sendLreqUnicast(getRound(),0);
                            setTimer(shmrpTimerDef::T_SEC_L_REPEAT,fp.t_sec_l_repeat);
                        }
                        break;
                    }
                    default: {
                        trace()<<"[error] How do we get here?";
                    }
                }
                break;
            }
            if(shmrpStateDef::WORK != getState()) {
                trace()<<"[info] Node still not in WORK state, starting T_SEC_L timer.";
                setTimer(shmrpTimerDef::T_SEC_L, fp.t_sec_l);
                break;
            }

            switch (getRingStatus()) {
                case shmrpRingDef::CENTRAL: {
                    trace()<<"[info] Node is sink, impossibru";
                    break;
                }
                case shmrpRingDef::INTERNAL: {
                    trace()<<"[info] Node is internal";
                    switch(fp.second_learn) {
                        case shmrpSecLParDef::OFF: {
                            trace()<<"[error] second learn off, while T_SEC_L timer expired";
                            throw std::runtime_error("[error] Invalid state");
                        }
                        case shmrpSecLParDef::BROADCAST: {
                            sendLreqBroadcast(getRound(),0);
                            break;
                        }
                        case shmrpSecLParDef::UNICAST: {
                            // most probably true...
                            if(!secLPerformed(getRound(), 0)) {
                                sendLreqUnicast(getRound(),0);
                                setTimer(shmrpTimerDef::T_SEC_L_REPEAT,fp.t_sec_l_repeat);
                            }
                            break;
                        }
                        default: {
                            trace()<<"[error] How do we get here?";
                        }
                    }
                    break;
                }
                case shmrpRingDef::BORDER: {
                    trace()<<"[info] Node is at border";
                    switch(fp.second_learn) {
                        case shmrpSecLParDef::OFF: {
                            trace()<<"[error] second learn off, while T_SEC_L timer expired";
                            throw std::runtime_error("[error] Invalid state");
                        }
                        case shmrpSecLParDef::BROADCAST: {
                            sendLreqBroadcast(getRound(),resolveNetworkAddress(SELF_NETWORK_ADDRESS));
                            break;
                        }
                        case shmrpSecLParDef::UNICAST: {
                            // most probably true...
                            if(!secLPerformed(getRound(),resolveNetworkAddress(SELF_NETWORK_ADDRESS) )) {
                                sendLreqUnicast(getRound(),resolveNetworkAddress(SELF_NETWORK_ADDRESS));
                                setTimer(shmrpTimerDef::T_SEC_L_REPEAT,fp.t_sec_l_repeat);
                            }
                            break;
                        }
                        default: {
                            trace()<<"[error] How do we get here?";
                        }
                    }

                    break;
                }
                case shmrpRingDef::EXTERNAL: {
                    trace()<<"[info] Node is external.";
                    bool send_sec_l=false;
                    if(1 < getRoutingTableSize()) {
                        trace()<<"[info] Enough routing entries";
                    } 
                    saveRreqTable();
                    clearRreqTable();
                    try {
                        constructRreqTable(selectAllPathid(0));
                    } catch (rreq_table_empty &e) {
                        trace()<<e.what();
                        trace()<<"[info] No alternate path possible to learn, propagate second learn.";
                        send_sec_l=true;
                        retrieveRreqTable();
                    }
                    catch (exception &e) {
                        trace()<<e.what();
                        break;
                    
                    }
                    if(send_sec_l) {
                        switch(fp.second_learn) {
                            case shmrpSecLParDef::OFF: {
                                trace()<<"[error] second learn off, while T_SEC_L timer expired";
                                throw std::runtime_error("[error] Invalid state");
                            }
                            case shmrpSecLParDef::BROADCAST: {
                                sendLreqBroadcast(getRound(),getSecLPathid());
                                break;
                            }
                            case shmrpSecLParDef::UNICAST: {
                                // most probably true...
                                if(!secLPerformed(getRound(),getSecLPathid() )) {
                                    sendLreqUnicast(getRound(),getSecLPathid());
                                    setTimer(shmrpTimerDef::T_SEC_L_REPEAT,fp.t_sec_l_repeat);
                                }
                                break;
                            }
                            default: {
                                trace()<<"[error] How do we get here?";
                            }
                        }

                        break;
                    }

                    setState(shmrpStateDef::S_ESTABLISH);
                    sendRreqs(); 
                    setTimer(shmrpTimerDef::T_ESTABLISH,getTest());
                    break;
                }
            } 
            break;
        }
        case shmrpTimerDef::T_SEC_L_REPEAT: {
            trace()<<"[timer] T_SEC_L_REPEAT timer expired";
            if(!secLPerformed(getRound(),getSecLPathid()) && g_sec_l_timeout < fp.t_sec_l_timeout-1 ) {
                sendLreqUnicast(getRound(),getSecLPathid());
                ++g_sec_l_timeout;
                setTimer(shmrpTimerDef::T_SEC_L_REPEAT,fp.t_sec_l_repeat);
            }
            break;
        }

        case shmrpTimerDef::T_REPEAT: {
            sendRinv(getRound());
            setTimer(shmrpTimerDef::T_REPEAT,par("t_start").doubleValue()*10.0);
            break;
        }
        case shmrpTimerDef::T_L: {
            trace()<<"[timer] T_L timer expired";

            switch (getState()) {
                case shmrpStateDef::LOCAL_LEARN: {
                    trace()<<"[info] LOCAL_LEARN finished";
                    constructRoutingTableFromRinvTable();
                    setState(shmrpStateDef::WORK);
                    break;
                }
                case shmrpStateDef::LEARN: {
                    trace()<<"[info] LEARN finished";
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
                    sendRreqs(); //maybe we could go directly to measure or work in case of hdmrp
                    setTimer(shmrpTimerDef::T_ESTABLISH,getTest());
                    break;
                }
                default: {
                    trace()<<"[error] State is not LEARN or LOCAL_LEARN: "<<stateToStr(getState());
                }
            }
            break;
        }
        case shmrpTimerDef::T_ESTABLISH: {
            trace()<<"[timer] T_ESTABLISH timer expired";

            if(isRreqTableEmpty()) {
                trace()<<"[error] RREQ table empty, impossibru";
                throw rreq_table_empty("[error] RREQ table empty");
            }

            if(fp.measure_w_rreq && (fp.meas_rreq_count > getRreqPktCount()) ) {
                sendRreqs();
                trace()<<"[info] measure_w_rreq active, restarting T_ESTABLISH timer";
                setTimer(shmrpTimerDef::T_ESTABLISH,getTest());
                break;
            }
            if(shmrpStateDef::S_ESTABLISH == getState()) {
                trace()<<"[info] Second learn finished";
                constructRoutingTable(fp.rresp_req, fp.cf_after_rresp, fp.qos_pdr, true /*update */ );
                retrieveAndMergeRreqTable();

                if(shmrpSecLParDef::OFF != fp.second_learn && isMaster()) {
                    sendRinvBasedOnHop();
                }

                switch(fp.second_learn) {
                    case shmrpSecLParDef::OFF: {
                        trace()<<"[error] second learn off, while T_SEC_L timer expired";
                        throw std::runtime_error("[error] Invalid state");
                    }
                    case shmrpSecLParDef::BROADCAST: {
                        while(!isSecLPathidEmpty()) {
                            sendLreqBroadcast(getRound(),popSecLPathid());
                        }
                    }
                    case shmrpSecLParDef::UNICAST: {
                        // most probably true...
                        if(!secLPerformed(getRound(),getSecLPathid() )) {
                            sendLreqUnicast(getRound(),getSecLPathid());
                            setTimer(shmrpTimerDef::T_SEC_L_REPEAT,fp.t_sec_l_repeat);
                        }
                        break;
                    }
                    default: {
                        trace()<<"[error] How do we get here?";
                    }
                }

                setState(shmrpStateDef::WORK);
                break;
            }

            if(!rrespReceived() && fp.rresp_req ) {
                trace()<<"[error] No RRESP packet received";
                if(fp.rst_learn) {
                    trace()<<"[info] Returning to learning state, staying in round";
                    setState(shmrpStateDef::LEARN);
                    //setRound(getRound()-1);
                    setTimer(shmrpTimerDef::T_L,getTl());
                    if(shmrpRinvTblAdminDef::ERASE_ON_LEARN==fp.rinv_tbl_admin) {
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
                    constructRoutingTable(fp.rresp_req, fp.cf_after_rresp, fp.qos_pdr);
                } else {
                    constructRoutingTable(fp.rresp_req);
                }
                setHop(calculateHopFromRoutingTable());
            } catch (routing_table_empty &e) {
                trace()<<e.what();
                trace()<<"[info] Returning to LEARN state";
                setState(shmrpStateDef::LEARN);
                setTimer(shmrpTimerDef::T_L,getTl());
                break;
                // return
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
                if(fp.e2e_qos_pdr > 0) {
                    trace()<<"[info] Starting T_SEND_PKT to schedule pkt sending";
                    setTimer(shmrpTimerDef::T_SEND_PKT,fp.t_send_pkt);
                }
                setState(shmrpStateDef::WORK);
                sendRinvBasedOnHop();
            }
            break;
        }
        case shmrpTimerDef::T_MEASURE: {
            trace()<<"[timer] T_MEASURE timer expired, PONG table size: "<<getPongTableSize();
            // What if it is in ESTABLISH state? What if new round? how to handle RINV table?
                trace()<<"[info] Measurement done, transitioning to WORK state";
                if(fp.e2e_qos_pdr > 0) {
                    trace()<<"[info] Starting T_SEND_PKT to schedule pkt sending";
                    setTimer(shmrpTimerDef::T_SEND_PKT,fp.t_send_pkt);
                }

            setState(shmrpStateDef::WORK);
            sendRinvBasedOnHop();
            break;
        }
        case shmrpTimerDef::T_SEND_PKT: {
            trace()<<"[timer] T_SEND_PKT timer expired, pkt_list size: "<<pkt_list.size();
            if(!pkt_list.empty()) {
                for(auto it = pkt_list.begin() ; it != pkt_list.end(); ) {
                    bool erased = false;
                    trace()<<"[info] Entry's destination: "<<(*it)->getDestination();
                    int rep;
                    try {
                        rep = calculateRepeat((*it)->getDestination());
                    } catch (exception &e) {
                        trace()<<e.what();
                        trace()<<"[info] Remove entry";
                        shmrpDataPacket *pkt_ptr = *it;
                        pkt_list.erase(it++);
                        erased=true;
                        delete pkt_ptr;
                        ++it;
                        continue;
                    }
                    toMacLayer((*it)->dup(), resolveNetworkAddress((*it)->getDestination()));
                    incPktCountInRoutingTable(std::string((*it)->getDestination()));

                    (*it)->setRepeat((*it)->getRepeat()+1);
                    if((*it)->getRepeat() >= rep) {
                        trace()<<"[info] Repeat count for pkt reached";
                        shmrpDataPacket *pkt_ptr = *it;
                        pkt_list.erase(it++);
                        erased=true;
                        delete pkt_ptr;
                    }
                    if(!erased) {
                        ++it;
                    }
                }
            }
            trace()<<"[timer] Re-scheduling T_SEND_PKT";
            setTimer(shmrpTimerDef::T_SEND_PKT,fp.t_send_pkt);
            break;
        }

        default: {
            trace()<<"[error] Unknown timer expired: "<<index;
            break;
        }
    }
}

void shmrp::fromApplicationLayer(cPacket * pkt, const char *destination) {
    if(0==std::strcmp(destination,BROADCAST_NETWORK_ADDRESS)) {
        trace()<<"[info] Broadcast packet";
        sendData(pkt,std::string(destination),0);
    } else if(0!=std::strcmp(destination,getSinkAddress().c_str())) {
        trace()<<"[error] Packet's destination not sink: "<<destination;
        return;
    } else {
        // Shouldn't we buffer?
        if(isRoutingTableEmpty()) {
            trace()<<"[error] Routing table empty, can't route packet";
            return;
        }

        auto pathid=selectPathid();
        std::string next_hop;

        if(shmrpRingDef::EXTERNAL==getRingStatus()) {
            next_hop=getNextHop(pathid.pathid);
        } else {
            next_hop=getNextHop(pathid.pathid,fp.rand_ring_hop);
        }

        incPktCountInRoutingTable(next_hop);
        if(fp.e2e_qos_pdr > 0.0) {
            schedulePkt(pkt, next_hop, pathid.pathid);
        }
        else {
            sendData(pkt,next_hop,pathid.pathid);
        }
    }
}

void shmrp::fromMacLayer(cPacket * pkt, int srcMacAddress, double rssi, double lqi) {
    shmrpPacket *net_pkt=dynamic_cast<shmrpPacket *>(pkt);
    if(!net_pkt) {
        trace()<<"[error] Dynamic cast of packet failed";
    }

    trace()<<"[info] Packet received from: NW "<<net_pkt->getSource()<<" MAC: "<<srcMacAddress;

    switch (net_pkt->getShmrpPacketKind()) {
        case shmrpPacketDef::RINV_PACKET: {
            trace()<<"[info] RINV_PACKET received";
            auto rinv_pkt=dynamic_cast<shmrpRinvPacket *>(pkt);

            handleTSecLTimer();

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
                setSecL(false);
                setState(shmrpStateDef::LEARN);
                setTimer(shmrpTimerDef::T_L,getTl());

            } else if(rinv_pkt->getRound() == getRound()) {
                if(true == rinv_pkt->getLocal()) {
                    // HOP!!!!!
                    if(shmrpStateDef::WORK == getState()) {
                        if(checkLocalid(rinv_pkt->getLocalid())) {
                            trace()<<"[info] Local learn initiated by "<<rinv_pkt->getLocalid()<<" already executed, exiting";
                        } else {
                            trace()<<"[info] New local learn, initiated by "<<rinv_pkt->getLocalid();
                            storeLocalid(rinv_pkt->getLocalid());
                            setState(shmrpStateDef::LOCAL_LEARN);
                            setTimer(shmrpTimerDef::T_L, getTl());
                            clearRinvTableLocalFlags();
                            try {
                                markRinvEntryLocal(std::string(rinv_pkt->getSource()));
                            } catch (no_available_entry &e) {
                                trace()<<e.what();
                                // maybe create?
                                break;
                            }
                        }
                        break;
                    } else if(shmrpStateDef::LOCAL_LEARN == getState()) {
                        if(checkLocalid(rinv_pkt->getLocalid())) {
                            trace()<<"[info] Local learn initiated by "<<rinv_pkt->getLocalid()<<" already executed, LOCAL_LEARN not restarting, exiting";
                            break; // this should be moved out
                        } else {
                            trace()<<"[info] Local learn restart due to "<<rinv_pkt->getLocalid();
                            storeLocalid(rinv_pkt->getLocalid());
                            cancelTimer(shmrpTimerDef::T_L);
                            setTimer(shmrpTimerDef::T_L, getTl());
                            try {
                                markRinvEntryLocal(std::string(rinv_pkt->getSource()));
                            } catch (no_available_entry &e) {
                                trace()<<e.what();
                                // maybe create?
                                break;
                            }
                            break;
                        }
                    }
                } else if(shmrpStateDef::LEARN != getState() && shmrpRinvTblAdminDef::ERASE_ON_LEARN==fp.rinv_tbl_admin) {
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

            handleTSecLTimer();

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
        case shmrpPacketDef::LREQ_PACKET: {
            trace()<<"[info] LREQ_PACKET received";
            if(isSink()) {
                trace()<<"[info] Node is sink, discarding LREQ_PACKET";
                break;
            }
            auto lreq_packet=dynamic_cast<shmrpLreqPacket *>(pkt);

            if(lreq_packet->getRound() != getRound()) {
                trace()<<"LREQ_PACKET out of round. Packet round: "<<lreq_packet->getRound()<<" own round: "<<getRound();
                break;
            }

            if(getSecL(lreq_packet->getPathid())) {
                trace()<<"[info] Second learn already performed";
                if(0==std::strcmp(lreq_packet->getDestination(),SELF_NETWORK_ADDRESS)) {
                    sendLresp(lreq_packet->getSource(),getRound(),lreq_packet->getPathid());
                }
                break;
            }

            // This is not a real issue, isn't it?
            if(lreq_packet->getHop() >= getHop()) {
                trace()<<"[info] LREQ_PACKET hop is higher than local hop. Packet hop: "<<lreq_packet->getRound()<<" own hop: "<<getHop();
                //                break;
            }

            if(shmrpRingDef::EXTERNAL == getRingStatus()) {
                // Most probably always true
                if(!checkPathid(lreq_packet->getPathid())) {
                    trace()<<"[info] LREQ_PACKET indicates unknown pathid, discarding.";
                    break;
                }
            }

            if(0==std::strcmp(lreq_packet->getDestination(),SELF_NETWORK_ADDRESS)) {
                sendLresp(lreq_packet->getSource(),getRound(),lreq_packet->getPathid());
            }

            setSecL(lreq_packet->getPathid(),true);
            if(getRingStatus() == shmrpRingDef::BORDER) {
                pushSecLPathid(resolveNetworkAddress(SELF_NETWORK_ADDRESS));
            } else {
                pushSecLPathid(lreq_packet->getPathid());
            }
            if(getTimer(shmrpTimerDef::T_SEC_L)!=-1 || getState()==shmrpStateDef::S_ESTABLISH ) {
                trace()<<"[info] T_SEC_L timer active or S_ESTABLISH state, no restart";
            } else {
                trace()<<"[info] Starting T_SEC_L timer";
                setTimer(shmrpTimerDef::T_SEC_L, fp.t_sec_l);
            }
            break;
        }
        case shmrpPacketDef::LRESP_PACKET: {
            trace()<<"[info] LRESP_PACKET received";
            auto lresp_packet=dynamic_cast<shmrpLrespPacket *>(pkt);
            if(recv_table.find(std::string(lresp_packet->getSource())) != recv_table.end()) {
                trace()<<"[info] Entry "<<lresp_packet->getSource()<<" exists";
                if(recv_table[std::string(lresp_packet->getSource())].round == lresp_packet->getRound()) { // && recv_table[std::string(lresp_packet->getSource())].pathid.end() != std::find(recv_table[std::string(lresp_packet->getSource())].pathid.begin(), recv_table[std::string(lresp_packet->getSource())].pathid.end() ,lresp_packet->getPathid())) {
                    trace()<<"[info] Entry has valid round";
                    recv_table[std::string(lresp_packet->getSource())].secl=true;
                }
            }
            break;
        }
        case shmrpPacketDef::RWARN_PACKET: {
            trace()<<"[info] RWARN_PACKET received";
            auto rwarn_pkt=dynamic_cast<shmrpRwarnPacket *>(pkt);
            if(rwarn_pkt->getRound() > getRound()) {
                trace()<<"[warn] Warning node's round is greater than receiving node's round. Exiting.";
                break;
            }
            switch (rwarn_pkt->getCause()) {
                case shmrpWarnDef::EMERGENCY_EVENT: {
                    trace()<<"[info] EMERGENCY_EVENT";
                    if(fp.rt_recalc_warn) {
                        try {
                            updateRreqEntryWithEmergency(rwarn_pkt->getSource());
                            if(fp.cf_after_rresp) {
                                constructRoutingTable(fp.rresp_req, fp.cf_after_rresp, fp.qos_pdr);
                            } else {
                                constructRoutingTable(fp.rresp_req);
                            }
                        } catch (no_available_entry &e) {
                            trace()<<"[info] Entry not available (not a real error): "<<e.what();
                        }
                    }
                    break;
                }
                case shmrpWarnDef::PATH_FAILURE_EVENT: {
                    trace()<<"[info] PATH_FAILURE_EVENT";
                    if(checkRoute(std::string(rwarn_pkt->getSource()))) {
                        removeRoute(std::string(rwarn_pkt->getSource()));
                        removeRreqEntry(std::string(rwarn_pkt->getSource()));
                        markRinvEntryFail(std::string(rwarn_pkt->getSource()));
                        handleLinkFailure(rwarn_pkt->getPathid());
                    } else if(rreqEntryExists(std::string(rwarn_pkt->getSource()))) {
                        removeRreqEntry(std::string(rwarn_pkt->getSource()));
                        markRinvEntryFail(std::string(rwarn_pkt->getSource()));
                    } else if(checkRinvEntry(std::string(rwarn_pkt->getSource()))) {
                        markRinvEntryFail(std::string(rwarn_pkt->getSource()));
                    }
                    break;
                }
            }
            break;
        }
        case shmrpPacketDef::PING_PACKET: {
            trace()<<"[info] PING_PACKET received";
            sendPong(dynamic_cast<shmrpPingPacket *>(pkt)->getRound());
            handleTSecLTimer();

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
            incPktCountInRecvTable(std::string(data_pkt->getSource()), data_pkt->getPathid(), getRound() );
            if(isSink() && 0==std::strcmp(data_pkt->getDestination(),SELF_NETWORK_ADDRESS)) {
                trace()<<"[info] DATA packet arrived, forwarding to Application layer";
                data_pkt->setSource(data_pkt->getOrigin());
                toApplicationLayer(decapsulatePacket(data_pkt));
                incPktCountInTrafficTable(std::string(data_pkt->getOrigin()), data_pkt->getPathid(), data_pkt->getReroute());
                break;
            } else if(0==std::strcmp(data_pkt->getDestination(), BROADCAST_NETWORK_ADDRESS)) {
                trace()<<"[info] Broadcast packet, forwarding to Application layer";
                toApplicationLayer(decapsulatePacket(data_pkt));
            } else {
                trace()<<"[info] DATA packet at interim node, routing forward";
                if(data_pkt->getOrigin() == std::string(SELF_NETWORK_ADDRESS)) {
                    trace()<<"[error] Loop detected at path: "<<data_pkt->getPathid();
                    auto entry = getNextHop(data_pkt->getPathid());
                    removeRoute(entry);
                    removeRreqEntry(entry);
                    markRinvEntryFail(entry);
                    handleLinkFailure(data_pkt->getPathid());
                    break;



                }
                        
                incPktCountInTrafficTable(std::string(data_pkt->getOrigin()), data_pkt->getPathid(), data_pkt->getReroute());

                std::string next_hop;
                bool reroute=false;
                try {
                    if(shmrpRingDef::EXTERNAL==getRingStatus()) {
                        next_hop=getNextHop(data_pkt->getPathid());
                    } else {
                        next_hop=getNextHop(selectPathid().pathid,fp.rand_ring_hop );
                    }
                } catch (no_available_entry &e) {
                    trace()<<"[error] Next hop not available for pathid: "<<data_pkt->getPathid();
                    if(fp.reroute_pkt) {
                        trace()<<"[info] Rerouting packet";
                        try {
                            next_hop=getNextHop(selectPathid().pathid);
                            if(next_hop == std::string(data_pkt->getSource())) {
                                trace()<<"[error] Possible loop due to reroute, aborting, removing route: "<<next_hop;
                                removeRoute(next_hop);
                                removeRreqEntry(next_hop);
                                markRinvEntryFail(next_hop);

                                if(getRoutingTableSize() == 0) {
                                    trace()<<"[error] Routing table empty";
                                    try {
                                        if(fp.rt_fallb_wo_qos) {
                                            constructRoutingTable(fp.rresp_req, fp.cf_after_rresp, 0.0, true /* update */);
                                        } else {
                                            constructRoutingTable(fp.rresp_req, fp.cf_after_rresp, fp.qos_pdr, true /* update */);
                                        }
                                    } catch (exception &e) {
                                    trace()<<"[error] "<<e.what()<<" returning to INIT";
                                    if(fp.send_pfail_rwarn) {
                                        sendRwarn(shmrpWarnDef::PATH_FAILURE_EVENT,data_pkt->getPathid());
                                    }
                                    setState(shmrpStateDef::INIT);
                                    }

                                }
                                break;
                            }
                            reroute=true;
                        } catch (no_available_entry &e) {
                            trace()<<"[error] "<<e.what();
                            if(fp.send_pfail_rwarn) {
                                sendRwarn(shmrpWarnDef::PATH_FAILURE_EVENT,data_pkt->getPathid());
                            }

                            break;
                        } catch (routing_table_empty &e) {
                            trace()<<"[error] "<<e.what();
                            break;
                        }
                    } else {
                        break;
                    }
                }

                incPktCountInRoutingTable(next_hop);
                forwardData(data_pkt->dup(),next_hop,reroute);
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
        y_out<<YAML::Value;

        y_out<<YAML::BeginSeq;
        for(auto p: i.second.pathid) {
            y_out<<YAML::BeginMap;
            y_out<<YAML::Key<<"pathid";
            y_out<<YAML::Value<<p.pathid;
            y_out<<YAML::Key<<"nmas";
            y_out<<YAML::Value<<p.nmas;
            y_out<<YAML::Key<<"secl";
            y_out<<YAML::Value<<p.secl;
            y_out<<YAML::Key<<"secl_performed";
            y_out<<YAML::Key<<p.secl_performed;
            y_out<<YAML::EndMap;;

        }
        y_out<<YAML::EndSeq;



        y_out<<YAML::Key<<"hop";
        y_out<<YAML::Value<<i.second.hop;
        y_out<<YAML::Key<<"pkt_count";
        y_out<<YAML::Value<<i.second.pkt_count;
        y_out<<YAML::Key<<"ack_count";          
        y_out<<YAML::Value<<i.second.ack_count;
        y_out<<YAML::Key<<"fail_count";
        y_out<<YAML::Value<<i.second.fail_count;
        y_out<<YAML::Key<<"fail";
        y_out<<YAML::Value<<i.second.fail;
        y_out<<YAML::Key<<"secl";
        y_out<<YAML::Value<<i.second.secl;
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
        y_out<<YAML::Key<<"pathid";

        y_out<<YAML::Value;

        y_out<<YAML::BeginSeq;
        for(auto p: i.second.pathid) {
            y_out<<YAML::BeginMap;
            y_out<<YAML::Key<<"pathid";
            y_out<<YAML::Value<<p.pathid;
            y_out<<YAML::Key<<"nmas";
            y_out<<YAML::Value<<p.nmas;
            y_out<<YAML::EndMap;;

        }
        y_out<<YAML::EndSeq;


        y_out<<YAML::Key<<"pkt_count";
        y_out<<YAML::Value<<i.second.pkt_count;
        y_out<<YAML::EndMap;
    }
    y_out<<YAML::EndSeq;

}

void shmrp::serializeRadioStats(PktBreakdown stats) {
    y_out<<YAML::BeginMap;
    y_out<<YAML::Key<<"TX_pkt";
    y_out<<YAML::Value<<stats.transmissions;
    y_out<<YAML::Key<<"RX_ok_no_interf";
    y_out<<YAML::Value<<stats.RxReachedNoInterference;
    y_out<<YAML::Key<<"RX_ok_interf";
    y_out<<YAML::Value<<stats.RxReachedInterference;
    y_out<<YAML::Key<<"RX_fail_no_interf";
    y_out<<YAML::Value<<stats.RxFailedNoInterference;	
    y_out<<YAML::Key<<"RX_fail_interf";
    y_out<<YAML::Value<<stats.RxFailedInterference;
    y_out<<YAML::Key<<"RX_fail_below_sensitivity";
    y_out<<YAML::Value<<stats.RxFailedSensitivity;
    y_out<<YAML::Key<<"RX_fail_wrong_modulation";
    y_out<<YAML::Value<<stats.RxFailedModulation;
    y_out<<YAML::Key<<"RX_fail_not_rx_state";
    y_out<<YAML::Value<<stats.RxFailedNoRxState;
    y_out<<YAML::Key<<"buffer_overflow";
    y_out<<YAML::Value<<stats.bufferOverflow;
    y_out<<YAML::EndMap;
}


void shmrp::finishSpecific() {
    if (isSink()) { // && getParentModule()->getIndex() == 0 ) 
        cTopology *topo;        // temp variable to access energy spent by other nodes
        topo = new cTopology("topo");
        topo->extractByNedTypeName(cStringTokenizer("node.Node").asVector());

        set<int> paths;
        auto sink_pos=dynamic_cast<VirtualMobilityManager *>(topo->getNode(0)->getModule()->getSubmodule("MobilityManager"))->getLocation();

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

        auto radio=dynamic_cast<Radio *>(topo->getNode(0)->getModule()->getSubmodule("Communication")->getSubmodule("Radio"));

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
        y_out<<YAML::Key<<"master";
        y_out<<YAML::Value<<isMaster();
        y_out<<YAML::Key<<"pong_table";
        y_out<<YAML::Value<<getPongTableSize();
        y_out<<YAML::Key<<"radio";
        y_out<<YAML::Value;
        serializeRadioStats(radio->getStats());
            y_out<<YAML::Key<<"traffic_table";
            y_out<<YAML::Value;
            y_out<<YAML::BeginSeq;
            for(auto ne: getTrafficTable()) {
                y_out<<YAML::BeginMap;
                y_out<<YAML::Key<<"node";
                y_out<<YAML::Value<<ne.first;
                y_out<<YAML::Key<<"pkt";
                y_out<<YAML::Value<<ne.second.pkt_count;
                y_out<<YAML::Key<<"pathid";
                y_out<<YAML::Value;
                y_out<<YAML::BeginSeq;
                for(auto p: ne.second.pathid) {
                    y_out<<YAML::BeginMap;
                    y_out<<YAML::Key<<"pathid";
                    y_out<<YAML::Value<<p.pathid;
                    y_out<<YAML::EndMap;
                }
                y_out<<YAML::EndSeq;
                y_out<<YAML::Key<<"reroute_count";
                y_out<<YAML::Value<<ne.second.reroute_count;
                y_out<<YAML::EndMap;
            }
            y_out<<YAML::EndSeq;

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
            try {
                auto table=shmrp_instance->getRreqTable();
                y_out<<YAML::Key<<"rreq_table";
                y_out<<YAML::Value;
                serializeRoutingTable(table);
            } catch (exception &e) {
                trace()<<"[error] No rreq table: "<<e.what();
            }
            try {
                auto table=shmrp_instance->getRinvTable();
                y_out<<YAML::Key<<"rinv_table";
                y_out<<YAML::Value;
                serializeRoutingTable(table);
            } catch (exception &e) {
                trace()<<"[error] No rinv table: "<<e.what();
            }

            y_out<<YAML::Key<<"state";
            y_out<<YAML::Value<<stateToStr(shmrp_instance->getState());
            y_out<<YAML::Key<<"round";
            y_out<<YAML::Value<<shmrp_instance->getRound();
            y_out<<YAML::Key<<"second_learn";
            y_out<<YAML::Value<<shmrp_instance->getSecL();

            auto radio=dynamic_cast<Radio *>(topo->getNode(i)->getModule()->getSubmodule("Communication")->getSubmodule("Radio"));
            y_out<<YAML::Key<<"radio";
            y_out<<YAML::Value;
            serializeRadioStats(radio->getStats());

            auto res_mgr=dynamic_cast<ResourceManager *>(topo->getNode(i)->getModule()->getSubmodule("ResourceManager"));


            // Format: {'Node1': [0,0], 'Node2': [0,10],'Node3':[10,0]}
            auto loc=mob_mgr->getLocation();
            y_out<<YAML::Key<<"x";
            y_out<<loc.x;
            y_out<<YAML::Key<<"y";
            y_out<<loc.y;
            y_out<<YAML::Key;
            y_out<<"role";
            y_out<<YAML::Value;
            y_out<<(res_mgr->isDead()?"dead":ringToStr(shmrp_instance->getRingStatus()));
            y_out<<YAML::Key<<"remaining_energy";
            y_out<<YAML::Value<<res_mgr->getRemainingEnergy();
            y_out<<YAML::Key<<"spent_energy";
            y_out<<YAML::Value<<res_mgr->getSpentEnergy();
            y_out<<YAML::Key<<"pong_table";
            y_out<<YAML::Value<<shmrp_instance->getPongTableSize();
            y_out<<YAML::Key;
            y_out<<"hop";
            y_out<<YAML::Value;
            y_out<<shmrp_instance->getHop();
            y_out<<YAML::Key;
            y_out<<"master";
            y_out<<YAML::Value;
            y_out<<shmrp_instance->isMaster();
            y_out<<YAML::Key<<"traffic_table";
            y_out<<YAML::Value;
            y_out<<YAML::BeginSeq;
            for(auto ne: shmrp_instance->getTrafficTable()) {
                y_out<<YAML::BeginMap;
                y_out<<YAML::Key<<"node";
                y_out<<YAML::Value<<ne.first;
                y_out<<YAML::Key<<"pkt";
                y_out<<YAML::Value<<ne.second.pkt_count;
                y_out<<YAML::Key<<"pathid";
                y_out<<YAML::Value;
                y_out<<YAML::BeginSeq;
                for(auto p: ne.second.pathid) {
                    y_out<<YAML::BeginMap;
                    y_out<<YAML::Key<<"pathid";
                    y_out<<YAML::Value<<p.pathid;
                    y_out<<YAML::EndMap;
                }
                y_out<<YAML::EndSeq;
                y_out<<YAML::Key<<"reroute_count";
                y_out<<YAML::Value<<ne.second.reroute_count;
                y_out<<YAML::EndMap;
            }
            y_out<<YAML::EndSeq;
            y_out<<YAML::EndMap;

            // seek back one character

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
            ys_out<<YAML::EndMap;

        }
        ys_out<<YAML::EndSeq;
        ofstream state_file("state_chng.yaml");
        state_file<<ys_out.c_str();
        state_file.close();


        delete(topo);
    }
    return;
}

void shmrp::handleMacControlMessage(cMessage *msg) {
    trace()<<"[info] Entering handleMacControlMessage()";
    /* Something we should do about buffer overflow */
    MacControlMessage *mac_ctrl_msg=check_and_cast<MacControlMessage *>(msg);
    if(MacControlMessage_type::MAC_BUFFER_FULL == mac_ctrl_msg->getMacControlMessageKind()) {
        trace()<<"[error] MAC buffer full";
        return;
    }
    TMacControlMessage *mac_msg=check_and_cast<TMacControlMessage *>(msg);
    if(!mac_msg) {
        trace()<<"[error] Not TMacControlMessage";
        throw cRuntimeError("[error] Message not a TMacControlMessage");
    }
    trace()<<"[info] Event: "<<mac_msg->getMacControlMessageKind()<<" Node: "<<mac_msg->getDestination()<<" Seqnum: "<<mac_msg->getSeq_num();
    std::string nw_address = std::to_string(mac_msg->getDestination());
    if(MacControlMessage_type::ACK_RECV == mac_msg->getMacControlMessageKind()) {
//        if(rreq_table.find(nw_address) != rreq_table.end() && (shmrpStateDef::ESTABLISH == getState() || shmrpStateDef::S_ESTABLISH == getState() ) ){
//            rreq_table[nw_address].ack_count++;
//        }
        if(routing_table.find(nw_address) != routing_table.end()) {
            routing_table[nw_address].ack_count++;
            if(fp.detect_link_fail) {
                trace()<<"[info] Resetting fail_count";
                routing_table[nw_address].fail_count = 0;
            }
        }
    }
    if(MacControlMessage_type::PKT_FAIL == mac_msg->getMacControlMessageKind()) {
        if(routing_table.find(nw_address) != routing_table.end()) {
            routing_table[nw_address].fail_count++;
            if(fp.detect_link_fail && fp.fail_count <= static_cast<int>(static_cast<double>(routing_table[nw_address].fail_count))/fp.qos_pdr && getSecL() && getHop() > 2 && getState() == shmrpStateDef::WORK) { // Ugly :-(
                trace()<<"[info] Link "<<nw_address<<" failed, removing";
                auto p=routing_table[nw_address].pathid;
                removeRoute(nw_address);
                removeRreqEntry(nw_address);
                markRinvEntryFail(nw_address);
                handleLinkFailure(p[0].pathid);
            }
        }
    }
    delete msg;
}

bool shmrp::secLPerformed(int round, int pathid) {
    trace()<<"[info] Entering secLPerformed(round="<<round<<", pathid="<<pathid<<")";
    bool performed=true;
    for(auto ne: recv_table) {
        if(ne.second.round == round) {
            for(auto p: ne.second.pathid) {
                if(p.pathid==pathid && ne.second.secl == false) {
                    trace()<<"[info] "<<ne.second.nw_address<<" second learn still missing";
                    performed=false;
                }
            }
        }
    }
    return performed;
}



void shmrp::handleNetworkControlCommand(cMessage *msg) {
    trace()<<"[info] Entering handleNetworkControlCommand()";
    EmergencyMessage *app_msg=check_and_cast<EmergencyMessage *>(msg);
    if(!msg) {
        trace()<<"[error] Unknown Network Control Command Message";
    }
    if(MsgType::EMERGENCY == app_msg->getEvent()) {
        trace()<<"[info] Application in Emergency state, start local re-learn";
        sendRwarn();
    }
}


void shmrp::sendRwarn() {
    trace()<<"[info] Entering sendRwarn()";
    sendRwarn(shmrpWarnDef::EMERGENCY_EVENT,selectPathid(true).pathid);
}

void shmrp::sendRwarn(shmrpWarnDef cause, int pathid) {
    trace()<<"[info] Entering sendRwarn(cause="<<cause<<", pathid="<<pathid<<")";
    shmrpRwarnPacket *warn_pkt=new shmrpRwarnPacket("SHMRP RWARN packet", NETWORK_LAYER_PACKET);
    warn_pkt->setByteLength(netDataFrameOverhead);
    warn_pkt->setCause(cause);
    warn_pkt->setShmrpPacketKind(shmrpPacketDef::RWARN_PACKET);
    warn_pkt->setSource(SELF_NETWORK_ADDRESS);
    warn_pkt->setDestination(BROADCAST_NETWORK_ADDRESS);
    warn_pkt->setRound(getRound());
    warn_pkt->setPathid(pathid);
    warn_pkt->setHop(getHop());
    warn_pkt->setSequenceNumber(currentSequenceNumber++);
    toMacLayer(warn_pkt, BROADCAST_MAC_ADDRESS);

}


void shmrp::incPktCountInTrafficTable(std::string node, int pathid, int reroute) {
    if(traffic_table.find(node) == traffic_table.end()) {
        traffic_table[node].nw_address=node;
        traffic_table[node].pkt_count=1;
        traffic_table[node].reroute_count=reroute;
        traffic_table[node].pathid.push_back({pathid,0});
    } else {
        ++(traffic_table[node].pkt_count);
        traffic_table[node].reroute_count+=reroute;
        bool add=true;
        for(auto ne: traffic_table[node].pathid) {
            if(ne.pathid==pathid) {
                add=false;
            }
        }
        if(add) {
            traffic_table[node].pathid.push_back({pathid,0});
        }
    }
}

void shmrp::handleLinkFailure(int p) {
    trace()<<"[info] Entering handleLinkFailure()";
    try {
        if(fp.rt_fallb_wo_qos) {
            constructRoutingTable(fp.rresp_req, fp.cf_after_rresp, 0, true /* update */);
        } else {
            constructRoutingTable(fp.rresp_req, fp.cf_after_rresp, fp.qos_pdr, true /* update */);
        }
    } catch (exception &e) {
        trace()<<"[error] "<<e.what()<<" returning to INIT";
        setState(shmrpStateDef::INIT);
        if(fp.send_pfail_rwarn) {
            sendRwarn(shmrpWarnDef::PATH_FAILURE_EVENT,p);
        }
    }
}

double shmrp::getEnergyValue() {
    if(ff_app==NULL) {
        return 1.0;
    }
    return ff_app->getEnergyValue();
}

double shmrp::getEmergencyValue() {
    if(ff_app==NULL) {
        return 1.0;
    }
    return ff_app->getEmergencyValue();
}

