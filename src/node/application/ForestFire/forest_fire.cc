/*******************************************************************************
 *  Copyright: Balint Aron Uveges, 2022                                        *
 *  Developed at Pazmany Peter Catholic University,                            *
 *               Faculty of Information Technology and Bionics                 *
 *  Author(s): Balint Aron Uveges                                              *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *                                                                             *
 *******************************************************************************/

#include "node/application/ForestFire/forest_fire.h"

Define_Module(ForestFire);

void ForestFire::startup()
{
    reportTreshold = par("reportTreshold");
    //	sampleInterval = (double)par("sampleInterval") / 1000;
    sampleInterval = (double)par("sampleInterval");

    reportDestination = par("reportDestination").stringValue();
    sampleSize = par("sampleSize");

    currentVersionPacket = 0;
    currentVersion = 0;
    currSampleSN = 1;
    outOfEnergy = 0;

    maxPayload = par("maxPayloadPacketSize");
    div_t tmp_div = div(maxPayload, sampleSize);
    maxSampleAccumulated = tmp_div.quot * sampleSize;

    version_info_table.clear();
    report_info_table.clear();

    report_period=par("reportPeriod"); 
    event_period=par("eventPeriod");
    emergency_threshold=par("emergencyThreshold");
    emergency_broadcast=par("emergencyBroadcastPeriod");

    rm=dynamic_cast<ResourceManager *>(getParentModule()->getSubmodule("ResourceManager"));

    report_timer_offset=par("report_timer_offset");
    if (!isSink) {
        setTimer(ForestFireTimers::REQUEST_SAMPLE, sampleInterval);
        if(report_timer_offset) {
            setTimer(ForestFireTimers::REPORT_PERIOD, sampleInterval+std::atof(selfAddress.c_str()));
        }
        else {
            setTimer(ForestFireTimers::REPORT_PERIOD, sampleInterval);
        }
    }
    emergency=false;
    declareOutput("Report packet");
    declareOutput("Event packet");

    event_sent=0;
    report_sent=0;
    reportRecv.clear();
    eventRecv.clear();
    pot_field   = par("pot_field"); 
    d_max       = par("d_max");
    d_high      = par("d_high");
    d_gamma     = par("d_gamma");
    srlz_pkt_arr= par("srlz_pkt_arr");
    srlz_nrg    = par("srlz_nrg");
    t_srlz_nrg  = par("t_srlz_nrg");

    if(srlz_pkt_arr) {
        yp_out<<YAML::BeginMap;
        yp_out<<YAML::Key<<"pkt_list";
        yp_out<<YAML::BeginSeq;
    }

    if(srlz_nrg) {
        yn_out<<YAML::BeginMap;
        yn_out<<YAML::Key<<"nrg_list";
        yn_out<<YAML::BeginSeq;
        setTimer(ForestFireTimers::SRLZ_NRG, t_srlz_nrg);
    }
}

int ForestFire::getEventSent() {
    return event_sent;
}

int ForestFire::getReportSent() {
    return report_sent;
}

map<int,int> ForestFire::getReportRecv() {
    return reportRecv; 
}

map<int,int> ForestFire::getEventRecv() {
    return eventRecv;
}

map<int,set<int>> ForestFire::getReportPacketsSeen() {
    return reportPacketsSeen;
}

map<int,set<int>> ForestFire::getEventPacketsSeen() {
    return eventPacketsSeen;
}



void ForestFire::sendEvent() {
    ForestFirePacket *newPkt = new ForestFirePacket("ForestFire event packet", APPLICATION_PACKET);
    newPkt->setName(EVENT_PACKET_NAME);
    newPkt->setData(sensedValue);
    newPkt->setSequenceNumber(currSampleSN);
    newPkt->setForestFirePacketKind(ForestFirePacketDef::PERIODIC_REPORT_PACKET);
    newPkt->setByteLength(2);
    toNetworkLayer(newPkt, reportDestination.c_str());
    currSampleSN++;
    collectOutput("Event packet","Sent");
    event_sent++;
}

void ForestFire::sendReport() {
    ForestFirePacket *newPkt = new ForestFirePacket("ForestFire report packet", APPLICATION_PACKET);
    newPkt->setName(REPORT_PACKET_NAME);
    newPkt->setData(sensedValue);
    newPkt->setSequenceNumber(currSampleSN);
    newPkt->setForestFirePacketKind(ForestFirePacketDef::PERIODIC_REPORT_PACKET);
    newPkt->setByteLength(2);
    toNetworkLayer(newPkt, reportDestination.c_str());
    currSampleSN++;
    collectOutput("Report packet","Sent");
    report_sent++;
}

void ForestFire::sendEmergencyBroadcast() {
    ForestFirePacket *newPkt = new ForestFirePacket("ForestFire emergency broadcast packet", APPLICATION_PACKET);
    newPkt->setName(BROADCAST_PACKET_NAME);
    newPkt->setData(sensedValue);
    newPkt->setSequenceNumber(currSampleSN);
    newPkt->setForestFirePacketKind(ForestFirePacketDef::EMERGENCY_BROADCAST_PACKET);
    newPkt->setByteLength(2);
    toNetworkLayer(newPkt, BROADCAST_NETWORK_ADDRESS);
    currSampleSN++;

}


bool ForestFire::isPacketSeen(int source, int sn, std::string name) {
    map<int,set<int>>* ptr = nullptr;
    if(name.compare(REPORT_PACKET_NAME)==0) {
        ptr=&reportPacketsSeen;
    } else if(name.compare(EVENT_PACKET_NAME)==0) {
        ptr=&eventPacketsSeen;
    } else {
        throw std::string("Unknown packet name");
    }
    if(ptr->find(source) == ptr->end()) {
        ptr->insert({source,{sn}});
        return false;
    }
    if(ptr->at(source).find(sn) == ptr->at(source).end()) {
        ptr->at(source).insert(sn);
        return false;
    }
    return true;
}


void ForestFire::timerFiredCallback(int timer)
{
    switch (timer) {
        case ForestFireTimers::REQUEST_SAMPLE:{
            trace()<<"REQUEST_SAMPLE timer expired";
            setTimer(REQUEST_SAMPLE, sampleInterval);
            requestSensorReading();
            break;
        }
        case ForestFireTimers::EVENT_PERIOD: {
            trace()<<"EVENT_PERIOD timer expired";
            sendEvent();
            setTimer(EVENT_PERIOD, event_period);
            break;
        }
        case ForestFireTimers::REPORT_PERIOD: {
            trace()<<"REPORT_PERIOD timer expired";
            sendReport();
            setTimer(REPORT_PERIOD, report_period);
            break;
        }
        case ForestFireTimers::EMERGENCY_BROADCAST: {
            trace()<<"EMERGENCY_BROADCAST timer expired";
            sendEmergencyBroadcast();
            setTimer(EMERGENCY_BROADCAST,emergency_broadcast);
            break;
        }
        case ForestFireTimers::SRLZ_NRG: {
            trace()<<"SRLZ_NRG timer expired";
            serializeEnergy();
            setTimer(ForestFireTimers::SRLZ_NRG, t_srlz_nrg);
            break;
        }
    }
}



void ForestFire::fromNetworkLayer(ApplicationPacket * rcvPacket,
        const char *source, double rssi, double lqi)
{
    string packetName(rcvPacket->getName());

    double data = rcvPacket->getData();
    int sequenceNumber = rcvPacket->getSequenceNumber();

    if(packetName.compare(BROADCAST_PACKET_NAME) == 0) {
        trace()<<"Emergency broadcast message received.";
        if(!emergency && getTimer(EVENT_PERIOD)==-1) {
            trace()<<"Start EVENT_PERIOD timer based on broadcast message";
            setTimer(EVENT_PERIOD, event_period);
            sendEvent();
        }
	return;
    }
    if(!isPacketSeen(atoi(source),rcvPacket->getSequenceNumber(),rcvPacket->getName() )) {
        if (packetName.compare(REPORT_PACKET_NAME) == 0) {
            collectOutput("Report packet","Received");
            reportRecv[atoi(source)]++;

            if(srlz_pkt_arr) {
                yp_out<<YAML::BeginMap;
                yp_out<<YAML::Key<<"source";
                yp_out<<YAML::Value<<source;
                yp_out<<YAML::Key<<"timestamp";
                yp_out<<YAML::Value<<simTime().dbl();
                yp_out<<YAML::Key<<"energy";
                yp_out<<YAML::Value<<getAverageSpentEnergy();
                yp_out<<YAML::EndMap;
            }

            // this is report packet which contains sensor reading information
            // NOTE that data field is used to store source address instead of using char *source
            // this is done because some routing and flooding is done on the application layer
            // and source address will not always correctly represent the author of the sensed data
            //		trace() << "Received report from " << (int)data;
            //		if (updateReportTable((int)data, sequenceNumber)) {
            //			// forward the packet only if we broadcast reports and this is a new (unseen) report
            //			// updateReportTable returns 0 for duplicate packets
            //			if (!isSink) {
            //				trace() << "Forwarding report packet from node " << (int)data;
            //				toNetworkLayer(rcvPacket->dup(), reportDestination.c_str());
            //			}
            //		}

        }// else if (packetName.compare(REPROGRAM_PACKET_NAME) == 0) {
         // this is version (reprogramming) packet
         //		if (!isSink && updateVersionTable(data, sequenceNumber)) {
         // forward the packet only if not sink and its a new packet
         // updateVersionTable returns 0 for duplicate packets
         //			toNetworkLayer(rcvPacket->dup(), BROADCAST_NETWORK_ADDRESS);
         //		}

         //	}
        else    if(0==packetName.compare(EVENT_PACKET_NAME)) {
            collectOutput("Event packet","Received");
            eventRecv[atoi(source)]++;

        }
        else {
            trace() << "unknown packet received: [" << packetName << "]";
        }
    } else {
        trace()<<"Duplicated packet";
    }
}

void ForestFire::alertRouting() {
    auto *msg=new EmergencyMessage("alert Routing", NETWORK_CONTROL_COMMAND);
    msg->setEvent(MsgType::EMERGENCY);
    toNetworkLayer(msg);
}

void ForestFire::handleSensorReading(SensorReadingMessage * sensorMsg)
{
    string sensType(sensorMsg->getSensorType());
    sensedValue = sensorMsg->getSensedValue();

    trace()<<"Sensed value: "<<sensedValue;
    if(sensedValue >= emergency_threshold && !emergency) {
        trace()<<"Node enters emergency state based on sensor reading.";
        emergency=true;
        sendEvent();
        cancelTimer(EVENT_PERIOD);
        setTimer(EVENT_PERIOD, event_period);
        sendEmergencyBroadcast();
        cancelTimer(EMERGENCY_BROADCAST);
        setTimer(EMERGENCY_BROADCAST,emergency_broadcast);
        alertRouting();
    }
    //if (isSink) {
    //	trace() << "Sink recieved SENSOR_READING (while it shouldnt) "
    //	    << sensValue << " (int)" << (int)sensValue;
    //	return;
    //}
    //
    //if (sensValue < reportTreshold) {
    //	trace() << "Sensed value " << sensValue << " is less than the treshold ("
    //		<< reportTreshold << "), discarding";
    //	return;
    //}
    //
    //currentSampleAccumulated += sampleSize;
    //if (currentSampleAccumulated < maxSampleAccumulated) {
    //	trace() << "Accumulated " << currentSampleAccumulated << "/" << maxSampleAccumulated << " bytes of samples";
    //	return;
    //}
    //

}

double ForestFire::getAverageSpentEnergy() {
    int numNodes = getParentModule()->getParentModule()->par("numNodes");
    cTopology *topo;	// temp variable to access packets received by other nodes
    topo = new cTopology("topo");
    topo->extractByNedTypeName(cStringTokenizer("node.Node").asVector());
    double nrg=0.0;
    for (int i = 0; i < numNodes; i++) {
        auto *rm = dynamic_cast<ResourceManager*>
            (topo->getNode(i)->getModule()->getSubmodule("ResourceManager"));
        nrg+=rm->getSpentEnergy();
    }
    return nrg;
}

void ForestFire::serializeEnergy() {
    int numNodes = getParentModule()->getParentModule()->par("numNodes");
    cTopology *topo;	// temp variable to access packets received by other nodes
    topo = new cTopology("topo");
    topo->extractByNedTypeName(cStringTokenizer("node.Node").asVector());
    yn_out<<YAML::BeginMap;
    yn_out<<YAML::Key<<"timestamp";
    yn_out<<YAML::Value<<simTime().dbl();
    yn_out<<YAML::Key<<"nodes";
    yn_out<<YAML::BeginSeq;
    for (int i = 0; i < numNodes; i++) {
        auto *rm = dynamic_cast<ResourceManager*>
            (topo->getNode(i)->getModule()->getSubmodule("ResourceManager"));
        yn_out<<YAML::BeginMap;
        yn_out<<YAML::Key<<"node";
        yn_out<<YAML::Value<<i;
        yn_out<<YAML::Key<<"energy";
        yn_out<<YAML::Value<<rm->getRemainingEnergy();
        yn_out<<YAML::Key<<"state";
        yn_out<<YAML::Value<<(rm->isDead()?"dead":"live");
        yn_out<<YAML::Key<<"report_sent";
        auto app = dynamic_cast<ForestFire *>(topo->getNode(i)->getModule()->getSubmodule("Application"));

        yn_out<<YAML::Value<<app->getReportSent();
        yn_out<<YAML::Key<<"event_sent";
        yn_out<<YAML::Value<<app->getEventSent();
        yn_out<<YAML::Key<<"report_recv";
        if(reportRecv.find(i) != reportRecv.end()) {
            yn_out<<YAML::Value<<reportRecv[i];
        } else {
            yn_out<<YAML::Value<<0;
        }
        yn_out<<YAML::Key<<"event_recv";
        if(eventRecv.find(i) != eventRecv.end()) {
            yn_out<<YAML::Value<<eventRecv[i];
        } else {
            yn_out<<YAML::Value<<0;
        }
        yn_out<<YAML::EndMap;
    }
    yn_out<<YAML::EndSeq;
    yn_out<<YAML::EndMap;

}

map<int,int> ForestFire::summarizeSentPkts(std::vector<map<int,set<int>>> pkts) {
    map<int, int> out;
    map<int,set<int>> nodes;

    for(auto sink: pkts) {
        for(auto node: sink) {
            if(nodes.find(node.first) != nodes.end()) {
                nodes[node.first].insert(node.second.begin(), node.second.end());
            } else {
                nodes[node.first]=node.second;
            }
        } 
    }
    for(auto node: nodes) {
        out[node.first]=node.second.size();
    }
    return out;
}

void ForestFire::finishSpecific()
{
    declareOutput("Event reception rate");
    declareOutput("Report reception rate");
    if(isSink && 0 == self) {
        int numNodes = getParentModule()->getParentModule()->par("numNodes");

        YAML::Emitter y_out;
        y_out<<YAML::BeginMap;
        y_out<<YAML::Key<<"seed";
        auto env_mod=getEnvir();
        auto conf_mod=env_mod->getConfig();
        y_out<<YAML::Value<<conf_mod->getConfigValue("seed-set");
        y_out<<YAML::Key<<"protocol";
        y_out<<YAML::Value<<getParentModule()->getSubmodule("Communication")->par("RoutingProtocolName").stringValue();
        y_out<<YAML::Key<<"pdr";
        y_out<<YAML::BeginSeq;

        cTopology *topo;	// temp variable to access packets received by other nodes
        topo = new cTopology("topo");
        topo->extractByNedTypeName(cStringTokenizer("node.Node").asVector());
        std::vector<map<int,int>> report_recvs;
        std::vector<map<int,int>> event_recvs;
        std::vector<map<int,set<int>>> report_pkts;
        std::vector<map<int,set<int>>> event_pkts;

        for (int i = 0 ; i < numNodes ; i++) {
            ForestFire *appModule = dynamic_cast<ForestFire*>
                (topo->getNode(i)->getModule()->getSubmodule("Application"));
            if (appModule && ( appModule->hasPar("isSink") ? appModule->par("isSink") : false)) {
                report_recvs.push_back(appModule->getReportRecv());
                event_recvs.push_back(appModule->getEventRecv());
                report_pkts.push_back(appModule->getReportPacketsSeen());
                event_pkts.push_back(appModule->getEventPacketsSeen());
            }
        }
        auto report_pkt_sum = summarizeSentPkts(report_pkts);
        auto event_pkt_sum = summarizeSentPkts(event_pkts);
        for (int i = 0; i < numNodes; i++) {
            y_out<<YAML::BeginMap;
            y_out<<YAML::Key<<"node";
            y_out<<YAML::Value<<i;
            ForestFire *appModule = dynamic_cast<ForestFire*>
                (topo->getNode(i)->getModule()->getSubmodule("Application"));
            if (appModule) {
                int reportSent = appModule->getReportSent();
                int eventSent  = appModule->getEventSent();
                if (reportSent > 0 ) { // this node sent us some packets
                    int report_recv = 0;
                    for(auto r : report_recvs) {
                        if(r.find(i) != r.end()) {
                            report_recv+=r[i];
                        }
                    }

                    float rate = (float)report_recv/(float)reportSent;
                    float rate_unique=(float)report_pkt_sum[i]/(float)reportSent;
                   collectOutput("Report reception rate", i, "total", rate);
                    y_out<<YAML::Key<<"report_pdr";
                    y_out<<YAML::Value<<rate;
                    y_out<<YAML::Key<<"report_pdr_new";
                    y_out<<YAML::Value<<rate_unique;
                }
                if (eventSent > 0) {
                    int event_recv = 0;
                    for(auto e : event_recvs) {
                        if(e.find(i) != e.end()) {
                            event_recv+=e[i];
                        }
                    }

                    float rate = (float)event_recv/(float)eventSent;
                    float rate_unique=(float)event_pkt_sum[i]/(float)eventSent;
                    collectOutput("Event reception rate", i, "total", rate);
                    y_out<<YAML::Key<<"event_pdr";
                    y_out<<YAML::Value<<rate;
                    y_out<<YAML::Key<<"event_pdr_new";
                    y_out<<YAML::Value<<rate_unique;
                }

            }
            y_out<<YAML::EndMap;
        }
        delete(topo);
        y_out<<YAML::EndSeq;
        y_out<<YAML::EndMap;
        ofstream pdr_file("pdr.yaml");
        pdr_file<<y_out.c_str();
        pdr_file.close();

        if(srlz_pkt_arr) {
            yp_out<<YAML::EndSeq;
            yp_out<<YAML::EndMap;
            ofstream pkt_file("pkt.yaml");
            pkt_file<<yp_out.c_str();
            pkt_file<<std::endl;
            pkt_file.close();
        }

        if(srlz_nrg) {
            yn_out<<YAML::EndSeq;
            yn_out<<YAML::EndMap;
            ofstream nrg_file("nrg.yaml");
            nrg_file<<yn_out.c_str();
            nrg_file<<std::endl;
            nrg_file.close();
        }

        if (isSink) {
            declareOutput("Report reception");
            for (int i = 0; i < (int)report_info_table.size(); i++) {
                collectOutput("Report reception", report_info_table[i].source,
                        "Success", report_info_table[i].parts.size());
                collectOutput("Report reception", report_info_table[i].source,
                        "Fail", report_info_table[i].seq - report_info_table[i].parts.size());
            }
        }
    }
}

double ForestFire::getEnergyValue() {
    return rm->getRemainingEnergy()/rm->getMaximumCapacity();
}

double ForestFire::getEmergencyValue() {
    double ret_val=0.0;
    if(d_high>=sensedValue) {
        ret_val=1.0;
    } else if(d_high<sensedValue && d_max>= sensedValue ) {
        trace()<<"sensedValue: "<<sensedValue<<", result: "<<(d_max-sensedValue)/(d_max-d_high);
        return (d_max-sensedValue)/(d_max-d_high);
    } else {
        return 0.0;
    }
    return ret_val;
}
