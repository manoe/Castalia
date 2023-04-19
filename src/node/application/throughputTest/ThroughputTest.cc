/****************************************************************************
 *  Copyright: National ICT Australia,  2007 - 2011                         *
 *  Developed at the ATP lab, Networked Systems research theme              *
 *  Author(s): Athanassios Boulis, Yuriy Tselishchev                        *
 *  This file is distributed under the terms in the attached LICENSE file.  *
 *  If you do not find this file, copies can be found by writing to:        *
 *                                                                          *
 *      NICTA, Locked Bag 9013, Alexandria, NSW 1435, Australia             *
 *      Attention:  License Inquiry.                                        *
 *                                                                          *
 ****************************************************************************/

#include "node/application/throughputTest/ThroughputTest.h"

Define_Module(ThroughputTest);

void ThroughputTest::startup()
{
	packet_rate = par("packet_rate");
	recipientAddress = par("nextRecipient").stringValue();
	recipientId = atoi(recipientAddress.c_str());
	startupDelay = par("startupDelay");
	delayLimit = par("delayLimit");
	packet_spacing = packet_rate > 0 ? 1 / float (packet_rate) : -1;
	dataSN = 0;
	
    periodic_measurement = par("periodic_measurement");
    meas_period = par("meas_period");
    meas_queue_length = par("meas_queue_length");

	numNodes = getParentModule()->getParentModule()->par("numNodes");
	packetsSent.clear();
	packetsReceived.clear();
	bytesReceived.clear();

    if(isSink && periodic_measurement) {
        y_per_out<<YAML::BeginSeq;
        initMeasQueues();
        setTimer(PERIODIC_MEAS, meas_period);
    }



	if (packet_spacing > 0 && recipientAddress.compare(SELF_NETWORK_ADDRESS) != 0)
		setTimer(SEND_PACKET, packet_spacing + startupDelay);
	else
		trace() << "Not sending packets";

	declareOutput("Packets received per node");
}

bool ThroughputTest::isPacketSeen(int source, int sn) {
    if(packetsSeen.find(source) == packetsSeen.end()) {
        packetsSeen[source].insert(sn);
        return false;
    }
    if(packetsSeen[source].find(sn) == packetsSeen[source].end()) {
        packetsSeen[source].insert(sn);
        return false;
    }
    return true;
}

void ThroughputTest::initMeasQueues() {
    for(int i=1 ; i < numNodes ; ++i) {
        measQueues[i].clear();
    }
}

void ThroughputTest::addToMeasQueue(int source, int sn) {
    measQueues[source].push_back(sn);
    if(measQueues[source].size() > meas_queue_length) {
        measQueues[source].pop_front();
    } 
}

void ThroughputTest::addToSentQueue(int sn, double time) {
    sentQueue.push_back({sn,time});
    if(sentQueue.size() > meas_queue_length) {
        sentQueue.pop_front();
    }
}

void ThroughputTest::fromNetworkLayer(ApplicationPacket * rcvPacket,
		const char *source, double rssi, double lqi)
{
	int sequenceNumber = rcvPacket->getSequenceNumber();
	int sourceId = atoi(source);

	// This node is the final recipient for the packet
	if (recipientAddress.compare(SELF_NETWORK_ADDRESS) == 0) {
		if (delayLimit == 0 || (simTime() - rcvPacket->getCreationTime()) <= delayLimit) { 
			trace() << "Received packet #" << sequenceNumber << " from node " << source;
			collectOutput("Packets received per node", sourceId);
            if(!isPacketSeen(sourceId,sequenceNumber)) {
                if(periodic_measurement) {
                    addToMeasQueue(sourceId,sequenceNumber);
                }
    			packetsReceived[sourceId]++;
	    		bytesReceived[sourceId] += rcvPacket->getByteLength();
            } else {
                trace()<<"Packet already received";
            }

		} else {
			trace() << "Packet #" << sequenceNumber << " from node " << source <<
				" exceeded delay limit of " << delayLimit << "s";
		}
	// Packet has to be forwarded to the next hop recipient
	} else {
		ApplicationPacket* fwdPacket = rcvPacket->dup();
		// Reset the size of the packet, otherwise the app overhead will keep adding on
		fwdPacket->setByteLength(0);
		toNetworkLayer(fwdPacket, recipientAddress.c_str());
	}
}

int ThroughputTest::countPackets(deque<int> recv_queue, deque<pkt_stat> sent_queue) {
    int count=0;
    for(auto it=sent_queue.begin() ; it != sent_queue.end() ; ++it) {
        for(auto it2=recv_queue.begin() ; it2 != recv_queue.end() ; ++it2) {
            if(it->sn==*it2) {
                ++count;
                break;
            }
        }
    }
    return count;
}

deque<pkt_stat> ThroughputTest::getSentQueueFromNode(int node) {
	cTopology *topo;	// temp variable to access packets received by other nodes
	topo = new cTopology("topo");
	topo->extractByNedTypeName(cStringTokenizer("node.Node").asVector());
	long bytesDelivered = 0;
    ThroughputTest *appModule = dynamic_cast<ThroughputTest*>
			(topo->getNode(node)->getModule()->getSubmodule("Application"));
    delete topo;
    return appModule->getSentQueue();
}

void ThroughputTest::timerFiredCallback(int index)
{
	switch (index) {
		case SEND_PACKET:{
			trace() << "Sending packet #" << dataSN;
			toNetworkLayer(createGenericDataPacket(0, dataSN), recipientAddress.c_str());
			packetsSent[recipientId]++;
            addToSentQueue(dataSN,simTime().dbl());
			dataSN++;
			setTimer(SEND_PACKET, packet_spacing);
			break;
		} case PERIODIC_MEAS: {
            y_per_out<<YAML::BeginMap;
            y_per_out<<YAML::Key<<"timestamp";
            y_per_out<<YAML::Value<<simTime().dbl();
            y_per_out<<YAML::Key<<"pdr";
            y_per_out<<YAML::Value<<YAML::BeginSeq;
            for(auto queue : measQueues) {
                y_per_out<<YAML::BeginMap;
                y_per_out<<YAML::Key<<"node";
                y_per_out<<YAML::Value<<queue.first;
                y_per_out<<YAML::Key<<"pdr";
                y_per_out<<YAML::Value<<static_cast<double>(countPackets(queue.second,getSentQueueFromNode(queue.first)))/static_cast<double>(getSentQueueFromNode(queue.first).size());
                y_per_out<<YAML::Key<<"recv";
                y_per_out<<YAML::Value<<countPackets(queue.second,getSentQueueFromNode(queue.first));
                y_per_out<<YAML::Key<<"sent";
                y_per_out<<YAML::Value<<getSentQueueFromNode(queue.first).size();
                y_per_out<<YAML::EndMap;
                trace()<<"[info] Counted packets for node "<<queue.first<<": "<<countPackets(queue.second,getSentQueueFromNode(queue.first));
            }
            y_per_out<<YAML::EndSeq;
            y_per_out<<YAML::EndMap;
            setTimer(PERIODIC_MEAS, meas_period);
            break;
        }
	}
}

// This method processes a received carrier sense interupt. Used only for demo purposes
// in some simulations. Feel free to comment out the trace command.
void ThroughputTest::handleRadioControlMessage(RadioControlMessage *radioMsg)
{
	switch (radioMsg->getRadioControlMessageKind()) {
		case CARRIER_SENSE_INTERRUPT:
			trace() << "CS Interrupt received! current RSSI value is: " << radioModule->readRSSI();
                        break;
	}
}

void ThroughputTest::finishSpecific() {
	declareOutput("Packets reception rate");
	declareOutput("Packets loss rate");

    YAML::Emitter y_out;

    // YAML structure
    // seed: seednum
    // pdr:
    //      node: pdrval
    //      node2: pdrval
    if(isSink) {
        y_out<<YAML::BeginMap;
        y_out<<YAML::Key<<"seed";
        auto env_mod=getEnvir();
        auto conf_mod=env_mod->getConfig();
        y_out<<YAML::Value<<conf_mod->getConfigValue("seed-set");
        y_out<<YAML::Key<<"pdr";
        y_out<<YAML::BeginMap;
    }


	cTopology *topo;	// temp variable to access packets received by other nodes
	topo = new cTopology("topo");
	topo->extractByNedTypeName(cStringTokenizer("node.Node").asVector());
	long bytesDelivered = 0;
	for (int i = 0; i < numNodes; i++) {
		ThroughputTest *appModule = dynamic_cast<ThroughputTest*>
			(topo->getNode(i)->getModule()->getSubmodule("Application"));
		if (appModule) {
			int packetsSent = appModule->getPacketsSent(self);
            // else branch: write 0
			if (packetsSent > 0) { // this node sent us some packets
				float rate = (float)packetsReceived[i]/(float)packetsSent;
				collectOutput("Packets reception rate", i, "total", rate);
				collectOutput("Packets loss rate", i, "total", 1-rate);
                trace()<<"Pkt received: "<<packetsReceived[i]<<" sent: "<<packetsSent;

                // write yaml
                if(isSink) {
                    //y_out<<YAML::Key<<i<<YAML::Value<<rate;
                    y_out<<YAML::Key<<i;
                    y_out<<YAML::Value;
                    y_out<<YAML::BeginMap;
                    y_out<<YAML::Key<<"pkt_recv";
                    y_out<<YAML::Value<<packetsReceived[i];
                    y_out<<YAML::Key<<"pkt_sent";
                    y_out<<YAML::Value<<packetsSent;
                    y_out<<YAML::EndMap;
                }

			} else {
                if(isSink) {
                    y_out<<YAML::Key<<i<<YAML::Value<<0;
                }
            }

			bytesDelivered += appModule->getBytesReceived(self);
		}
	}
	delete(topo);
    if(isSink) {
        y_out<<YAML::EndMap;
        y_out<<YAML::EndMap;
        ofstream pdr_file("pdr.yaml");
        pdr_file<<y_out.c_str();
        pdr_file.close();
        if(periodic_measurement) {
            y_per_out<<YAML::EndSeq;
            ofstream periodic_file("per_pdr.yaml");
            periodic_file<<y_per_out.c_str();
            periodic_file.close();

        }
    }

	if (packet_rate > 0 && bytesDelivered > 0) {
		double energy = (resMgrModule->getSpentEnergy() * 1000000000)/(bytesDelivered * 8);	//in nanojoules/bit
		declareOutput("Energy nJ/bit");
		collectOutput("Energy nJ/bit","",energy);
	}
}
