/****************************************************************************
 *                                                                          *
 * Author: Balint Uveges                                                    *
 *                                                                          *  
 ****************************************************************************/

#include "node/mobilityManager/discreteMobilityManager/DiscreteMobilityManager.h"

Define_Module(DiscreteMobilityManager);

void DiscreteMobilityManager::initialize()
{
	VirtualMobilityManager::initialize();
    dm_delay = par("dm_delay");
}

void DiscreteMobilityManager::handleMessage(cMessage * msg)
{
	int msgKind = msg->getKind();
	switch (msgKind) {
        case MobilityManagerMessageType::DISCRETE_MOBILITY: {
            if(timer_armed==true) {
                trace()<<"[error] Timer already armed";
                DiscreteMobilityManagerMessageNack *dm_nack_msg=new DiscreteMobilityManagerMessageNack();
                dm_nack_msg->setKind(MobilityManagerMessageType::DISCRETE_MOBILITY_NACK);
                send(dm_nack_msg,"toApplicationModule");
                break;
            }
            DiscreteMobilityManagerMessage *dis_msg=check_and_cast<DiscreteMobilityManagerMessage *>(msg);
            trace()<<"Current location (x:y:z) is: ("<<nodeLocation.x<<":"<<nodeLocation.y<<":"<<nodeLocation.z<<")";
            j_x = dis_msg->getX();
            j_y = dis_msg->getY();
            j_z = nodeLocation.z;

            trace() << "Change location (x:y:z) to (" << j_x << 
					":" << j_y << ":" << j_z<<") scheduled";

            scheduleAt(simTime()+dm_delay, new cMessage("Mobility delay timer", TIMER_SERVICE));
            timer_armed=true;
            break;

        }
        case TIMER_SERVICE: {
            trace()<<"[info] Timer expired";
            if(timer_armed==false) {
                trace()<<"[error] Timer not armed?";
                break;
            }
            timer_armed=false;
            trace()<<"Current location (x:y:z) is: ("<<nodeLocation.x<<":"<<nodeLocation.y<<":"<<nodeLocation.z<<")";

            nodeLocation.x = j_x;
            nodeLocation.y = j_y;
            nodeLocation.z = j_z;

            notifyWirelessChannel();
			trace() << "Changed location (x:y:z) to (" << nodeLocation.x << 
					":" << nodeLocation.y << ":" << nodeLocation.z<<")";

            DiscreteMobilityManagerMessageAck *dm_ack_msg=new DiscreteMobilityManagerMessageAck();
            dm_ack_msg->setKind(MobilityManagerMessageType::DISCRETE_MOBILITY_ACK);
            send(dm_ack_msg,"toApplicationModule");

            break;
        }
		default: {
			trace() << "WARNING: Unexpected message " << msgKind;
            break;
		}
	}

	delete msg;
	msg = NULL;
}

