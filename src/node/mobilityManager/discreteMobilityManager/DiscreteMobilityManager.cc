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
}

void DiscreteMobilityManager::handleMessage(cMessage * msg)
{
	int msgKind = msg->getKind();
	switch (msgKind) {
        case MobilityManagerMessageType::DISCRETE_MOBILITY: {
            DiscreteMobilityManagerMessage *dis_msg=check_and_cast<DiscreteMobilityManagerMessage *>(msg);
            trace()<<"Current location (x:y:z) is: ("<<nodeLocation.x<<":"<<nodeLocation.y<<":"<<nodeLocation.z<<")";
            nodeLocation.x = dis_msg->getX();
            nodeLocation.y = dis_msg->getY();
            notifyWirelessChannel();
			trace() << "Changed location (x:y:z) to (" << nodeLocation.x << 
					":" << nodeLocation.y << ":" << nodeLocation.z<<")";
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

