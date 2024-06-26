/*******************************************************************************
 *  Copyright: National ICT Australia,  2007 - 2012                            *
 *  Developed at the ATP lab, Networked Systems research theme                 *
 *  Author(s): Athanassios Boulis, Dimosthenis Pediaditakis, Yuriy Tselishchev *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *  If you do not find this file, copies can be found by writing to:           *
 *                                                                             *
 *      NICTA, Locked Bag 9013, Alexandria, NSW 1435, Australia                *
 *      Attention:  License Inquiry.                                           *
 *                                                                             *  
 *******************************************************************************/

#include "node/resourceManager/ResourceManager.h"

Define_Module(ResourceManager);

void ResourceManager::initialize()
{
	sigmaCPUClockDrift = par("sigmaCPUClockDrift");
	//using the "0" rng generator of the ResourceManager module
	cpuClockDrift = normal(0, sigmaCPUClockDrift);
	/* Crop any values beyond +/- 3 sigmas. Some protocols (e.g., MAC) rely on
	 * bounded cpuClockDrift. Although the bounds are conservative (usually 3sigmas),
	 * if you instantiate thousands of nodes (in multiple runs) we will get a
	 * couple of nodes that will be beyond this bound. Limiting/Croping the drift
	 * is actually realistic, since usually there is some kind of quality
	 * control on quartz crystals or the boards that use them (sensor node)
	 */
	if (cpuClockDrift > 3 * sigmaCPUClockDrift)
		cpuClockDrift = 3 * sigmaCPUClockDrift;
	if (cpuClockDrift < -3 * sigmaCPUClockDrift)
		cpuClockDrift = -3 * sigmaCPUClockDrift;

	initialEnergy = par("initialEnergy");
	ramSize = par("ramSize");
	baselineNodePower = par("baselineNodePower");
	periodicEnergyCalculationInterval = (double)par("periodicEnergyCalculationInterval") / 1000;
    selfDestructTimer = par("selfDestructTimer");
    nullEnergyOnFail = par("nullEnergyOnFail");
    maximumCapacity = par("maximumCapacity");

    if(selfDestructTimer > 0.0) {
        scheduleAt(simTime() + selfDestructTimer, new cMessage("Destroy node message", DESTROY_NODE));
    }

	if (baselineNodePower < 0 || periodicEnergyCalculationInterval < 0)
		throw cRuntimeError("Illegal values for baselineNodePower and/or periodicEnergyCalculationInterval in resource manager module");

	currentNodePower = baselineNodePower;
	remainingEnergy = initialEnergy;
	totalRamData = 0;
	disabled = true;
     
    cModule *appModule = getParentModule()->getSubmodule("Application");
    master=par("isMaster");
    if(appModule->hasPar("isMaster") && master==false ) {
        master=appModule->par("isMaster");
    }
    trace()<<"Master node: "<<master;

}

void ResourceManager::calculateEnergySpent()
{
	if (remainingEnergy > 0) {
		simtime_t timePassed = simTime() - timeOfLastCalculation;
		trace() << "energy consumed in the last " << timePassed << 
			"s is " <<(timePassed * currentNodePower)<<" leftover energy is "<< remainingEnergy;
		consumeEnergy(SIMTIME_DBL(timePassed * currentNodePower / 1000.0));
		timeOfLastCalculation = simTime();

		cancelEvent(energyMsg);
		scheduleAt(simTime() + periodicEnergyCalculationInterval, energyMsg);
	}
}

/* The ResourceManager module has only one "unconnected" port where it can receive messages that
 * update the power drawn by a module, or a NODE_STARTUP message. If disabled we still process
 * messages because we want to have the latest power drawn from any module.
 */
void ResourceManager::handleMessage(cMessage * msg)
{

	switch (msg->getKind()) {

		case NODE_STARTUP:{
			disabled = false;
			timeOfLastCalculation = simTime();
			energyMsg = new cMessage("Periodic energy calculation", TIMER_SERVICE);
        		scheduleAt(simTime() + periodicEnergyCalculationInterval, energyMsg);
			break;
		}
	
		case TIMER_SERVICE:{
            if(!disabled) {
    			calculateEnergySpent();
            }
			return;
		}

		case RESOURCE_MANAGER_DRAW_POWER:{
			ResourceManagerMessage *resMsg = check_and_cast<ResourceManagerMessage*>(msg);
			int id = resMsg->getSenderModuleId();
			double oldPower = storedPowerConsumptions[id];
			trace() << "New power consumption, id = " << id << ", oldPower = " << 
					currentNodePower << ", newPower = " << 
					currentNodePower - oldPower + resMsg->getPowerConsumed();
			if (!disabled)
				calculateEnergySpent();
			currentNodePower = currentNodePower - oldPower + resMsg->getPowerConsumed();
			storedPowerConsumptions[id] = resMsg->getPowerConsumed();
			break;
		}

        case TERMINAL_EVENT: {
            trace()<<"Terminal event message received, Node terminating.";
            destroyNode();
            break;
        }

        case DESTROY_NODE: {
            trace()<<"selfDestructTimer expired";
            destroyNode();
            break;
        }

		default:{
			throw cRuntimeError("ERROR: Unexpected message received by resource manager: %d", msg->getKind());
		}
	}
	delete msg;
}

void ResourceManager::finishSpecific()
{
	calculateEnergySpent();
    if(!master) {
    	declareOutput("Consumed Energy");
	    collectOutput("Consumed Energy", "", initialEnergy - remainingEnergy);
    	declareOutput("Remaining Energy");
    	collectOutput("Remaining Energy", "", remainingEnergy);
    }

	if (getParentModule()->getIndex() == 0) {
		cTopology *topo;	// temp variable to access energy spent by other nodes
		topo = new cTopology("topo");
		topo->extractByNedTypeName(cStringTokenizer("node.Node").asVector());

		double minLifetime = estimateLifetime();
		for (int i = 1; i < topo->getNumNodes(); i++) {
			ResourceManager *resMng = dynamic_cast<ResourceManager*>
				(topo->getNode(i)->getModule()->getSubmodule("ResourceManager"));

			if (!resMng->isMaster() && minLifetime > resMng->estimateLifetime()) 
				minLifetime = resMng->estimateLifetime();
		}
		declareOutput("Estimated network lifetime (days)");
		collectOutput("Estimated network lifetime (days)", "", minLifetime);
		delete(topo);
	}
}

double ResourceManager::estimateLifetime(void) 
{
	return ((initialEnergy * simTime().dbl()) / ((initialEnergy - remainingEnergy) * 86400.0));
}

double ResourceManager::getSpentEnergy(void)
{
	Enter_Method("getSpentEnergy()");
	return (initialEnergy - remainingEnergy);
}

double ResourceManager::getRemainingEnergy(void)
{
    Enter_Method("getRemainingEnergy()");
    return remainingEnergy;
}

double ResourceManager::getInitialEnergy(void) {
    Enter_Method("getInitialEnergy()");
    return initialEnergy;
}

double ResourceManager::getMaximumCapacity(void) {
    Enter_Method("getMaximumCapacity()");
    return maximumCapacity;
}

double ResourceManager::getCPUClockDrift(void)
{
	Enter_Method("getCPUClockDrift(void)");
	return (1.0f + cpuClockDrift);
}

void ResourceManager::consumeEnergy(double amount)
{
	Enter_Method("consumeEnergy(double amount)");

    if(master) {
        return;
    }

	if (remainingEnergy <= amount) {
		remainingEnergy = 0;
		send(new cMessage("Destroy node message", OUT_OF_ENERGY), "toSensorDevManager");
		send(new cMessage("Destroy node message", OUT_OF_ENERGY), "toApplication");
		send(new cMessage("Destroy node message", OUT_OF_ENERGY), "toNetwork");
		send(new cMessage("Destroy node message", OUT_OF_ENERGY), "toMac");
		send(new cMessage("Destroy node message", OUT_OF_ENERGY), "toRadio");
        disabled = true;
        declareOutput("Dead Node");
		collectOutput("Dead Node", "yes?", 1);
		collectOutput("Dead Node", "time", SIMTIME_DBL(simTime()));
	} else
		remainingEnergy -= amount;
}

void ResourceManager::destroyNode(void)
{
	Enter_Method("destroyNode(void)");

	send(new cMessage("Destroy node message", DESTROY_NODE), "toSensorDevManager");
	send(new cMessage("Destroy node message", DESTROY_NODE), "toApplication");
	send(new cMessage("Destroy node message", DESTROY_NODE), "toNetwork");
	send(new cMessage("Destroy node message", DESTROY_NODE), "toMac");
	send(new cMessage("Destroy node message", DESTROY_NODE), "toRadio");
	disabled = true;

    declareOutput("Dead Node");
	collectOutput("Dead Node", "yes?", 1);
	collectOutput("Dead Node", "time", SIMTIME_DBL(simTime()));

    if(nullEnergyOnFail) {
        remainingEnergy = 0;
    }
}

bool ResourceManager::isDead() {
    return disabled;
}


int ResourceManager::RamStore(int numBytes)
{
	Enter_Method("RamStore(int numBytes)");

	int ramHasSpace = ((totalRamData + numBytes) <= ramSize) ? 1 : 0;
	if (!ramHasSpace) {
		trace() << "\n[Resource Manager] t= " << simTime() <<
				": WARNING: Data not stored to Ram. Not enough space to store them.";
		return 0;
	} else
		totalRamData += numBytes;
	return 1;
}

void ResourceManager::RamFree(int numBytes)
{
	Enter_Method("RamFree(int numBytes)");
	totalRamData -= numBytes;
	totalRamData = (totalRamData < 0) ? 0 : totalRamData;
}

bool ResourceManager::isMaster() { return master; }
