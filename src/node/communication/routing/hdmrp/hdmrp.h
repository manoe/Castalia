/*******************************************************************************
 *  Copyright: National ICT Australia,  2007 - 2011                            *
 *  Developed at the ATP lab, Networked Systems research theme                 *
 *  Author(s): Athanassios Boulis, Yuriy Tselishchev                           *
 *  This file is distributed under the terms in the attached LICENSE file.     *
 *  If you do not find this file, copies can be found by writing to:           *
 *                                                                             *
 *      NICTA, Locked Bag 9013, Alexandria, NSW 1435, Australia                *
 *      Attention:  License Inquiry.                                           *
 *                                                                             *
 *******************************************************************************/

#ifndef _HDMRP_H_
#define _HDMRP_H_

#include "node/communication/routing/VirtualRouting.h"
#include "node/communication/routing/hdmrp/hdmrp_m.h"

class hdmrp: public VirtualRouting {
 private:
     int round;
     int t_l;
     hdmrpStateDef state;
     hdmrpRoleDef role;

 protected:
     void startup();
     void fromApplicationLayer(cPacket *, const char *);
     void fromMacLayer(cPacket *, int, double, double);
     void timerFiredCallback(int);

     void sendSRREQ();
     void sendRREQ();
     void sendRREQ(int, int);
     void sendRREQ(int, int, int, int); 
     void storeRREQ(hdmrpPacket *);

     bool isSink() const;
     bool isRoot() const;
     bool isSubRoot() const;
     bool isNonRoot() const;
     void setRole(hdmrpRoleDef);

     bool isWorkingState() const;
     bool isLearningState() const;
     void setState(hdmrpStateDef);

     void initRound();
     void newRound();
     void setRound(int);
     int  getRound() const;
     

};

#endif //HDMRP
