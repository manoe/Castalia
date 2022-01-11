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
     bool isSink;
     int round;
     int t_l;
     hdmrpStateDef state;
     hdmrpRoleDef role;

 protected:
     void startup();
     void fromApplicationLayer(cPacket *, const char *);
     void fromMacLayer(cPacket *, int, double, double);
     void sendRREQ();
     void storeRREQ(hdmrpPacket *);
};

#endif //HDMRP
