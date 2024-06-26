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

#ifndef _BYPASSROUTING_H_
#define _BYPASSROUTING_H_

#include "node/communication/routing/VirtualRouting.h"
#include "node/communication/routing/flooding/floodingPacket_m.h"

using namespace std;

class flooding: public VirtualRouting {
    private:
        int repeat;
 protected:
    virtual void startup();
	virtual void fromApplicationLayer(cPacket *, const char *);
	virtual void fromMacLayer(cPacket *, int, double, double);
};

#endif				//BYPASSROUTINGMODULE
