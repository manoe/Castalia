//****************************************************************************
//*  Copyright: National ICT Australia,  2007 - 2010                         *
//*  Developed at the ATP lab, Networked Systems research theme              *
//*  Author(s): Yuriy Tselishchev                                            *
//*  This file is distributed under the terms in the attached LICENSE file.  *
//*  If you do not find this file, copies can be found by writing to:        *
//*                                                                          *
//*      NICTA, Locked Bag 9013, Alexandria, NSW 1435, Australia             *
//*      Attention:  License Inquiry.                                        *
//*                                                                          *  
//****************************************************************************/

cplusplus {{
#include "node/communication/routing/RoutingPacket_m.h"
}}

class RoutingPacket;

enum hdmrpPacketDef {
    DATA_PACKET         = 1;
    RREQ_PACKET         = 2;
    SINK_RREQ_PACKET    = 4;
}

enum hdmrpRoleDef {
    SINK        = 1;
    ROOT        = 2;
    SUB_ROOT    = 3;
    NON_ROOT    = 4;
}

enum hdmrpStateDef {
    WORK    = 1;
    INIT    = 2;
    LEARN   = 3;
    RELAY   = 4;
}

enum hdmrpTimerDef {
    SINK_START      = 1;
    T_L             = 2;
    ROOT_GUARD      = 3;
    SUB_ROOT_GUARD  = 4;
    NEW_ROUND       = 5;
    T_RELAY         = 6;
}

// RREQ = { R, S, P, len, nmas }
packet hdmrpPacket extends RoutingPacket {
    int hdmrpPacketKind enum (hdmrpPacketDef); // 1 byte
    int round;      // 2 byte
    int path_id;    // 2 byte
    int len;        // 1 byte?
    int nmas;       // 1 byte?
}

