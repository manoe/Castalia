enum MsgType {
    EMERGENCY     = 1;
    STEADY        = 2;
    PREP_MOBILITY = 3;
    RELEARN       = 4;
}

message EmergencyMessage {
    int event enum(MsgType);
    double environment;
    double energy;
}
