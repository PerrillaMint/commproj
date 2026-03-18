#ifndef SRC_COMMUNICATIONSSYSTEM_H_
#define SRC_COMMUNICATIONSSYSTEM_H_

#include <iostream>
#include <thread>
#include <sys/dispatch.h>
#include "Msg_structs.h"

class CommunicationsSystem {
public:
    CommunicationsSystem();
    ~CommunicationsSystem();

private:
    void handleCommunications();
    void messageAircraft(const Message_inter_process& msg);
    std::thread communicationsThread;
};

#endif /* SRC_COMMUNICATIONSSYSTEM_H_ */
