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
    void HandleCommunications();
    void messageAircraft(const Message& msg);
    std::thread Communications_System;
};


#endif /* SRC_COMMUNICATIONSSYSTEM_H_ */
