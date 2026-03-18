#ifndef OPERATORCONSOLE_H_
#define OPERATORCONSOLE_H_

#include <iostream>
#include <sys/dispatch.h>
#include <thread>
#include "Msg_structs.h"

class OperatorConsole {
public:
    OperatorConsole();
    ~OperatorConsole();

private:
    void handleConsoleInputs();
    void logCommand(const std::string& command);
    std::thread consoleThread;
    bool exitRequested = false;
};

#endif /* OPERATORCONSOLE_H_ */
