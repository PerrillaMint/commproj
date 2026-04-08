#ifndef OPERATORCONSOLE_H_
#define OPERATORCONSOLE_H_

#include <iostream>
#include <fstream>
#include <sys/dispatch.h>
#include <thread>
#include "Msg_structs.h"

class OperatorConsole {
public:
	OperatorConsole();
    ~OperatorConsole();

private:
    void HandleConsoleInputs();
    void logCommand(const std::string& command);
    std::thread Operator_Console;
    bool exit = false;
    std::ofstream commandLogFile;  // Spec: store operator requests in a log file
};



#endif /* OPERATORCONSOLE_H_ */
