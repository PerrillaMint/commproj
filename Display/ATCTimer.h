#ifndef ATCTIMER_H_
#define ATCTIMER_H_

#include <stdio.h>
#include <iostream>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include <sync.h>
#include <sys/siginfo.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <sys/syspage.h>
#include <inttypes.h>
#include <stdint.h>

class ATCTimer {
	int channel_id;  		// The ID of the message channel
	int connection_id;		// The ID for the connection to the timer

	// Structure for timer signal event
	struct sigevent sig_event;

	// Structure for timer specification (time intervals)
	struct itimerspec timer_spec;

	// Timer identifier
	timer_t timer_id;

	char msg_buffer[100];			// Buffer for receiving messages

	// Clock-related variables
	uint64_t cycles_per_sec; 			// Cycles per second, for time calculation
	uint64_t tick_cycles, tock_cycles;	// Variables to store cycle counts for time measurement
public:
	// Constructor to initialize timer with seconds and milliseconds
	ATCTimer(uint32_t,uint32_t);


	// Function to set the timer specifications (time intervals)
	void setTimerSpecification(uint32_t,uint32_t);

	// Function to block until the timer expires
	void waitTimer();

	// Function to start the timer
	void startTimer();

	// Function to record the current time for tick
	void tick();
	// Function to calculate the elapsed time since the last tick
	double tock();

	virtual ~ATCTimer(); //destructor
};

#endif /* ATCTIMER_H_ */
