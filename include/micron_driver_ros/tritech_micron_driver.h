// ######################################################################
//
//      TritechMicronDriver - A protocol parser for Tritech Micron sonars.
//      Copyright (C) 2011  Randolph Voorhies
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// ######################################################################

#ifndef TRITECHMICRON_TRITECHMICRON_H
#define TRITECHMICRON_TRITECHMICRON_H

#include "constants.h"
#include "message_types.h"
#include <vector>
#include "Serial.h"
#include <boost/thread.hpp>
#include "stdint.h"

class TritechMicronDriver
{
  public:

	uint16_t nBins;
    float range;
    float VOS;
    uint8_t angleStepSize;
    int leftLimit; 
	int rightLimit;
	int contFlag;

    TritechMicronDriver(uint16_t _nBins, float _range, float _VOS, uint8_t _angleStepSize, int _leftLimit, int _rightLimit, int _contFlag = 0x83, bool debugMode = true);

    ~TritechMicronDriver();

    //! Connect to a Sonar device, and configure it
    /*! @param nBins The desired number of bins in each scanline
        @param range The desired range (in meters)
        @param VOS The velocity of sound in the current medium
        @param stepAngleSize The angular resolution of each scan */
    bool connect(std::string const& devName);

    //! Disconnect from the Sonar device and kill all of our associated threads
    void disconnect();

    //! Configure the sonar
    void configure();

    void resetMessage();
	
	//This function allows the user to reconfigure the sonar once it is running without restarting the driver
	void reconfigure(uint16_t _nBins, float _range, float _VOS, uint8_t _angleStepSize, int _leftLimit, int _rightLimit, int _contFlag);

    //! Get access to the scan lines that have been read by the sonar.
    /*! Note that this method is thread safe and will lock the internal scan
        line data structure, therefore one should call this method as
        infrequently as possible */
    std::map<float, std::vector<uint8_t>> scanLines();



    void registerScanLineCallback(
        std::function<void(float/*angle*/, float/*meters per bin*/, std::vector<uint8_t>/*scanline*/)>);

  private:

	//! This function sets the necessary parameters for the Sonar operation.
	void setParameters(uint16_t _nBins, float _range, float _VOS, uint8_t _angleStepSize, int _leftLimit, int _rightLimit, int _contFlag);

    //! Process a single incoming byte (add it onto itsRawMsg, etc.)
    void processByte(uint8_t byte);

    //! Process an incoming message
    void processMessage(tritech::Message msg);

    //! The method being run by itsSerialThread
    void serialThreadMethod();

	//! The method being run by itsProcessingThread
	void processingThreadMethod();
    
    //! A thread that just spins and reads data from the serial port
    boost::thread itsSerialThread;

	//! A thread that just spins and interacts with the Sonar
    boost::thread itsProcessingThread;

    //! Current state of the incoming protocol FSM
    enum StateType 
    {
      WaitingForAt  = 0, //!< Waiting for an @ to appear
      ReadingHeader = 1, //!< The @ sign has been found, now we're reading the header data     
      ReadingData   = 2, //!< The header has been read, now we're just reading the data
    };

	//! The current state of the FSM controlling the parsing of the incoming protocol
    StateType itsState;

	//Semaphore type
	enum Semaphore 
	{
	red, 
	green, 
	};

	//State Machine Semaphore
	Semaphore stateMachineSemaphore; 

	//Scanning callback routine semaphore
	Semaphore scanningCallbackSemaphore;  

    //! States for the protocol state machine
	enum StateMachine_states 
	{
	  waitingforMtAlive_1,
	  waitingforMtAlive_2, 
	  versionData, 
	  configuring,
	  scanning,
	  reset, 
	};

	// The variable controlling the state machine 
	StateMachine_states state; 
	  
    
    //! The current message buffer begin read in
    std::vector<uint8_t> itsRawMsg; 

    //! The current incoming message begin constructed from itsRawMsg
    tritech::Message itsMsg;

    //! Have we ever heard an mtAlive message from the sonar? 
    bool hasHeardMtAlive;

    //! Have we ever heard an mtVersionData from the sonar?
    bool hasHeardMtVersionData;

	// Have we received a mtHeadData from the sonar? 
	bool hasHeardMtHeadData;

	//Has the Sonar received it's parameters?
	bool hasParams; 

    //! The actual serial port
    SerialPort itsSerial;

    //! Are we currently running?
    bool itsRunning;

    //! Should we be printing debugging information to the console? Warning, this is very noisy and slow.
    bool itsDebugMode;

    std::function<void(float/*angle*/, float/*meters per bin*/, std::vector<uint8_t>/*scanline*/)> itsScanLineCallback;
};

#endif // TRITECHMICRON_TRITECHMICRON_H

