/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <ros/ros.h>
#include <RelaisBoardNode.h>



int RelaisBoardNode::init() 
{
	if (n.hasParam("ComPort"))
	{
		n.getParam("ComPort", sComPort);
		ROS_INFO("Loaded ComPort parameter from parameter server: %s",sComPort.c_str());
	}

	n.param("message_timeout", relayboard_timeout_, 2.0);
	n.param("requestRate", requestRate, 25.0);

	n.param("protocol_version", protocol_version_, 1);
    

        n.param("voltage_min", voltage_min_, 23.0);
	n.param("voltage_max", voltage_max_, 27.5);
	n.param("charge_nominal", charge_nominal_, 80.0);
	charge_nominal_ = charge_nominal_ * 360; //converts [Ah] to [As]
	n.param("voltage_nominal", voltage_nominal_, 24.0);

	current_voltage = 0;

	m_SerRelayBoard = new SerRelayBoard();
	ROS_INFO("Opened Relayboard at ComPort = %s", sComPort.c_str());

	n.getParam("drive1/CANId", motorCanIdent[0]);
	n.getParam("drive2/CANId", motorCanIdent[1]);
	n.getParam("drive3/CANId", motorCanIdent[2]);
	n.getParam("drive4/CANId", motorCanIdent[3]);
	n.getParam("drive1/joint_name", joint_names[0]);
	n.getParam("drive2/joint_name", joint_names[1]);
	n.getParam("drive3/joint_name", joint_names[2]);
	n.getParam("drive4/joint_name", joint_names[3]);
	//topics, which get published if the module is available
	n.param("hasMotorRight", activeModule[DRIVE1], 0);
	n.param("hasMotorLeft", activeModule[DRIVE2], 0);
	n.param("hasMotorRearRight", activeModule[DRIVE3], 0);
	n.param("hasMotorRearLeft", activeModule[DRIVE4], 0);
	if(activeModule[DRIVE1] == 1 || activeModule[DRIVE2] == 1 || activeModule[DRIVE3] == 1 || activeModule[DRIVE4] == 1)
	{
		topicPub_drives = n.advertise<cob_relayboard::DriveStates>("/drive_states",1);
		topicSub_drives = n.subscribe("/cmd_drives",1,&RelaisBoardNode::getNewDriveStates, this);
	}
	n.param("hasIOBoard", activeModule[IO_BOARD],0);
	n.param("hasLCDOut", hasLCDOut,0);
	if(hasLCDOut == 1) topicSub_lcdDisplay = n.subscribe("/srb_lcd_display",1,&RelaisBoardNode::getNewLCDOutput, this);

	if(activeModule[IO_BOARD] == 1)
	{
		topicSub_setDigOut = n.subscribe("/srb_io_set_dig_out",1,&RelaisBoardNode::getIOBoardDigOut, this);
		topicPub_ioDigIn = n.advertise<std_msgs::Int16>("/srb_io_dig_in",1);
		topicPub_ioDigOut = n.advertise<std_msgs::Int16>("/srb_io_dig_out",1);
		topicPub_analogIn = n.advertise<cob_relayboard::IOAnalogIn>("/srb_io_analog_in",1);

	}
	n.param("hasUSBoard", activeModule[US_BOARD], 0);
	if(activeModule[US_BOARD] == 1)
	{
		topicPub_usBoard = n.advertise<cob_relayboard::USBoard>("/srb_us_measurements",1);
		topicSub_startUSBoard = n.subscribe("/srb_start_us_board",1,&RelaisBoardNode::startUSBoard, this);
		topicSub_stopUSBoard = n.subscribe("/srb_stop_us_board",1,&RelaisBoardNode::stopUSBoard, this);

	}
	n.param("hasRadarBoard", activeModule[RADAR_BOARD], 0);
	if(activeModule[RADAR_BOARD] == 1) topicPub_radarBoard = n.advertise<cob_relayboard::RadarBoard>("/srb_radar_measurements",1);

	n.param("hasGyroBoard", activeModule[GYRO_BOARD], 0);
	if(activeModule[GYRO_BOARD] == 1)
	{
		topicPub_gyroBoard = n.advertise<cob_relayboard::GyroBoard>("/srb_gyro_measurements",1);
		topicSub_zeroGyro = n.subscribe("/srb_zero_gyro",1,&RelaisBoardNode::zeroGyro, this);
	}

	n.param("hasKeyPad", hasKeyPad, 0);
	if(hasKeyPad == 1) topicPub_keypad = n.advertise<cob_relayboard::Keypad>("/srb_keypad",1);
	n.param("hasIRSensors", hasIRSensors, 0);
	if(hasIRSensors == 1) topicPub_IRSensor = n.advertise<cob_relayboard::IRSensors>("/srb_ir_measurements",1);

	std::cout 	<< "RelaisBoardNode: " << "ComPort: " << sComPort << " message_timeout: " << relayboard_timeout_ << " requestRate: " << requestRate << " protocol_version: " << protocol_version_
			<< " voltage_min: " << voltage_min_ << " voltage_max: " << voltage_max_ << " charge_nominal: " << charge_nominal_ << " voltage_nominal: " << voltage_nominal_
			<< " drive1/CANId: " << motorCanIdent[0] << " drive2/CANId: " << motorCanIdent[1] << " drive3/CANId: " << motorCanIdent[2] << " drive4/CANId: " << motorCanIdent[3]
			<< " drive1/joint_name: " << joint_names[0] << " drive2/joint_name: " << joint_names[1] << " drive3/joint_name: " << joint_names[2] << " drive4/joint_name: " << joint_names[3]
			<< " hasMotorRight: " << activeModule[DRIVE1] << " hasMotorLeft: " << activeModule[DRIVE2] << " hasMotorRearRight: " << activeModule[DRIVE3] << " hasMotorRearLeft: " << activeModule[DRIVE4]
			<< " hasIOBoard: " << activeModule[IO_BOARD] << " hasLCDOut: " << hasLCDOut << " hasUSBoard: " << activeModule[US_BOARD] << " hasRadarBoard: " << activeModule[RADAR_BOARD]
			<< " hasGyroBoard: " << activeModule[GYRO_BOARD] << " hasKeyPad: " << hasKeyPad << " hasIRSensors: " << hasIRSensors << "\n";


	DriveParam driveParamLeft, driveParamRight, driveParamRearLeft, driveParamRearRight;

	int iEncIncrPerRevMot;
	double dVelMeasFrqHz;
	double dGearRatio, dBeltRatio;
	int iSign;
	bool bHoming;
	double dHomePos, dHomeVel;
	int iHomeEvent, iHomeDigIn, iHomeTimeOut;
	double dVelMaxEncIncrS, dVelPModeEncIncrS;
	double dAccIncrS2, dDecIncrS2;
	int iCANId;
	// drive parameters
	n.getParam("drive1/EncIncrPerRevMot", iEncIncrPerRevMot);
	n.getParam("drive1/VelMeasFrqHz", dVelMeasFrqHz);
	n.getParam("drive1/BeltRatio", dBeltRatio);
	n.getParam("drive1/GearRatio", dGearRatio);
	n.getParam("drive1/Sign", iSign);
	n.getParam("drive1/Homing", bHoming);
	n.getParam("drive1/HomePos", dHomePos);
	n.getParam("drive1/HomeVel", dHomeVel);
	n.getParam("drive1/HomeEvent", iHomeEvent);
	n.getParam("drive1/HomeDigIn", iHomeDigIn);
	n.getParam("drive1/HomeTimeOut", iHomeTimeOut);
	n.getParam("drive1/VelMaxEncIncrS", dVelMaxEncIncrS);
	n.getParam("drive1/VelPModeEncIncrS", dVelPModeEncIncrS);
	n.getParam("drive1/AccIncrS", dAccIncrS2);
	n.getParam("drive1/DecIncrS", dDecIncrS2);
	n.getParam("drive1/CANId", iCANId);
	driveParamLeft.set(	0,
							iEncIncrPerRevMot,
							dVelMeasFrqHz,
							dBeltRatio, dGearRatio,
							iSign,
							bHoming, dHomePos, dHomeVel, iHomeEvent, iHomeDigIn, iHomeTimeOut,
							dVelMaxEncIncrS, dVelPModeEncIncrS,
							dAccIncrS2, dDecIncrS2,
							DriveParam::ENCODER_INCREMENTAL,
							iCANId,
							false, true );
						
	n.getParam("drive2/EncIncrPerRevMot", iEncIncrPerRevMot);
	n.getParam("drive2/VelMeasFrqHz", dVelMeasFrqHz);
	n.getParam("drive2/BeltRatio", dBeltRatio);
	n.getParam("drive2/GearRatio", dGearRatio);
	n.getParam("drive2/Sign", iSign);
	n.getParam("drive2/Homing", bHoming);
	n.getParam("drive2/HomePos", dHomePos);
	n.getParam("drive2/HomeVel", dHomeVel);
	n.getParam("drive2/HomeEvent", iHomeEvent);
	n.getParam("drive2/HomeDigIn", iHomeDigIn);
	n.getParam("drive2/HomeTimeOut", iHomeTimeOut);
	n.getParam("drive2/VelMaxEncIncrS", dVelMaxEncIncrS);
	n.getParam("drive2/VelPModeEncIncrS", dVelPModeEncIncrS);
	n.getParam("drive2/AccIncrS", dAccIncrS2);
	n.getParam("drive2/DecIncrS", dDecIncrS2);
	n.getParam("drive2/CANId", iCANId);
	driveParamRight.set(	1,
							iEncIncrPerRevMot,
							dVelMeasFrqHz,
							dBeltRatio, dGearRatio,
							iSign,
							bHoming, dHomePos, dHomeVel, iHomeEvent, iHomeDigIn, iHomeTimeOut,
							dVelMaxEncIncrS, dVelPModeEncIncrS,
							dAccIncrS2, dDecIncrS2,
							DriveParam::ENCODER_INCREMENTAL,
							iCANId,
							false, true );


	n.getParam("drive3/EncIncrPerRevMot", iEncIncrPerRevMot);
	n.getParam("drive3/VelMeasFrqHz", dVelMeasFrqHz);
	n.getParam("drive3/BeltRatio", dBeltRatio);
	n.getParam("drive3/GearRatio", dGearRatio);
	n.getParam("drive3/Sign", iSign);
	n.getParam("drive3/Homing", bHoming);
	n.getParam("drive3/HomePos", dHomePos);
	n.getParam("drive3/HomeVel", dHomeVel);
	n.getParam("drive3/HomeEvent", iHomeEvent);
	n.getParam("drive3/HomeDigIn", iHomeDigIn);
	n.getParam("drive3/HomeTimeOut", iHomeTimeOut);
	n.getParam("drive3/VelMaxEncIncrS", dVelMaxEncIncrS);
	n.getParam("drive3/VelPModeEncIncrS", dVelPModeEncIncrS);
	n.getParam("drive3/AccIncrS", dAccIncrS2);
	n.getParam("drive3/DecIncrS", dDecIncrS2);
	n.getParam("drive3/CANId", iCANId);
	driveParamRearLeft.set(	1,
							iEncIncrPerRevMot,
							dVelMeasFrqHz,
							dBeltRatio, dGearRatio,
							iSign,
							bHoming, dHomePos, dHomeVel, iHomeEvent, iHomeDigIn, iHomeTimeOut,
							dVelMaxEncIncrS, dVelPModeEncIncrS,
							dAccIncrS2, dDecIncrS2,
							DriveParam::ENCODER_INCREMENTAL,
							iCANId,
							false, true );


	n.getParam("drive4/EncIncrPerRevMot", iEncIncrPerRevMot);
	n.getParam("drive4/VelMeasFrqHz", dVelMeasFrqHz);
	n.getParam("drive4/BeltRatio", dBeltRatio);
	n.getParam("drive4/GearRatio", dGearRatio);
	n.getParam("drive4/Sign", iSign);
	n.getParam("drive4/Homing", bHoming);
	n.getParam("drive4/HomePos", dHomePos);
	n.getParam("drive4/HomeVel", dHomeVel);
	n.getParam("drive4/HomeEvent", iHomeEvent);
	n.getParam("drive4/HomeDigIn", iHomeDigIn);
	n.getParam("drive4/HomeTimeOut", iHomeTimeOut);
	n.getParam("drive4/VelMaxEncIncrS", dVelMaxEncIncrS);
	n.getParam("drive4/VelPModeEncIncrS", dVelPModeEncIncrS);
	n.getParam("drive4/AccIncrS", dAccIncrS2);
	n.getParam("drive4/DecIncrS", dDecIncrS2);
	n.getParam("drive4/CANId", iCANId);
	driveParamRearRight.set(	1,
							iEncIncrPerRevMot,
							dVelMeasFrqHz,
							dBeltRatio, dGearRatio,
							iSign,
							bHoming, dHomePos, dHomeVel, iHomeEvent, iHomeDigIn, iHomeTimeOut,
							dVelMaxEncIncrS, dVelPModeEncIncrS,
							dAccIncrS2, dDecIncrS2,
							DriveParam::ENCODER_INCREMENTAL,
							iCANId,
							false, true );



	m_SerRelayBoard->setConfig(	protocol_version_, sComPort, activeModule[DRIVE1], 
					activeModule[DRIVE2], activeModule[DRIVE3], activeModule[DRIVE4],
					activeModule[IO_BOARD], activeModule[US_BOARD], activeModule[RADAR_BOARD], activeModule[GYRO_BOARD], 
					driveParamLeft, driveParamRight,
					driveParamRearLeft, driveParamRearRight
			);
	
	m_SerRelayBoard->init();

	// Init member variable for EM State
	EM_stop_status_ = ST_EM_ACTIVE;
	duration_for_EM_free_ = ros::Duration(1);
	return 0;
}


int RelaisBoardNode::requestBoardStatus() {
	int ret;	
	
	// Request Status of RelayBoard 
	ret = m_SerRelayBoard->sendRequest();

	if(ret != SerRelayBoard::NO_ERROR) {
		ROS_ERROR("Error in sending message to Relayboard over SerialIO, lost bytes during writing");
	}

	ret = m_SerRelayBoard->evalRxBuffer();
	if(ret==SerRelayBoard::NOT_INITIALIZED) {
		ROS_ERROR("Failed to read relayboard data over Serial, the device is not initialized");
		relayboard_online = false;
	} else if(ret==SerRelayBoard::NO_MESSAGES) {
		ROS_ERROR("For a long time, no messages from RelayBoard have been received, check com port!");
		if(time_last_message_received_.toSec() - ros::Time::now().toSec() > relayboard_timeout_) {relayboard_online = false;}
	} else if(ret==SerRelayBoard::TOO_LESS_BYTES_IN_QUEUE) {
		//ROS_ERROR("Relayboard: Too less bytes in queue");
	} else if(ret==SerRelayBoard::CHECKSUM_ERROR) {
		ROS_ERROR("A checksum error occurred while reading from relayboard data");
	} else if(ret==SerRelayBoard::NO_ERROR) {
		relayboard_online = true;
		relayboard_available = true;
		time_last_message_received_ = ros::Time::now();
	}

	return 0;
}

double RelaisBoardNode::getRequestRate()
{
	return requestRate;
}

//////////////
// RelaisBoard

void RelaisBoardNode::sendEmergencyStopStates()
{

	if(!relayboard_available) return;
	
	
	bool EM_signal;
	ros::Duration duration_since_EM_confirmed;
	cob_relayboard::EmergencyStopState EM_msg;

	// assign input (laser, button) specific EM state TODO: Laser and Scanner stop can't be read independently (e.g. if button is stop --> no informtion about scanner, if scanner ist stop --> no informtion about button stop)
	EM_msg.emergency_button_stop = m_SerRelayBoard->isEMStop();
	EM_msg.scanner_stop = m_SerRelayBoard->isScannerStop();

	// determine current EMStopState
	EM_signal = (EM_msg.emergency_button_stop || EM_msg.scanner_stop);

	switch (EM_stop_status_)
	{
		case ST_EM_FREE:
		{
			if (EM_signal == true)
			{
				ROS_ERROR("Emergency stop was issued");
				EM_stop_status_ = EM_msg.EMSTOP;
			}
			break;
		}
		case ST_EM_ACTIVE:
		{
			if (EM_signal == false)
			{
				ROS_INFO("Emergency stop was confirmed");
				EM_stop_status_ = EM_msg.EMCONFIRMED;
				time_of_EM_confirmed_ = ros::Time::now();
			}
			break;
		}
		case ST_EM_CONFIRMED:
		{
			if (EM_signal == true)
			{
				ROS_ERROR("Emergency stop was issued");
				EM_stop_status_ = EM_msg.EMSTOP;
			}
			else
			{
				duration_since_EM_confirmed = ros::Time::now() - time_of_EM_confirmed_;
				if( duration_since_EM_confirmed.toSec() > duration_for_EM_free_.toSec() )
				{
					ROS_INFO("Emergency stop released");
					EM_stop_status_ = EM_msg.EMFREE;
				}
			}
			break;
		}
	};

	
	EM_msg.emergency_state = EM_stop_status_;

	//publish EM-Stop-Active-messages, when connection to relayboard got cut
	if(relayboard_online == false) {
		EM_msg.emergency_state = EM_msg.EMSTOP;
	}
	topicPub_isEmergencyStop.publish(EM_msg);

        pr2_msgs::PowerBoardState pbs;
        pbs.header.stamp = ros::Time::now();
	// pr2 power_board_state
	if(EM_msg.emergency_button_stop)
	  pbs.run_stop = false;
	else
	  pbs.run_stop = true;
	
	//for cob the wireless stop field is misused as laser stop field
	if(EM_msg.scanner_stop)
	  pbs.wireless_stop = false; 
	else
	  pbs.wireless_stop = true;
	pbs.input_voltage = current_voltage;
	topicPub_boardState.publish(pbs);
}


void RelaisBoardNode::sendAnalogIn()
{
	if(!relayboard_available) return;
	int analogIn[8];
	m_SerRelayBoard->getRelayBoardAnalogIn(analogIn);
	//temperatur
	cob_relayboard::Temperatur temp;
	temp.temperatur = analogIn[2];
	topicPub_temperatur.publish(temp);
	//battery
	pr2_msgs::PowerState bat;
	current_voltage = analogIn[1]/1000;
	bat.header.stamp = ros::Time::now();
	double percentage = ((analogIn[1] /*measured volts*/ / 1000.0) - voltage_min_) * 100 / (voltage_max_ - voltage_min_);
	/*dt_remaining = (i*dt)_nominal * v_nominal * percentage_remaining / (v_meassured * i_meassured)*/
	bat.relative_capacity = percentage;
	/* charging rate: analogIn[0];*/
	topicPub_batVoltage.publish(bat);
	//keypad
	if(hasKeyPad == 1)
	{
		cob_relayboard::Keypad pad;
		int mask = 1;
		for(int i = 0; i<4; i++)
		{
			if((analogIn[3] & mask) != 0)
			{
				pad.button[i] = true;
			} else {
				pad.button[i] = false;
			} 
			mask = mask << 1;
		}
		topicPub_keypad.publish(pad);
	}
	if(hasIRSensors == 1)
	{
		cob_relayboard::IRSensors irmsg;
		for(int i=0; i<4; i++) irmsg.measurement[i] = analogIn[4+i];
		topicPub_IRSensor.publish(irmsg);
	}
}

//////////////
// motorCtrl

void RelaisBoardNode::sendDriveStates()
{
	if(!relayboard_available) return;
	cob_relayboard::DriveStates state;
	for(int i = 0; i<4; i++)  state.joint_names[i] = joint_names[i];
	int temp;

	if (activeModule[DRIVE1] == 1)
	{
		m_SerRelayBoard->getWheelPosVel(motorCanIdent[0],&(state.angularPosition[0]), &(state.angularVelocity[0]));
		m_SerRelayBoard->getStatus(motorCanIdent[0], &(state.motorState[0]), &temp);
	} 
	if (activeModule[DRIVE2] == 1)
	{
		m_SerRelayBoard->getWheelPosVel(motorCanIdent[1],&(state.angularPosition[1]), &(state.angularVelocity[1]));
		m_SerRelayBoard->getStatus(motorCanIdent[1], &(state.motorState[1]), &temp);

	}
	if (activeModule[DRIVE3] == 1)
	{
		m_SerRelayBoard->getWheelPosVel(motorCanIdent[2],&(state.angularPosition[2]), &(state.angularVelocity[2]));
		m_SerRelayBoard->getStatus(motorCanIdent[2], &(state.motorState[2]), &temp);
	} 
	if (activeModule[DRIVE4] == 1)
	{
		m_SerRelayBoard->getWheelPosVel(motorCanIdent[3],&(state.angularPosition[3]), &(state.angularVelocity[3]));
		m_SerRelayBoard->getStatus(motorCanIdent[3], &(state.motorState[3]), &temp);

	}
	if(activeModule[DRIVE1] == 1 || activeModule[DRIVE2] == 1 || activeModule[DRIVE3] == 1 || activeModule[DRIVE4] == 1)
	{	
		topicPub_drives.publish(state);
	} 
}

void RelaisBoardNode::getNewDriveStates(const cob_relayboard::DriveCommands& driveCommands)
{

	if(!relayboard_available) return;
	for(int i=0; i<4; i++)
	{	
		if(driveCommands.driveActive[i]){
			ROS_DEBUG("%i : received drive command: %f   %f",motorCanIdent[i],driveCommands.angularVelocity[i],driveCommands.angularVelocity[i]);
			m_SerRelayBoard->disableBrake(motorCanIdent[i],driveCommands.disableBrake[i]);
			m_SerRelayBoard->setWheelVel(motorCanIdent[i], driveCommands.angularVelocity[i], driveCommands.quickStop[i]);
		}
	}
}

//////////////
// GyroBoard

void RelaisBoardNode::sendGyroBoard()
{
	if(!relayboard_available || activeModule[GYRO_BOARD] != 1) return;
	cob_relayboard::GyroBoard gyro;
	m_SerRelayBoard->getGyroBoardAngBoost(&(gyro.orientation),gyro.acceleration);
	topicPub_gyroBoard.publish(gyro);
}

void RelaisBoardNode::zeroGyro(const std_msgs::Bool& b)
{
	if(!relayboard_available || activeModule[GYRO_BOARD] != 1) return;
	m_SerRelayBoard->zeroGyro(b.data);
}

//////////////
// radarBoard

void RelaisBoardNode::sendRadarBoard()
{
	if(!relayboard_available || activeModule[RADAR_BOARD] != 1) return;
	cob_relayboard::RadarBoard radar;
	double radarState[4];
	m_SerRelayBoard->getRadarBoardData(radarState);
	for(int i=0; i<3;i++)
	{
		radar.velocity[i] = radarState[i];
	}
	radar.state = radarState[3];
	topicPub_radarBoard.publish(radar);
}

//////////////
// usBoard

void RelaisBoardNode::sendUSBoard()
{
	if(!relayboard_available || activeModule[US_BOARD] != 1) return;
	int usSensors[8];
	int usAnalog[4];
	cob_relayboard::USBoard usBoard;
	m_SerRelayBoard->getUSBoardData1To8(usSensors);
	for(int i=0; i<8; i++) usBoard.sensor[i] = usSensors[i];
	m_SerRelayBoard->getUSBoardData9To16(usSensors);
	for(int i=0; i<8; i++) usBoard.sensor[i+8] = usSensors[i];
	m_SerRelayBoard->getUSBoardAnalogIn(usAnalog);
	for(int i=0; i<4; i++) usBoard.analog[i] = usAnalog[i];	
	topicPub_usBoard.publish(usBoard);
}

void RelaisBoardNode::startUSBoard(const std_msgs::Int16& configuration)
{
	if(!relayboard_available || activeModule[US_BOARD] != 1) return;
	m_SerRelayBoard->startUS(configuration.data);
}

void RelaisBoardNode::stopUSBoard(const std_msgs::Empty& empty)
{
	if(!relayboard_available || activeModule[US_BOARD] != 1) return;
	m_SerRelayBoard->stopUS();
}

//////////////
// ioBoard

void RelaisBoardNode::getNewLCDOutput(const cob_relayboard::LCDOutput& msg) 
{
	if(!relayboard_available || hasLCDOut != 1) return;
	m_SerRelayBoard->writeIOBoardLCD(0,0,msg.msg_line);

}

void RelaisBoardNode::getIOBoardDigOut(const cob_relayboard::IOOut& setOut)
{
	if(!relayboard_available || activeModule[IO_BOARD] != 1) return;
	m_SerRelayBoard->setIOBoardDigOut(setOut.channel, setOut.active);
}

void RelaisBoardNode::sendIOBoardDigIn()
{
	if(!relayboard_available || activeModule[IO_BOARD] != 1) return;
 	std_msgs::Int16 i;
	i.data = m_SerRelayBoard->getIOBoardDigIn();
	topicPub_ioDigIn.publish(i);

}

void RelaisBoardNode::sendIOBoardDigOut()
{
	if(!relayboard_available || activeModule[IO_BOARD] != 1) return;
 	std_msgs::Int16 i;
	i.data = m_SerRelayBoard->getIOBoardDigOut();
	topicPub_ioDigOut.publish(i);
}

void RelaisBoardNode::sendIOBoardAnalogIn()
{
	if(!relayboard_available || activeModule[IO_BOARD] != 1) return;
	int analogIn[8];
	cob_relayboard::IOAnalogIn in;
	m_SerRelayBoard->getIOBoardAnalogIn(analogIn);
	for(int i=0;i <8; i++) in.input[i] = analogIn[i];
	topicPub_analogIn.publish(in);
}
