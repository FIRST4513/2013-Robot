// **************************************************************************************
//       SimpleTemplate - MyRobot10.cpp		2/10/13		6:00 PM
//
//		version 11	12-3-13	Added code for Network tables to permit vision interface
//							with Roborealm vision software
//
//		version 10	2-19-13	Cleaned up some code for clarity (moved sidecar ports to
//							match the order assigned.
//
//		version 9	2-19-13	Added a second jaguar driver for the lift motor
//
//		version 8	2-19-13	Moved Cam buttons to left joystick from right. Also put in
//							test display of input ports. And set a max speed for the
//							hook Lift mechanism.
//
//		version 7 	2-15-13 Added routine to gather and display joystick and throttle
//							settings. This is to verify deadband zone and throttle
//							calculations.
//
//		version 6	2-10-13	Moved basic funcions from operator mode into sub-functions.
//                          Set up constants for joystick buttons. Set up sererate wait
//                          constants for seating limit switches for Hook, Arm and Cam. 
//                          Updated deadband calculations. Modified smartdash board
//                          routines to use variables in routines to set messages and
//                          only update the display at timed intervals at end of operator
//                          control loop (ie. every 0.1 second intervals).
//
//		version 5	2-9-13	Various updates to smartdash board and comments for clarity
//
//		version 4	2-9-13	Updated encoder methods
//
//		version 1	2-8-13	Base Code to manually control climbing arm. Modified speed
//                          control for Throttle. Utilized simplified SmartDashBoard settings
//							Utilized magnetic sensor for position info. on Arm and Lift.
// **************************************************************************************
#include "WPILib.h"
#include "NetworkTables/NetworkTable.h"
#include <iostream>
#include <sstream>

#include <string>

const string kProgramName = "Simple Template - MyRobot Version 10 (2/19/2013)";

const UINT32 kLmtSwTrue = 1;			// Limit sw is currently pressed
const UINT32 kLmtSwFalse = 0;			// Limit sw is currently NOT pressed

const float	 kDriveDeadband = 0.28;		// scale constant for Drive deadband zone
const float  kTurnRate = 1.2;			// scale the turn rate by this amount

const float	 kHookLiftDeadband = 0.125;	// deadband = 1/8 (0.125) of max speed
const float	 kHLMaxSpeed = 1.0;			// Hook Lift Speed Max .1 = 10% 1 = full
const double kInitHookLiftSpeed = 0.05;	// init speed is 1/2 of Max Speed of .1
const double kHookLiftWait = 0.0;		// Wait time to fully seat limit switches
const double kHookLiftDistancePerPulse = 0.878906;	// 360 degress / 4096 cnts

const float	 kArmDeadband = 0.1;		// scale constant for Arm deadband zone
const double kInitArmSpeed = 0.3;		// Speed to move arm during reset arm
const double kArmWait = 0.0;			// Wait time to fully seat limit switches
const double kArmDistancePerPulse = 0.878906;		// 360 degress / 4096 cnts

const double kCamWait = 0.0;			// Wait time to fully seat limit switches

const UINT32 kDriveBtn = 1;				// Button 1 (Trigger) R-Joy to drive
const UINT32 kHookLiftBtn = 2;			// Button 2 (Top) R-Joy to move Hook Lift
const UINT32 kArmBtn = 2;				// Button 2 (Top) L-Joy to move Hook Lift
const UINT32 kOpenCamBtn = 4;			// Button 4 L-Joy to open cam
const UINT32 kCloseCamBtn = 5;			// Button 5 L-Joy to close cam
const UINT32 kInitArmBtn = 6;			// Button 6 L-Joy &  R-Joy to reset arm
										// to init position

class RobotDemo : public SimpleRobot

{
	Joystick rstick;					// right joystick
	Joystick lstick;	 				// left joystick

	RobotDrive myRobot;					// robot drive system
	Jaguar hookLiftMotor;				// HookLift Motor
	Jaguar hookLiftMotor2;				// 2nd Hook lift motor
	Jaguar armMotor;					// Arm rotate motor
	Relay camSpike;						// Cam spike controller

	DigitalInput armLmtSwBack;			// Hook Limit Switch
	DigitalInput armLmtSwForward;		// Hook Limit Switch
	DigitalInput hookLiftLmtSwTop;		// Hook Limit Switch
	DigitalInput hookLiftLmtSwBottom;	// Hook Limit Switch
	DigitalInput camLmtSwOpen;			// Hook Limit Switch
	DigitalInput camLmtSwClosed;		// Hook Limit Switch

	DigitalInput in7;					// Test input port 7
	DigitalInput in8;					// Test input port 8
	DigitalInput in9;					// Test input port 9
	DigitalInput in10;					// Test input port 10

	DigitalInput hookLiftEncA;			// Hook Lift Encoder Input A
	DigitalInput hookLiftEncB; 			// Hook Lift Encoder Input A
	Encoder hookLiftEnc;				// Hook Lift Encoder Object
	DigitalInput armEncA;				// Arm Encoder Input A
	DigitalInput armEncB;				// Arm Encoder Input A
	Encoder armEnc;						// Arm Encoder Object
		

	DriverStationLCD *dsLCD;

	float aThrottle, aSpeed, aTurn;
	float rThrottle, rSpeed, rTurn;
	float hThrottle, hSpeed, hTurn;
	bool camOpenFlag, camCloseFlag;
	double timeLast;
	string functionName;	// Store last function name
	string rJoystickMsg;	// Store last right joystick press
	string lJoystickMsg;	// Store last left joystick press
	string msg1;			// Store a message
	string msg2;			// second message

public:
	
	NetworkTable *table;
	
	RobotDemo(void):
		rstick(1),
		lstick(2),

		myRobot(1, 2),			// PWM Output Port Assignments	
		hookLiftMotor(3),
		hookLiftMotor2(4),
		armMotor(5),
		
		camSpike(1),			// Relay Output Port Assignments
		
		armLmtSwBack(1),		// Digital I/O Port Assignments
		armLmtSwForward(2),
		hookLiftLmtSwTop(3),
		hookLiftLmtSwBottom(4),
		camLmtSwOpen(5),
		camLmtSwClosed(6),
		in7(7),
		in8(8),
		in9(9),
		in10(10),
		hookLiftEncA(11),		
		hookLiftEncB(12),
		hookLiftEnc(hookLiftEncA,hookLiftEncB,true),
		armEncA(13),
		armEncB(14),		
		armEnc(armEncA,armEncB,true)

	{
		table = NetworkTable::GetTable("datatable");
		
		myRobot.SetExpiration(1.0f); //TODO REMOVE
		myRobot.SetSafetyEnabled(false); //TODO REMOVE
		GetWatchdog().SetEnabled(false); //TODO REMOVE
		SmartDashboard::init();
		dsLCD = DriverStationLCD::GetInstance();
		
		hookLiftEnc.SetDistancePerPulse(kHookLiftDistancePerPulse);
		armEnc.SetDistancePerPulse(kArmDistancePerPulse);
		
		camOpenFlag = false;
		camCloseFlag = false;
		timeLast = 0;			// this should result in updating the display first pass through
		functionName = "Startup";
		msg1 = " ";
		msg2 = " ";
		lJoystickMsg = " ";
		rJoystickMsg = " ";
		
		updateSmartDash();
	}


	// ****************************************************************
	// *                     TELEOPERATOR MODE                        *
	// ****************************************************************
	void OperatorControl(void)
	{
		dsLCD->Printf(DriverStationLCD::Line(1),1,"Teleoperator Mode Running");
		dsLCD->UpdateLCD();		
		hookLiftEnc.Reset();					// Reset internal counter to zero
		hookLiftEnc.Start();
		armEnc.Reset();
		armEnc.Start();
//		double x = 0.8;

		while (IsOperatorControl())
		{
//			table->putNumber("X", x);
			updateDrive();
			updateHookLift();
			updateArm();
//			updateCam();
//			checkResetArm();
//			testThrottle();
			if (Timer::GetFPGATimestamp() - timeLast > .1)
			{	//Update display 10 times a second
				updateSmartDash();
				timeLast =Timer::GetFPGATimestamp();
			}
		} // ---- end of IsOperatorControl loop ----
	} // --------end of Operator mode ---------

	// ****************************************************************
	// *                          TEST MODE                           *
	// ****************************************************************

	void Test() {
		dsLCD->Printf(DriverStationLCD::Line(1),1,"Test Mode Running");
		dsLCD->UpdateLCD();		
		while (IsTest())
		{
			testThrottle();
			if (Timer::GetFPGATimestamp() - timeLast > .1)
			{	//Update display 10 times a second
				updateSmartDash();
				timeLast =Timer::GetFPGATimestamp();
			}			
		} // -------- end of test loop ------------
	} // ------------ end of test mode --------------------

	// ****************************************************************
	// *                   AUTONOMOUS MODE                            *
	// ****************************************************************
	void Autonomous(void)
	{
		dsLCD->Printf(DriverStationLCD::Line(1),1,"Autonomous Mode Running");
		dsLCD->UpdateLCD();		
		myRobot.SetSafetyEnabled(false);
		myRobot.Drive(-0.5, 0.0); 	// drive forwards half speed
		Wait(2.0); 					// for 2 seconds
		myRobot.Drive(0.0, 0.0); 	// stop robot
	}

	
	
	
	
	
	
	// *******************************************************************************
	// *                         Operator Control Routines                           *				  
	// *******************************************************************************


	// -------------------- DRIVE ROBOT (Right Trigger) -------------------------
	//    this routine checks the drive button and send data to update motors
	// ---------------------------------------------------------------------------

	void updateDrive()
	{
		if (rstick.GetRawButton(kDriveBtn)== true){ 
			functionName="Drive Robot";				
			rJoystickMsg="Trigger Pushed";

			// -------- scale speed ------
			rThrottle = ((rstick.GetThrottle() - 1)/-2);
			rSpeed = rstick.GetY() * rThrottle;
			rTurn = rstick.GetX() * kTurnRate * rThrottle;
			if (rTurn >1) rTurn = 1;
			else if (rTurn < -1) rTurn=-1;
			// calculate a deadband
			if (rSpeed > (- kDriveDeadband * rThrottle) and rSpeed < (kDriveDeadband * rThrottle))
				rSpeed = 0;
			if (rTurn > (-kDriveDeadband * rThrottle) and rTurn < (kDriveDeadband * rThrottle))
				rTurn = 0;
			Wait(0.005); // wait for a motor update time
		}
		else {
			rSpeed = 0.0;
			rTurn = 0.0;
		}
		myRobot.ArcadeDrive(rSpeed, rTurn); // send drive speed to drive class
	} //--------end updateDrive routine -------------


	// ---------------RAISE/LOWER Hook (R-Joystick Top Button) -------------------
	//          this routine lifts and lowers hook and verifies limits
	// ---------------------------------------------------------------------------
	
	void updateHookLift()
	{		
		if (rstick.GetRawButton(kHookLiftBtn)== true) {
			functionName="Lift/Lower Hook";				
			rJoystickMsg="Top Button Pushed";

			// -------- scale speed --------
			hThrottle = ((rstick.GetThrottle() -1)/-2);	// converts(+1 -> -1) to (0 -> 1)
			hSpeed = (rstick.GetY() * kHLMaxSpeed) * hThrottle; // maximum will never exceed max
			
			// calculate a dead band
//			if  ((hSpeed < (kHookLiftDeadband * hThrottle * kHLMaxSpeed)) and
//				(hSpeed > (-kHookLiftDeadband * hThrottle * kHLMaxSpeed)))
//				{
//					hSpeed = 0;
//				}
			
			// ------- check direction and limit switches and update motor speed ---------
			if (hSpeed == 0.0)	// stop the motor
				{
				hookLiftMotor.Set(0.0);
				hookLiftMotor2.Set(0.0);				
				}
			else if (hSpeed > 0){	// speed > 0 therefore RAISE hooklift
				if (hookLiftLmtSwTop.Get() == kLmtSwFalse){	// Are we at top already?
					hookLiftMotor.Set(hSpeed );				// No, so we can drive on up
					hookLiftMotor2.Set(hSpeed);
				}
				else{ 	// lift has hit the TOP limit switch so STOP the motor
					// Wait(kHookLiftWait);					// give time to fully seat at limit
					hookLiftMotor.Set(0.0);					// stop motor
					hookLiftMotor2.Set(0.0);					
					hookLiftEnc.Reset();					// Reset Encoder to zero
					hookLiftEnc.Start();
				}
			}
			else	// speed < 0 so LOWER the hooklift
			{
				if (hookLiftLmtSwBottom.Get() == kLmtSwFalse){	// Are we at the BOTTOM already?
					hookLiftMotor.Set(hSpeed);					// No, so we can drive on down
					hookLiftMotor2.Set(hSpeed);
				}
				else{  // lift has hit the BOTTOM limit switch so STOP the motor
					// Wait(kHookLiftWait);						// give time to fully seat at limit
					hookLiftMotor.Set(0.0);						// stop motor
					hookLiftMotor2.Set(0.0);						// stop motor
				}
			}
		}
		else // The Joystick button isn't pressed so stop the motor
			{
			hookLiftMotor.Set(0.0);
			hookLiftMotor2.Set(0.0);
			}
	}  //      -------- end of hooklift woutine -------------


	// --------- ARM FORWARD/BACKWORD Routine (L-Joystick Top Button) ----------------
	//     this routine rotates the Arm Forward/Backward and verifies limits
	// -------------------------------------------------------------------------------
	
	void updateArm()
	{		
		if (lstick.GetRawButton(kArmBtn)== true){
			functionName="Forward/Backword Arm";				
			lJoystickMsg="Top Button Pushed";

			// ------------------------- scale speed -------------------------------------
			aThrottle = ((lstick.GetThrottle() - 1)/-2); // converts(+1 -> -1) to (0 -> 1)	
			aSpeed = lstick.GetY()* - 1.0 * aThrottle;
			
			// produce dead band
//			if (aSpeed > (-kArmDeadband * aThrottle) and aSpeed < (kArmDeadband * aThrottle))
//				aSpeed = 0;

			// ----- check limit switches, speed direction and update motor speed --------
			if (aSpeed == 0.0)		// speed = 0 so stop the motor
				armMotor.Set(0.0);
			else if (aSpeed < 0){	// speed < 0 therefore move arm FORWARD
				if (armLmtSwBack.Get() == kLmtSwFalse){	// Are we fully forward ?
					armMotor.Set(aSpeed);					// No, so drive on forward		
				}
				else{ // arm has hit the FORWARD limit switch so STOP the motor
					Wait(kArmWait);							// give time to fully seat at limit
					armMotor.Set(0.0);						// stop motor
				}
			}
			else	// speed >0 so move arm BACKWARD
			{
				if (armLmtSwForward.Get() == kLmtSwFalse){		// Are we fully back?
					armMotor.Set(aSpeed);					// No, so we can move backwards		
				}
				else { // arm has hit the BACK limit switch so STOP
					Wait(kArmWait);							// give time to fully seat at limit
					armMotor.Set(0.0);						// stop motor
					armEnc.Reset();							// Reset Encoder to zero
					armEnc.Start();
				}
			}
		}
		else	// The joystick button is not pushed so stop motors
			armMotor.Set(0.0);
		} // -------- end of hooklift routine -------------

	
	// ---------------------------------------------------------------------------
	// --- Update Cam Position OPEN/CLOSE Cam (R-Joystick Button 4/5 top left) ---
	// ---------------------------------------------------------------------------

	void updateCam()
	{
		if (lstick.GetRawButton(kOpenCamBtn)== true or camOpenFlag == true)
			{
			// --------------- OPEN Cam (R-Joystick Button 5 top right) -------------------
			// -----------------------------------------------------------------------------
			functionName="Open CAM";
			lJoystickMsg="Top Button 4 Pushed";

			if (camLmtSwOpen.Get() == kLmtSwFalse ){	// We have NOT hit the OPEN limit Switch yet
					camSpike.Set(Relay::kReverse);		// Start motor moving back to open position
					camOpenFlag = true;					// set true to continue moveing till open
					}
			else { // we HAVE hit the OPEN limit switch so lets stop
				Wait(kCamWait);							// give time to fully seat at limit
				camSpike.Set(Relay::kOff);				// Stop the motorh
				camOpenFlag = false;					// Clear the Flag - we are done moving
				}			
			} //-------- end Open Cam -------------

		if (lstick.GetRawButton(kCloseCamBtn)== true or camCloseFlag == true)
			{
			// --------------- CLOSE Cam (R-Joystick Button 5 top right) -------------------
			// -----------------------------------------------------------------------------

			functionName="Close CAM";
			lJoystickMsg="Top Button 5 Pushed";
			if (camLmtSwClosed.Get() == kLmtSwFalse ){	// We have NOT hit the CLOSED limit switchg
					camSpike.Set(Relay::kForward);		// Keep moving forward to closed position
					camCloseFlag = true;				// Set true to continue moving till closed
					}
			else {	// We have now hit the CLOSED limit switch lets stop the motor
				Wait(kCamWait);							// give time to fully seat at limit
				camSpike.Set(Relay::kOff);				// Stop the motorh
				camCloseFlag = false;					// set the flag to false we we are done
				}		
			} // --------- end Closecam ---------------------
	}	//-------- end updateCam routine -------------
	
	
	// ----------------------------------------------------------------------------------
	// ------ RESET Arm to initial position (L&R-Joystick Button 6 Lwr/Left/Front) --------
	// ----------------------------------------------------------------------------------
	void checkResetArm()
	{
		if ((lstick.GetRawButton(kInitArmBtn) == true) and
			(rstick.GetRawButton(kInitArmBtn) == true))
				{
				initArmPosition();
				}
	}	//--------end checkRestArm routine -------------


	// ----------- Initialize the Arm Position -------------	
	// 		consider disabeling this function when climbing
	void initArmPosition()
	{
		while (hookLiftLmtSwTop.Get() == kLmtSwFalse){
			hookLiftMotor.Set(kInitHookLiftSpeed);		// move motor to top
			myRobot.ArcadeDrive(0.0, 0.0);			 	// stop drive motors
			Wait(0.005);								// for command 
		}
		Wait (kHookLiftWait);							// Give motors time to set switch
		hookLiftMotor.Set(0.0);							// stop motor were at limit
		hookLiftEnc.Reset();							// Reset internal cntrs to zero
		hookLiftEnc.Start();

		while (armLmtSwBack.Get() == kLmtSwFalse){
			armMotor.Set(kInitArmSpeed);				// move arm to back
			myRobot.ArcadeDrive(0.0, 0.0);				// stop drive motors
			Wait(0.005);
		}
		Wait (kArmWait);								// Give motor time to set switch
		armMotor.Set(0.0);								// stop motor
		armEnc.Reset();									// Reset internal counters to zero
		armEnc.Start();
		
		while (camLmtSwOpen.Get() == kLmtSwFalse){
			camSpike.Set(Relay::kReverse);				// move Cam open
			myRobot.ArcadeDrive(0.0, 0.0);				 // stop drive motors
			Wait(0.005);
		}
		Wait (kCamWait);								// Time to seat limit switch
		camSpike.Set(Relay::kOff);						// stop motor
	} // -------------------- end initArm Routine ---------------------


	
	// ***********************************************************************************
	//                  Test Throttle Routine to Display Joystick Values                 *
	// ***********************************************************************************
	
	void testThrottle(void)
	{
		functionName="Test Throttle";						
		
		// -------- scale speed ------
		rThrottle = ((rstick.GetThrottle() - 1)/-2);
		msg1 =  "Throttle=" + float_to_string(rstick.GetThrottle());			
		msg1 += "   rThrottle=" + float_to_string(rThrottle);
			
		rSpeed = rstick.GetY() * rThrottle;
		lJoystickMsg =  "Yaxis=" + float_to_string(rstick.GetY());			
		lJoystickMsg += "   rSpeed=" + float_to_string(rSpeed);
			
		rTurn = rstick.GetX() * rThrottle;
		rJoystickMsg =  "Xaxis=" + float_to_string(rstick.GetX());
		rJoystickMsg += "    rTurn=" + float_to_string(rTurn);
			
			// calculate a deadband
		msg2 =  " Deadband=" + float_to_string(kDriveDeadband);
		msg2 += " Scaled (Dbnd * rThrottle)=" + float_to_string(kDriveDeadband * rThrottle);
		if (rSpeed > (- kDriveDeadband * rThrottle) and rSpeed < (kDriveDeadband * rThrottle))
			rSpeed = 0;
		if (rTurn > (-kDriveDeadband * rThrottle) and rTurn < (kDriveDeadband * rThrottle))
			rTurn = 0;
		Wait(0.005); // wait for a motor update time
	
		msg2 += " rSpeed Final=" + float_to_string(rSpeed);
	} // ------------- end of testThrottle routine ----------
	
		
	// ***********************************************************************************
	//                              Display Update Routines                              *
	// ***********************************************************************************

	void updateSmartDash(void)
	{
		SmartDashboard::PutString("Program",kProgramName);		
		// ----------- update function and joystick Display -------------			
		SmartDashboard::PutString("Function",functionName);
		SmartDashboard::PutString("R-Joystick",rJoystickMsg);				
		SmartDashboard::PutString("L-Joystick",lJoystickMsg);
		
		testInputs();								// temp test of inputs into msg1 & 2

		SmartDashboard::PutString("Msg1",msg1);
		SmartDashboard::PutString("Msg2",msg2);
		
		// ----------------------- update HookLift Display -------------------------
		SmartDashboard::PutNumber("H.L. Position",(float)hookLiftEnc.GetDistance());
		
		if (hookLiftLmtSwTop.Get() == kLmtSwFalse)
			SmartDashboard::PutString("H.L Limit Sw. Top","Open");
		else
			SmartDashboard::PutString("H.L Limit Sw. Top","Closed");
		if (hookLiftLmtSwBottom.Get() == kLmtSwFalse)
			SmartDashboard::PutString("H.L Limit Sw. Bottom","Open");
		else
			SmartDashboard::PutString("H.L Limit Sw. Bottom","Closed");
	
		// ------------------------- update Arm Display ------------------------------	
	
		SmartDashboard::PutNumber("Arm Position",(float)armEnc.GetDistance());
		if (armLmtSwBack.Get() == kLmtSwFalse)
			SmartDashboard::PutString("Arm Limit Sw. Back","Open");
		else
			SmartDashboard::PutString("Arm Limit Sw. Back","Closed");
		if (armLmtSwForward.Get() == kLmtSwFalse)
			SmartDashboard::PutString("Arm Limit Sw. Forward","Open");
		else
			SmartDashboard::PutString("Arm Limit Sw. Forward","Closed");
	
		// ---------------- update Cam Display ----------------------------	
		
		if (camLmtSwOpen.Get() == kLmtSwFalse)
			SmartDashboard::PutString("Cam Limit Sw. Open","Open");
		else
			SmartDashboard::PutString("Cam Limit Sw. Open","Closed");
		if (camLmtSwClosed.Get() == kLmtSwFalse)
			SmartDashboard::PutString("Cam Limit Sw. Closed","Open");
		else
			SmartDashboard::PutString("Cam Limit Sw. Closed","Closed");
	}	// -------------- end updateArmPositionData routine -------------------
	

	// ***********************************************************************************
	//                        Convert a float number to a string                         *
	// ***********************************************************************************

	string float_to_string (float f)
	{
	    int prec;
		ostringstream s;
		if (f >= 100000 ) prec = 16;
		   else if (f >= 10000) prec = 11;
		        else if (f >= 1000 ) prec = 10;
		             else if (f >= 100) prec = 9;
		                  else if ( f >= 10) prec = 8;
		                       else prec = 7;
		                       
		s.precision(prec);
		s << f;
		return s.str();
	} // end float_to_string
	
	
	// ***********************************************************************************
	//                      Check input switches and update msg1 and msg2 fields         *
	// ***********************************************************************************

	void testInputs(void)
	{
	if (armLmtSwBack.Get() == true)
		msg1="In1=1 ";
	else
		msg1="In1=0 ";
	if (armLmtSwForward.Get() == true)
		msg1 += "In2=1 ";
	else
		msg1 += "In2=0 ";
	if (hookLiftLmtSwTop.Get() == true)
		msg1 += "In3=1 ";
	else
		msg1 += "In3=0 ";
	if (hookLiftLmtSwBottom.Get() == true)
		msg1 += "In4=1 ";
	else
		msg1+="In4=0 ";
	if (camLmtSwOpen.Get() == true)
		msg1 += "In5=1 ";
	else
		msg1 += "In5=0 ";
	if (camLmtSwClosed.Get() == true)
		msg1 += "In6=1 ";
	else
		msg1 += "In6=0 ";
	if (in7.Get() == true)
		msg1 += "In7=1 ";
	else
		msg1 += "In7=0 ";
	
	if (in8.Get() == true)
		msg2 = "In8=1 ";
	else
		msg2 = "In8=0 ";
	if (in9.Get() == true)
		msg2 += "In9=1 ";
	else
		msg2 += "In9=0 ";
	if (in10.Get() == true)
		msg2 += "In10=1 ";
	else
		msg2 += "In10=0 ";
	if (hookLiftEncA.Get() == true)
		msg2 += "In11=1 ";
	else
		msg2 += "In11=0 ";
	if (hookLiftEncB.Get() == true)
		msg2 += "In12=1 ";
	else
		msg2 += "In12=0 ";
	if (armEncA.Get() == true)
		msg2 += "In13=1 ";
	else
		msg2 += "In13=0 ";
	if (armEncB.Get() == true)
		msg2 += "In14=1 ";
	else
		msg2 += "In14=0 ";
	} // ----- end test Inputs ----------
	
}; // end of Class

START_ROBOT_CLASS(RobotDemo);

