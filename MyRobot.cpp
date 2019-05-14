// **************************************************************************************
//       SimpleTemplate - MyRobot2014 ver 1.cpp		2/10/13		6:00 PM
//
//		version 2013-1	12-7-13	New code for final challenge - cannon shooter
//
//
//
// **************************************************************************************
#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Math.h"
#include <iostream>
#include <sstream>
#include <string>

const string kProgramName = "Simple Template - MyRobot 2014 Version 1 (12/07/2013)";

const UINT32 kLmtSwTrue = 0;			// Limit sw is currently pressed
const UINT32 kLmtSwFalse = 1;			// Limit sw is currently NOT pressed

const UINT32 kDriveBtn = 1;				// Button 1 (Trigger) R-Joy to drive
const UINT32 kElevBtn = 2;				// Button 2 Raise/Lower Cannon
const UINT32 kFireBtn = 3;				// Button to Start spinning up shooter motor
const UINT32 kFireConfirmBtn = 6;		// Button to confirm fire
const UINT32 kSpinUpMtrBtn = 4;			// Button to Start spinning up shooter motor
const UINT32 kSpinDnMtrBtn = 5;			// Button to Stop spinning the shooter motor
const UINT32 kInitCannonBtn = 11;		// Button to reset cannon to lowered position
const UINT32 kRingLedOnBtn = 8;			// Button to turn on LED Ring Light
const UINT32 kRingLedOffBtn = 9;		// Button to turn off LED Ring Light

const float	 kDriveDeadband = 0.28;		// scale constant for Drive deadband zone
const float  kTurnRate = 1.2;			// scale the turn rate by this amount

//Camera constants used for distance calculation
#define X_IMAGE_RES 320					// X Image resolution in pixels, should be 160, 320 or 640
#define VIEW_ANGLE 48					// Axis M1013 camera
//#define VIEW_ANGLE 48					// Axis 206 camera
//#define VIEW_ANGLE 43.5  				// Axis M1011 camera
#define PI 3.141592653

//Score limits used for target identification
#define RECTANGULARITY_LIMIT 60
#define ASPECT_RATIO_LIMIT 75
#define X_EDGE_LIMIT 40
#define Y_EDGE_LIMIT 60

//Minimum area of particles to be considered
#define AREA_MINIMUM 500

//Edge profile constants used for hollowness score calculation
#define XMAXSIZE 24
#define XMINSIZE 24
#define YMAXSIZE 24
#define YMINSIZE 48
const double xMax[XMAXSIZE] = {1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, 1, 1, 1, 1};
const double xMin[XMINSIZE] = {.4, .6, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, 0.6, 0};
const double yMax[YMAXSIZE] = {1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, 1, 1, 1, 1};
const double yMin[YMINSIZE] = {.4, .6, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
								.05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
								.05, .05, .6, 0};


class RobotDemo : public SimpleRobot
{

	struct Scores {	//Structure to represent the scores for the various tests used for target identification
		double rectangularity;
		double aspectRatioInner;
		double aspectRatioOuter;
		double xEdge;
		double yEdge;
	};

	Joystick rstick;					// right joystick
	Joystick lstick;	 				// left joystick

	RobotDrive myRobot;					// robot drive system
	
	Jaguar elevMotor;					// This spike can raise/lwr cannon barrel
	Relay ringLedSpike;					// Ring LED light for camera
	Jaguar launchMotors;				// Spinning wheels for shooting the ball
	Relay plungerMotor;					// Motor to push tennis ball into chamber
	

	DigitalInput plungerLmtSw;			// Test for plunger back load cannon
	DigitalInput elevLmtSwHi;			// Test for cannon raised to top 
	DigitalInput elevLmtSwLo;			// Test for cannon raised to bottom
	//	AnalogInput elevVal;			// pot analog 1	

	DriverStationLCD *dsLCD;

	float aThrottle, aSpeed, aTurn;
	float rThrottle, rSpeed, rTurn;
	float hThrottle, hSpeed, hTurn;
	double timeLast, cannonTimer, fireTimer;
	string functionName;	// Store last function name
	string rJoystickMsg;	// Store last right joystick press
	string lJoystickMsg;	// Store last left joystick press
	string msg1;			// Store a message
	string msg2;			// second message
	
	Scores *scores;

public:
	
	NetworkTable *table;
	
	RobotDemo(void):
		rstick(1),
		lstick(2),

		myRobot(1, 2),			// PWM Output Port Assignments	
		elevMotor(3),			// PWM Jaguar to raise/lwr cannon barrel
		ringLedSpike(1),		// RELAY - Ring LED Light
		launchMotors(10),		// PWM - Spinning wheels for shooting the ball
		plungerMotor(2),		// RELAY - Motor to push tennis ball into chamber
		plungerLmtSw(1),		// DIGITAL INPUT - Test for plunger back load cannon
		elevLmtSwHi(2),			// DIGITAL INPUT - Test for cannon raised to top 
		elevLmtSwLo(3)			// DIGITAL INPUT - Test for cannon raised to bottom

	{
		table = NetworkTable::GetTable("datatable");
		
		myRobot.SetExpiration(1.0f);	 //TODO REMOVE
		myRobot.SetSafetyEnabled(false); //TODO REMOVE
		GetWatchdog().SetEnabled(false); //TODO REMOVE
		SmartDashboard::init();
		dsLCD = DriverStationLCD::GetInstance();
		
		timeLast = 0;
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

		while (IsOperatorControl() && IsEnabled())
		{
			updateDrive();				// drive robot
			updateLaunchMotors();		// speed up wheels
			updatePlunger();			// push plunger to fire
			updateElev();				// joystick to move cannon up/dn
			checkRingLEDBtn();			// joystick to move cannon up/dn			

			if (Timer::GetFPGATimestamp() - timeLast > .1)		//Update display 10 times a second
			{
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
		
		//HSV threshold criteria, ranges are in that order ie. Hue is 60-100
		Threshold threshold(60, 100, 90, 255, 20, 255);
		
		//Particle filter criteria, used to filter out small particles		
		ParticleFilterCriteria2 criteria[] = {{IMAQ_MT_AREA, AREA_MINIMUM, 65535, false, false}};
		
		AxisCamera &camera = AxisCamera::GetInstance();	//To use the Axis camera uncomment this line
		
		while (IsTest() && IsEnabled()) {
			ringLedOnOff( 1 );						// Turn On LED ring
			Wait(1);								// Wait 1 second and aquire image	

			ColorImage *image;
			camera.GetImage(image);					// Get image

			Wait (1);								// Wait 1 second and turn off LED
			ringLedOnOff( 0 );						// Turn Off LED ring

			 // get just the green target pixels
			BinaryImage *thresholdImage = image->ThresholdHSV(threshold);
			//thresholdImage->Write("/threshold.bmp");

  			// fill in partial and full rectangles
			BinaryImage *convexHullImage = thresholdImage->ConvexHull(false);
			//convexHullImage->Write("/ConvexHull.bmp");
			
			//Remove small particles
			BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria, 1);
			//filteredImage->Write("Filtered.bmp");

			//get a particle analysis report for each particle
			vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();
			scores = new Scores[reports->size()];
			
			//Iterate through each particle, scoring it and determining whether it is a target or not
			for (unsigned i = 0; i < reports->size(); i++) {
				ParticleAnalysisReport *report = &(reports->at(i));
				
				scores[i].rectangularity = scoreRectangularity(report);
				scores[i].aspectRatioOuter = scoreAspectRatio(filteredImage, report, true);
				scores[i].aspectRatioInner = scoreAspectRatio(filteredImage, report, false);			
				scores[i].xEdge = scoreXEdge(thresholdImage, report);
				scores[i].yEdge = scoreYEdge(thresholdImage, report);
				
				if(scoreCompare(scores[i], false))
				{
					printf("particle: %d  is a High Goal  centerX: %f  centerY: %f \n",
							i, report->center_mass_x_normalized, report->center_mass_y_normalized);
					printf("Distance: %f \n", computeDistance(thresholdImage, report, false));
				} else if (scoreCompare(scores[i], true)) {
					printf("particle: %d  is a Middle Goal  centerX: %f  centerY: %f \n",
							i, report->center_mass_x_normalized, report->center_mass_y_normalized);
					printf("Distance: %f \n", computeDistance(thresholdImage, report, true));
				} else {
					printf("particle: %d  is not a goal  centerX: %f  centerY: %f \n",
							i, report->center_mass_x_normalized, report->center_mass_y_normalized);
				}
				printf("rect: %f  ARinner: %f \n",
						scores[i].rectangularity, scores[i].aspectRatioInner);
				printf("ARouter: %f  xEdge: %f  yEdge: %f  \n",
						scores[i].aspectRatioOuter, scores[i].xEdge, scores[i].yEdge);	
			}
			printf("\n");
			
			Wait (120);			//	Wait 120 seconds to give time to download data
			// be sure to delete images after using them
			delete filteredImage;
			delete convexHullImage;
			delete thresholdImage;
			delete image;
			
			//delete allocated reports and Scores objects also
			delete scores;
			delete reports;
		} // end of while loop
	} // end of test routine
		

	// ****************************************************************
	// *                   AUTONOMOUS MODE                            *
	// ****************************************************************
	void Autonomous(void)
	{
		dsLCD->Printf(DriverStationLCD::Line(1),1,"Autonomous Mode Running");
		dsLCD->UpdateLCD();		
		myRobot.SetSafetyEnabled(false);
		
		while (IsAutonomous() && !IsEnabled()) Wait(0.1);	// wait here till enabled
			
		// reset cannon to lowered position
		msg2 ="Auto Step 1 - Speed up Wheels & Reset Cannon";
		updateSmartDash();
		Wait (2);
		launchMotors.Set(1.0);					// Start Spinning wheels
		ResetCannon();							// Lower cannnon to bottom (gives known spot to start)
		Wait (2.0);								// Give 2 seconds for wheels to come up to speed
		
		// go forward to starting section lined up on first target
		msg2 ="Auto Step 2 - Move up to Firing area";
		updateSmartDash();
		Wait (2);
		moveRobotTank ( 0.5, 0.5, 2, 1);		// Move forward half speed 2 seconds and stop
		Wait (2);								// Give 2 second to stop
		
		// fire on first target (Center)
		msg2 ="Auto Step 3 - Raise Cannon and FIRE **1** !";
		updateSmartDash();
		Wait (2);
		moveRobotTank ( -0.2 , 0.2 , 0.25 , 1);// Point turn left 1/4 speed for 1/2 second and stop 
		moveCannonUpDn( 0.25, 2.0);				// raise cannon for 2 seconds at 1/2 speed
		fireShot();
		Wait (2);								// Wait 2 seconds to reset for next shot
		
		// turn and fire on second target (left)
		msg2 ="Auto Step 4 - Turn to SECOND target reposition cannon and FIRE **2**";
		updateSmartDash();
		Wait (2);
		moveRobotTank ( 0.25 , -0.25 , 0.5 , 1);// Point turn left 1/4 speed for 1/2 second and stop 
//		moveCannonUpDn( 0.1, 0.5);				// raise cannon a smidge ( 1 seconds at 1/2 speed)
		fireShot();	
		Wait (2);								// Wait 2 seconds to reset for next shot
				
		// turn and fire on third target (right)
		msg2 ="Auto Step 5 - Turn to THIRD target reposition cannon and FIRE **3**";
		updateSmartDash();
		Wait (2);
		moveRobotTank ( 0.25 , -0.25 , 0.50 , 1);// Point turn right 1/4 speed for 1/2 second and stop 		
		fireShot();	
		Wait (2);								// Wait 2 seconds to reset for next shot
		
		// Shut down fire motors and exit
		msg2 ="Auto Step 6 - Shut down fire motors - THATS ALL !!!!!";
		updateSmartDash();
		Wait (2);
		launchMotors.Set(0.0);					// Stop Spinning wheels
		Wait (5);								// Give time for motors to stop
	}
	
	
	// *******************************************************************************
	// *                         Operator Control Routines                           *				  
	// *******************************************************************************

	
	// ---------------------------------------------------------------------------
	// -------------------- DRIVE ROBOT By Joystick & Right Trigger --------------
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

	// -----------------------------------------------------------------------
	// -----------   Drive Robot by Autonomous Command      ------------------
	// -----------------------------------------------------------------------
	
	void moveRobotTank (float rLeftSpeed, float rRightSpeed, float rTime, int rBrake=1 )
	{	// Used in the autonomous routine to move the robot
		// rLeftSpeed = left motor Speed    rRightSpeed = right motor Speed
		// rTime = How long in seconds to drive
		// rBrake = Flag to indicate wehther to stop motor or allow next command
		// to change motor speed defaults to 1
		
		myRobot.TankDrive(rLeftSpeed, rRightSpeed); 			// send drive speed to drive class
		Wait(rTime);
		if (rBrake == 1) myRobot.TankDrive( 0.0, 0.0); 			// stop robot if brake is set
	}
	
	
	// -------------------------- Fire Control Code ------------------------------
	//                  Spin up wheels or stop wheels spinning
	// ---------------------------------------------------------------------------
	void updateLaunchMotors()
	{
		if (rstick.GetRawButton(kSpinUpMtrBtn) == true){	//spin up aunch motors
			launchMotors.Set(1.0); 
			rJoystickMsg = "Turning on Launcher motors";
		}
		if (rstick.GetRawButton(kSpinDnMtrBtn) == true){	//turn off launch motors
			launchMotors.Set(0.0);
			rJoystickMsg = "Turning off Launcher motors";
		}
	}
	
	// ------------------------ Fire Control Code --------------------------------
	//                    Check for fire button pressed
	// ---------------------------------------------------------------------------
	void updatePlunger() 
	{
		if ((rstick.GetRawButton(kFireBtn)== true) && (rstick.GetRawButton(kFireConfirmBtn)== true))
		{	// you are pressing the fire button so fire
			fireShot();
			while (rstick.GetRawButton(kFireBtn)== true){
				Wait(.05);						// wait for button to be released
												// to avoid multiple shots
			}
		}
	}
	
	// ------------------------ Fire Control Code --------------------------------
	//          drive the plunger forward to push a tennis ball into cannon
	// ---------------------------------------------------------------------------
	void fireShot()
	{
		rJoystickMsg="Fireing";
												// this assumes plunger is already back
		plungerMotor.Set(Relay::kForward);		// move forward to push ball into chamber
		Wait(0.25);								// give time for ball to get into chamber
		plungerMotor.Set(Relay::kOff);			// stop motor
		plungerMotor.Set(Relay::kReverse);		// Reverse plunger to start position
		
		fireTimer = Timer::GetFPGATimestamp();	// setup time out 
		//
		fireTimer = Timer::GetFPGATimestamp();
		while((plungerLmtSw.Get() == kLmtSwFalse) &&(Timer::GetFPGATimestamp() - fireTimer < .8)) {
			Wait(0.05);							// wait while plunger returns to start position		
		} // end while
		plungerMotor.Set(Relay::kOff);			// stop motor now
	}
	
	
	// ----------------------------------------------------------------------------------
	// -----------------------------  Turn On/Off Ring LEDS -----------------------------
	// ----------------------------------------------------------------------------------
	
	// --------------------------- check for LED button pressed --------------------
	void checkRingLEDBtn()
	{
		if (rstick.GetRawButton(kRingLedOnBtn)== true)
		{	functionName="Update Ring LED - ON";				
			ringLedOnOff( 1 );								// Turn On LED ring
		}
		if (rstick.GetRawButton(kRingLedOnBtn)== true)
		{	functionName="Update Ring LED OFF";
			ringLedOnOff( 0 );								// Turn Off LED ring
		}
	}
		
	// ------------------------ Drive LED Ring on/off -----------------------------	
	void ringLedOnOff( int ledFlag )
	{
		if (ledFlag == 0){
			ringLedSpike.Set(Relay::kOff);
		}
		else	{
			ringLedSpike.Set(Relay::kForward);
		}
	}
				
	
	// ----------------------------------------------------------------------------------
	// ----------------------------------- Cannon Routines ------------------------------
	// ----------------------------------------------------------------------------------

	void checkResetCannon()	// ------------- Has reset button been pressed ----------------
	{
		if (rstick.GetRawButton(kInitCannonBtn) == true) ResetCannon();	
	}
	// -----------------------------------------------------------------------------------------
	void ResetCannon()	// ---------- Reset cannon to lowered position ---------------------
	{	// this routine reses the cannon to the low position
		while (elevLmtSwLo.Get() == kLmtSwFalse) elevMotor.Set(-0.5);		// Drive cannon down
		elevMotor.Set(0.0);													// stop motor at bottom	
	}
	// -----------------------------------------------------------------------------------------
	void updateElev()	// --------- Cannon Elevation by Joystick Routine ----------------
	{		
		if (rstick.GetRawButton(kElevBtn)== true)
		{	functionName="Raise Lower cannon";				
			rJoystickMsg="Top Button Pushed";				

			// ------------------------- scale speed -------------------------------------
			rThrottle = ((rstick.GetThrottle() - 1)/-2);	// converts(+1 -> -1) to (0 -> 1)	
			rSpeed = rstick.GetY() * rThrottle;			

			// ----- check limit switches, speed direction and update motor speed --------
			if (rSpeed == 0.0){								// speed = 0 so stop the motor
				elevMotor.Set(0.0);
			}
			else if (rSpeed < 0){							// speed < 0 therefore move arm FORWARD
				
				if (elevLmtSwLo.Get() == kLmtSwFalse){		// Are we fully forward ?
					elevMotor.Set(rSpeed);				// Start motor moving back to open position	
				}
				else{ // cannon has hit the LOW limit switch so STOP the motor
					Wait(0.1);								// give time to fully seat at limit
					elevMotor.Set(0.0);						// stop motor
					msg2 ="Elev stop 2";
				}
			}
			else											// speed >0 so move cannon up
			{
				if (elevLmtSwHi.Get() == kLmtSwFalse){		// Are we fully back?
					elevMotor.Set(rSpeed);					// No, so we can move backwards		
				}
				else { 										// cannon has hit the High limit switch so STOP
					Wait(0.1);								// give time to fully seat at limit
					elevMotor.Set(0.0);						// stop motor
				}
			}
		}
		else
		{// The joystick button is not pushed so stop motors
			elevMotor.Set(0.0);
		}
	} // -------- end of  routine -------------

	// -------------------------------------------------------------------------------------
	// --------- Cannon Elevation by Autonomous Command ----------------
	void moveCannonUpDn(float cannonSpeed, float cannonAngle)
	{	// cannonAngle = positive/negative number to indicate the angle to move to (currently use a
		// timer since we don't currently have a sensor in place to measure the actual angle of the cannon
		//
		// Speed determins how fast the motor should run (always given as a positive number)
		//
		// the sign of the angle will determine the positive or negative of the speed
		// check limit switchs to not over drive cannon
		functionName="Raise Lower cannon";				
		rJoystickMsg="Top Button Pushed";
		
		cannonTimer = Timer::GetFPGATimestamp();;
		if (cannonAngle < 0){					// Negative angle means drive down
			cannonSpeed = cannonSpeed * -1;
			cannonAngle = cannonAngle * -1;
		}
		while (cannonTimer > (Timer::GetFPGATimestamp() - cannonTimer))
		{ 	//	while loop is to continously move motors for the given time duration				
			// ----- check limit switches, speed direction and update motor speed --------
			if (cannonSpeed == 0.0)								// speed = 0 so stop the motor
				elevMotor.Set(0.0);
			else if (cannonSpeed < 0){							// speed < 0 therefore move arm FORWARD
				if (elevLmtSwLo.Get() == kLmtSwFalse){			// Are we fully forward ?
					elevMotor.Set(cannonSpeed);					// Start motor moving back to open position	
				}
				else{ 											// cannon has hit the LOW limit switch so ....
					elevMotor.Set(0.0);							// STOP the motor
				}
			}
			else												// speed >0 so move cannon up
			{
				if (elevLmtSwHi.Get() == kLmtSwFalse){			// Are we fully back?
					elevMotor.Set(cannonSpeed);						// No, so we can move backwards		
				}
				else { 											// cannon has hit the High limit switch so ...
					elevMotor.Set(0.0);							// STOP the motor
				}
			}
		} 														// end of while loop based on timer
		elevMotor.Set(0.0);										// stop elevation motor
	} // -------- end of  routine -------------


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
		
		testInputs();		// temp test of inputs into msg1 & 2

		SmartDashboard::PutString("Msg1",msg1);
		SmartDashboard::PutString("Msg2",msg2);
		
		// ----------------------- update Limit Switch Display -------------------------
		if (plungerLmtSw.Get() == kLmtSwFalse)
			SmartDashboard::PutString("Plunger Limit Sw. ","Open");
		else
			SmartDashboard::PutString("Plunger Limit Sw. ","Closed");
		if (elevLmtSwHi.Get() == kLmtSwFalse)
			SmartDashboard::PutString("Elevation Limit Sw. Hi ","Open");
		else
			SmartDashboard::PutString("Elevation Limit Sw. Hi ","Closed");
		if (elevLmtSwLo.Get() == kLmtSwFalse)
			SmartDashboard::PutString("Elevation Limit Sw. Lo ","Open");
		else
			SmartDashboard::PutString("Elevation Limit Sw. Lo ","Closed");
	}	// -------------- End Update SmartDisplay routine -------------------
	
	// ***********************************************************************************
	//                      Check input switches and update msg1 and msg2 fields         *
	// ***********************************************************************************

	void testInputs(void)
	{
		if (plungerLmtSw.Get() == true)
			msg1="In1=1 ";
		else
			msg1="In1=0 ";
		if (elevLmtSwHi.Get() == true)
			msg1 += "In2=1 ";
		else
			msg1 += "In2=0 ";
		if (elevLmtSwLo.Get() == true)
			msg1 += "In3=1 ";
		else
			msg1 += "In3=0 ";
		// msg2="";
	} // ----- end test Inputs ----------
	
	
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
	
	
	// ****************************************************************************************	
	// ************************************* Vision Processing Routines ***********************
	// ****************************************************************************************
	
	
	//  ----------------------------------------------------------------------------
	// ---------------------- Compute Distance to Target ---------------------------
	// -----------------------------------------------------------------------------
	 /**
	 * Computes the estimated distance to a target using the height of the particle in the image.
	 * For more information and graphics showing the math behind this approach see the Vision
	 * Processing section of the ScreenStepsLive documentation.
	 * @param image The image to use for measuring the particle estimated rectangle
	 * @param report The Particle Analysis Report for the particle
	 * @param outer True if the particle should be treated as an outer target, false to treat it
	 *   as a center target
	 * @return The estimated distance to the target in Inches.
	 */
	double computeDistance (BinaryImage *image, ParticleAnalysisReport *report, bool outer) {
		double rectShort, height;
		int targetHeight;
		
		imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0,
				IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
		// using the smaller of the estimated rectangle short side and the bounding rectangle
		// height results in better performance on skewed rectangles
		height = min(report->boundingRect.height, rectShort);
		targetHeight = outer ? 29 : 21;
		return X_IMAGE_RES * targetHeight / (height * 12 * 2 * tan(VIEW_ANGLE*PI/(180*2)));
	}

	
	//  ----------------------------------------------------------------------------
	// ---------------------- Compute Aspect Ratio of Targets ----------------------
	// -----------------------------------------------------------------------------
	/**
	 * Computes a score (0-100) comparing the aspect ratio to the ideal aspect ratio for the target.
	 * This method uses the equivalent rectangle sides to determine aspect ratio as it performs
	 * better as the target gets skewed by moving to the left or right. The equivalent rectangle
	 * is the rectangle with sides x and y where particle area= x*y and particle perimeter= 2x+2y
	 * 
	 * @param image The image containing the particle to score, needed to perform additional
	 *    measurements
	 * @param report The Particle Analysis Report for the particle, used for the width, height,
	 *    and particle number
	 * @param outer	Indicates whether the particle aspect ratio should be compared to the ratio
	 *    for the inner target or the outer
	 * @return The aspect ratio score (0-100)
	 */
	double scoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report, bool outer){
		double rectLong, rectShort, idealAspectRatio, aspectRatio;
		
		//Dimensions of goal opening + 4 inches on all 4 sides for reflective tape
		idealAspectRatio = outer ? (62/29) : (62/20);
		
		imaqMeasureParticle(image->GetImaqImage(),
				report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
		imaqMeasureParticle(image->GetImaqImage(),
				report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
		
		//Divide width by height to measure aspect ratio
		if(report->boundingRect.width > report->boundingRect.height){
			//particle is wider than it is tall, divide long by short
			aspectRatio = 100*(1-fabs((1-((rectLong/rectShort)/idealAspectRatio))));
		} else {
			//particle is taller than it is wide, divide short by long
			aspectRatio = 100*(1-fabs((1-((rectShort/rectLong)/idealAspectRatio))));
		}
		return (max(0, min(aspectRatio, 100)));		//force to be in range 0-100
	}
	
	
	//  ----------------------------------------------------------------------------
	// ------------------ Compares Scores of Particals to Targets ------------------
	// -----------------------------------------------------------------------------
	/**
	 * Compares scores to defined limits and returns true if the particle appears to be a target
	 * 
	 * @param scores The structure containing the scores to compare
	 * @param outer True if the particle should be treated as an outer target, false to treat it
	 *         as a center target
	 * @return True if the particle meets all limits, false otherwise
	 */
	bool scoreCompare(Scores scores, bool outer){
		bool isTarget = true;
		isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
		if(outer){
			isTarget &= scores.aspectRatioOuter > ASPECT_RATIO_LIMIT;
		} else {
			isTarget &= scores.aspectRatioInner > ASPECT_RATIO_LIMIT;
		}
		isTarget &= scores.xEdge > X_EDGE_LIMIT;
		isTarget &= scores.yEdge > Y_EDGE_LIMIT;
		return isTarget;
	}
	
	
	//  ----------------------------------------------------------------------------
	// -------------------- Score Rectangularity of Partical -----------------------
	// -----------------------------------------------------------------------------
	/**
	 * Computes a score (0-100) estimating how rectangular the particle is by comparing
	 * the area of the particle to the area of the bounding box surrounding it. A
	 * perfect rectangle would cover the entire bounding box.
	 * 
	 * @param report The Particle Analysis Report for the particle to score
	 * @return The rectangularity score (0-100)
	 */
	double scoreRectangularity(ParticleAnalysisReport *report){
		if(report->boundingRect.width*report->boundingRect.height !=0){
			return 100*report->particleArea/(report->boundingRect.width*report->boundingRect.height);
		} else {
			return 0;
		}	
	}
	
	
	//  ----------------------------------------------------------------------------
	// -------------------------- ScoreXEdge of Partical ---------------------------
	// -----------------------------------------------------------------------------
	/**
	 * Computes a score based on the match between a template profile and the particle profile in the
	 * X direction. This method uses the column averages and the profile defined at the top of the 
	 * sample to look for the solid vertical edges with a hollow center.
	 * 
	 * @param image The image to use, should be the image before the convex hull is performed
	 * @param report The Particle Analysis Report for the particle
	 * @return The X Edge Score (0-100)
	 */
	double scoreXEdge(BinaryImage *image, ParticleAnalysisReport *report){
		double total = 0;
		LinearAverages *averages = imaqLinearAverages2(image->GetImaqImage(),
				IMAQ_COLUMN_AVERAGES, report->boundingRect);
		for(int i=0; i < (averages->columnCount); i++){
			if(xMin[i*(XMINSIZE-1)/averages->columnCount] < averages->columnAverages[i] 
			   && averages->columnAverages[i] < xMax[i*(XMAXSIZE-1)/averages->columnCount]){
				total++;
			}
		}
		total = 100*total/(averages->columnCount);		//convert to score 0-100
		imaqDispose(averages);							//let IMAQ dispose of the averages struct
		return total;
	}

	
	//  ----------------------------------------------------------------------------
	// -------------------------- ScoreYEdge of Partical ---------------------------
	// -----------------------------------------------------------------------------
	/**
	 * Computes a score based on the match between a template profile and the particle profile
	 * in the Y direction. This method uses the the row averages and the profile defined at the
	 * top of the sample to look for the solid horizontal edges with a hollow center
	 * 
	 * @param image The image to use, should be the image before the convex hull is performed
	 * @param report The Particle Analysis Report for the particle
	 * @return The Y Edge score (0-100)
	 */
	double scoreYEdge(BinaryImage *image, ParticleAnalysisReport *report){
		double total = 0;
		LinearAverages *averages = imaqLinearAverages2(image->GetImaqImage(),
				IMAQ_ROW_AVERAGES, report->boundingRect);
		for(int i=0; i < (averages->rowCount); i++){
			if(yMin[i*(YMINSIZE-1)/averages->rowCount] < averages->rowAverages[i] 
			   && averages->rowAverages[i] < yMax[i*(YMAXSIZE-1)/averages->rowCount]){
				total++;
			}
		}
		total = 100*total/(averages->rowCount);		//convert to score 0-100
		imaqDispose(averages);						//let IMAQ dispose of the averages struct
		return total;
	}			
	
}; // end of Class

START_ROBOT_CLASS(RobotDemo);

