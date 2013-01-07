#include "WPILib.h"
#include "math.h"
#include "DashboardDataSender.h"
#define MINIMUM_SCORE 0.005


void CheckSolenoid(void);
void OmniDrive(GenericHID*, GenericHID*);
/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	RobotDrive myRobot; // robot drive system
	Joystick *stick, *stick2; // only joystick DISREGARD THAT I SUCK COCKS!!!!!!!!!
	Solenoid *firingMechanism;
	Compressor *compressor;
	DashboardDataSender *dds;

public:
	RobotDemo(void): 
		myRobot(1, 3, 2, 4)	// these must be initialized in the same order
		{
			GetWatchdog().SetExpiration(0.1);
			stick = new Joystick(1);				// as they are declared above
			stick2 = new Joystick(2);
			firingMechanism = new Solenoid(7, 1);
			compressor = new Compressor(1, 6);
			dds = new DashboardDataSender();
		}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		GetWatchdog().SetEnabled(false);
		while(IsAutonomous())
		{
			//DoCamera();
			for (int i = 0; i < 2; i++)
			{
				myRobot.HolonomicDrive(.25, 0,  0);
				Wait(.5);
				//fire the solenoid
				firingMechanism->Set(true);
				Wait(.05);
				firingMechanism->Set(false);
				myRobot.HolonomicDrive(.25, 180	, 0);
				Wait(.5);
				myRobot.HolonomicDrive(.25, 90	, 0);
				Wait(.5);
				myRobot.HolonomicDrive(0, 0, 0);
			}
		}
	}

	bool firingEnabled;
	bool fastSpeedEnabled;
	Timer timer;
	/**
	 * Runs the awesome wheels steering and does checks on the solenoid
	 * firing and compressor
	 */
	void OperatorControl(void) 
	{
		myRobot.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		//myRobot.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		AxisCamera &camera = AxisCamera::GetInstance();
		camera.WriteResolution(AxisCamera::kResolution_320x240);
		camera.WriteCompression(20);
		camera.WriteBrightness(5);
		firingEnabled = FALSE;
		fastSpeedEnabled = FALSE;
		timer.Start();
		GetWatchdog().SetEnabled(true);
		while (IsOperatorControl())
		{
			GetWatchdog().Feed();
			OmniDrive(stick, stick2);
			CheckSolenoid();
			CheckCompressor();
			Wait(0.005);				// wait for a motor update time
		}
	}
	
	void DoCamera()
	{
		AxisCamera &camera = AxisCamera::GetInstance();
		
		if (camera.IsFreshImage())
		{
			timer.Reset();
			// get the camera image
			HSLImage *image = camera.GetImage();
			

			// find FRC targets in the image
			vector<Target> targets = Target::FindCircularTargets(image);
			delete image;
			if (targets.size() == 0 || targets[0].m_score < MINIMUM_SCORE)
			{
				// no targets found. Make sure the first one in the list is 0,0
				// since the dashboard program annotates the first target in green
				// and the others in magenta. With no qualified targets, they'll all
				// be magenta.
				Target nullTarget;
				nullTarget.m_majorRadius = 0.0;
				nullTarget.m_minorRadius = 0.0;
				nullTarget.m_score = 0.0;
				if (targets.size() == 0)
					targets.push_back(nullTarget);
				else
					targets.insert(targets.begin(), nullTarget);
				dds->sendVisionData(stick->GetY(), 0.0, 0.0, 0.0, targets);
			}
			else {
				// We have some targets.
				// set the new PID heading setpoint to the first target in the list
				//double horizontalAngle = targets[0].GetHorizontalAngle();
				//double setPoint = horizontalAngle;

				//turnController.SetSetpoint(setPoint);
				
				// send dashbaord data for target tracking
				dds->sendVisionData(0.0, 0.0, 0.0, targets[0].m_xPos / targets[0].m_xMax, targets);
			}
			printf("Time: %f\n", 1.0 / timer.Get());
		}
	}
	
	/**
	 * Checks the appropriate controller to see if the solenoid should be fired
	 * if so takes action accordingly
	 */
	void CheckSolenoid()
	{
		if (stick == NULL || stick2 == NULL)
		{
			wpi_fatal(NullParameter);
			return;
		}
		if (stick->GetRawButton(3))
		{
			firingEnabled = !firingEnabled;
			while(stick->GetRawButton(3))
			{ }
		}
		if (firingEnabled && stick->GetRawButton(1))
			firingMechanism->Set(true);
		else
			firingMechanism->Set(false);
	}
	
	/**
	 * Checks the comressor switch and starts it when enabled
	 */
	void CheckCompressor()
	{
		UINT32 valve = compressor->GetPressureSwitchValue();	
		if (valve)
			compressor->Stop();
		else
			compressor->Start();
	}
	
	/**
	 * Because mecanum wheels are that awesome, this takes input and 
	 * drives the robot accordingly
	 */
	void OmniDrive(GenericHID *leftStick, GenericHID *rightStick)
	{
		if (leftStick == NULL || rightStick == NULL)
		{
			wpi_fatal(NullParameter);
			return;
		}
		if (stick->GetRawButton(2))
		{
			fastSpeedEnabled = !fastSpeedEnabled;
			while(stick->GetRawButton(2))
			{ }
		}
		float leftYValue = fastSpeedEnabled ? -leftStick->GetY() : -leftStick->GetY() / 2;
		float leftXValue = fastSpeedEnabled ? leftStick->GetX() : leftStick->GetX() / 2;
		float magnitude = sqrt((leftYValue * leftYValue) + (leftXValue * leftXValue));
		if (magnitude < .1)
			magnitude = 0;
		if (leftXValue > -.1 && leftXValue < .1)
			leftXValue = .00001;
		if (leftYValue > -.1 && leftYValue < .1)
			leftYValue = .00001;
		float direction = (180 / 3.14159) * atan(leftXValue/leftYValue);
		//if in the lower quadrants...
		if (leftYValue < 0.0)
			direction += 180.0;
		//rotation is based on the second joystick
		float rotation = rightStick->GetX();
		if (rotation < .1 && rotation > -.1)
			rotation = 0;
		myRobot.HolonomicDrive(magnitude, direction,  rotation);
		
	}
};

START_ROBOT_CLASS(RobotDemo);

