//----------------------------------------------------------------------------
//
//  $Workfile: Drive.java$
//
//  $Revision: X$
//
//  Project:    Stealth Libraries
//
//                            Copyright (c) 2018
//                               Cedarcrest High School
//                            All Rights Reserved
//
//  Modification History:
//  $Log:
//  $
//
//----------------------------------------------------------------------------
package org.usfirst.frc4089.Stealth2018.subsystems;

import org.usfirst.frc4089.Stealth2018.RobotMap;

import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc4089.Stealth2018.Constants;
import org.usfirst.frc4089.Stealth2018.Robot;
import org.usfirst.frc4089.Stealth2018.commands.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import org.usfirst.frc4089.Stealth2018.utilities.*;


public class Drive extends Subsystem {
  static final double kPgain = 0.003; /* percent throttle per degree of error */
  static final double kDgain = 0.0002; /* percent throttle per angular velocity dps */
  static final double kMaxCorrectionRatio = 0.20; /* cap corrective turning throttle to 30 percent of forward throttle */
  static final double kSpeedGain = 0.05; // The ramp for the speed
  
  public static enum SpeedPreset {
	  Slow,
	  Medium,
	  Fast,
	  FullSpeed
  }
  
  double mTargetAngle = 0;
  boolean mSendJoystickCommands = true;   // send the joystick to the drive, we surpress this in auto
  double mCurrentAngle = 0.0;
  double mActualSpeed = 0.0;
  double mActualSpeedL = 0.0;
  double mActualSpeedR = 0.0;
  StopWatch mDisplay = new StopWatch(500);
  public SpeedPreset mSpeedPreset = SpeedPreset.Slow;
  
  public void SetSpeedPreset(SpeedPreset input) {
	  mSpeedPreset = input;
  }
   
    public void initDefaultCommand() {
    	setDefaultCommand(new UserDrive());
    }
    
    public void DriveRobot(Joystick driveJoystick) {
    	
    	if(driveJoystick.getType() == HIDType.kHIDJoystick && Robot.EmergencyStop == false)
	    {
		      double speed = driveJoystick.getRawAxis(1);
		      double turn = driveJoystick.getRawAxis(2);
		      
		      speed = DriveMath.DeadBand(speed,Constants.kDriveSpeedDeadBandJoystick);
		      turn = DriveMath.DeadBand(turn,Constants.kDriveTurnDeadBandJoystick);
		      
		      //adjust speed based on selected setting
		      if (mSpeedPreset == SpeedPreset.Slow) {
		    	  speed *= Constants.kSlowSpeed;
		    	  turn *= Constants.kSlowTurnSpeed;
		      } else if (mSpeedPreset == SpeedPreset.Medium) {
		    	  speed *= Constants.kNormalSpeed;
		    	  turn *= Constants.kNormalTurnSpeed;
		      } else if (mSpeedPreset == SpeedPreset.Fast) {
		    	  speed *= Constants.kFastSpeed;
		    	  turn *= Constants.kFastTurnSpeed;
		      } else if (mSpeedPreset == SpeedPreset.FullSpeed) {
		    	  speed *= 1;
		    	  turn *= 1;
		      }
		      //stopped turn modifier
		      if (speed <= 0.15 && speed >= -0.15) {
		    	  turn *= Constants.kStopedTurnModifier;
		      }
		      //reverse speed modifier
		      if (speed < -0.15) {
		    	  speed *= Constants.kReverseSpeedModifier;
		      }
		      
		      //send calculated speed and turn values
		      DriveRobot(speed, turn);
		      
	    } else if(driveJoystick.getType() == HIDType.kHIDGamepad && Robot.EmergencyStop == false)
	    {
    		double SpeedL = driveJoystick.getRawAxis(1);
    		double SpeedR = driveJoystick.getRawAxis(5);
    		
    		DriveMath.DeadBand(SpeedL,Constants.kDriveSpeedDeadBandGamepad);
    		DriveMath.DeadBand(SpeedR,Constants.kDriveSpeedDeadBandGamepad);
    		
    		//adjust speed based on selected setting
		      if (mSpeedPreset == SpeedPreset.Slow) {
		    	  SpeedL *= Constants.kSlowSpeed;
		    	  SpeedR *= Constants.kSlowSpeed;
		      } else if (mSpeedPreset == SpeedPreset.Medium) {
		    	  SpeedL *= Constants.kNormalSpeed;
		    	  SpeedR *= Constants.kNormalSpeed;
		      } else if (mSpeedPreset == SpeedPreset.Fast) {
		    	  SpeedL *= Constants.kFastSpeed;
		    	  SpeedR *= Constants.kFastSpeed;
		      } else if (mSpeedPreset == SpeedPreset.FullSpeed) {
		    	  SpeedL *= 1;
		    	  SpeedR *= 1;
		      }
		      
		    RawTankDriveRobot(SpeedL, SpeedR);
	    }
    }    
    
    public void DriveRobotWithOutGyro(double speed, double turn) {
    	
    	if (Robot.sensors.GetFrontUltrasonic() < Constants.FrontUltrasonicSafetyStopInches && speed < 0.05 && Constants.StopSafteyEnabled) {
      	  speed = 0;
        }
    	
    	RawDriveRobot(speed*0.5, turn*0.5);
    }
    
    public void DriveRobot(double speed, double turn) {  
    	
      if (Robot.sensors.GetFrontUltrasonic() < Constants.FrontUltrasonicSafetyStopInches && speed < 0.05 && Constants.StopSafteyEnabled) {
    	  speed = 0;
      }
      
      PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
      double [] xyz_dps = new double [3];
      RobotMap.pigeonIMU.getRawGyro(xyz_dps);
      RobotMap.pigeonIMU.getFusedHeading(fusionStatus);
      
      mCurrentAngle = fusionStatus.heading;
      double currentAngularRate = xyz_dps[2];
      double turnThrottle = turn;
      
      // IF we are turning, turn off the gyro
      if (Math.abs(turn) > 0.2) {
        RawDriveRobot(speed*.5, turn*.5);
        mTargetAngle = mCurrentAngle;
      } else {
        if (Math.abs(speed) > 0.1) {
          double angleError = (mTargetAngle - mCurrentAngle);
          turnThrottle = angleError * kPgain - (currentAngularRate) * kDgain;
          double maxThrot = DriveMath.MaxCorrection(speed, kMaxCorrectionRatio);
          turnThrottle =  -1*DriveMath.Cap(turnThrottle, maxThrot);
          RawDriveRobot(speed,turnThrottle);
        }
        else
        {
          RawDriveRobot(0, 0);
          mTargetAngle = mCurrentAngle;
        }
      }
    }
    
    public void RawTankDriveRobot (double speedL, double speedR) {
    	// Ramp the speed for l
        if(mActualSpeedL != speedL)
        {
          if(mActualSpeedL<speedL)
          {
            mActualSpeedL = Math.min(mActualSpeedL+kSpeedGain, speedL);
          }
          else
          {
            mActualSpeedL = Math.max(mActualSpeedL-kSpeedGain, speedL);
          }
        }
     // Ramp the speed for r
        if(mActualSpeedR != speedR)
        {
          if(mActualSpeedR<speedR)
          {
            mActualSpeedR = Math.min(mActualSpeedR+kSpeedGain, speedR);
          }
          else
          {
            mActualSpeedR = Math.max(mActualSpeedR-kSpeedGain, speedR);
          }
        }
        
        double targetSpeedL = (mActualSpeedL) * 4000;
        double targetSpeedR = (mActualSpeedR) * 4000;
    	
    	RobotMap.driveSRXDriveLF.set(ControlMode.Velocity, targetSpeedL);
    	RobotMap.driveSRXDriveRF.set(ControlMode.Velocity, targetSpeedR);
    }

    public void RawDriveRobot(double speed, double turn) {
      // Ramp the speed
      if(mActualSpeed != speed)
      {
        if(mActualSpeed<speed)
        {
          mActualSpeed = Math.min(mActualSpeed+kSpeedGain, speed);
        }
        else
        {
          mActualSpeed = Math.max(mActualSpeed-kSpeedGain, speed);
        }
      }
      
      double targetSpeedL = (mActualSpeed + turn) * 4000;
      double targetSpeedR = (mActualSpeed - turn) * 4000;
      
      RobotMap.driveSRXDriveLF.set(ControlMode.Velocity, targetSpeedL);
      RobotMap.driveSRXDriveRF.set(ControlMode.Velocity, targetSpeedR);
    }
    
    public double GetSpeed() {
    	int LeftVelosity = RobotMap.driveSRXDriveLF.getSelectedSensorVelocity(0);
    	int RightVelosity = RobotMap.driveSRXDriveRF.getSelectedSensorVelocity(0);
    	
    	return ((LeftVelosity + RightVelosity) / 2);
    }
}

