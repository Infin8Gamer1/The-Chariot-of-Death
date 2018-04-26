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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import org.usfirst.frc4089.Stealth2018.utilities.*;


public class Drive extends Subsystem {
  //----------------------------------------------------------------------------
  //  Class Constants 
  //----------------------------------------------------------------------------
  static final double kPgain = 0.003; /* percent throttle per degree of error */
  static final double kDgain = 0.0002; /* percent throttle per angular velocity dps */
  static final double kMaxCorrectionRatio = 0.20; /* cap corrective turning throttle to 30 percent of forward throttle */
  static final double kSpeedGain = 0.05; // The ramp for the speed
  
  //----------------------------------------------------------------------------
  //  ENUM Constants 
  //----------------------------------------------------------------------------
  // The robot drivetrain's various states.
  public enum DriveControlState {
      OPEN_LOOP, // open loop voltage control
      VELOCITY_SETPOINT, // velocity PID control
      PATH_FOLLOWING, // used for autonomous driving
      AIM_TO_GOAL, // turn to face the boiler
      TURN_TO_HEADING, // turn in place
      DRIVE_TOWARDS_GOAL_COARSE_ALIGN, // turn to face the boiler, then DRIVE_TOWARDS_GOAL_COARSE_ALIGN
      DRIVE_TOWARDS_GOAL_APPROACH // drive forwards until we are at optimal shooting distance
  }
  
  public static enum SpeedPreset {
	  Slow,
	  Medium,
	  Fast,
	  FullSpeed
  }
  
  //----------------------------------------------------------------------------
  //  Class Attributes 
  //----------------------------------------------------------------------------
  double mTargetAngle = 0;
  boolean mSendJoystickCommands = true;   // send the joystick to the drive, we surpress this in auto
  double mCurrentAngle = 0.0;
  double mActualSpeed = 0.0;
  StopWatch mDisplay = new StopWatch(500);
  DriveControlState mState = DriveControlState.OPEN_LOOP;
  public SpeedPreset mSpeedPreset = SpeedPreset.Slow;
  
  public void SetSpeedPreset(SpeedPreset input) {
	  mSpeedPreset = input;
  }
  
  
  public void SetAuto()
  {
    mState = DriveControlState.PATH_FOLLOWING;
  }
  
  
  public void SetTele()
  {
    mState = DriveControlState.OPEN_LOOP;
  }
   
    public void initDefaultCommand() {
    	setDefaultCommand(new UserDrive());
    }

    @Override
    public void periodic() {
      // Do Nothing
    }

    public void SetTargetAngle(double targetAngle)
    {
      mTargetAngle = targetAngle;
    }
    
    
    public void ClearCurrentAngle()
    {
      mCurrentAngle = 0.0;
      RobotMap.pigeonIMU.setFusedHeading(0, 30);
    }
    
    
    public void AutoDrive(double speedInPerTenthSec, double speedL, double speedR, double heading, FileWriter logFile) {
	    PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
	    double [] xyz_dps = new double [3];
	    RobotMap.pigeonIMU.getRawGyro(xyz_dps);
	    RobotMap.pigeonIMU.getFusedHeading(fusionStatus);
	    
	    
	    
	    mCurrentAngle = fusionStatus.heading;
	  
	    if(heading>180)
	    {
	      heading -= 360.0;
	    }
	    
	    double targetSpeedL = speedL*-25.5;
	    double targetSpeedR = speedR*-25.5;
	   
	    double angle_difference = heading - mCurrentAngle;    // Make sure to bound this from -180 to 180, otherwise you will get super large values
	  
	    double turn = 3.0*angle_difference;
	  
	      targetSpeedL += turn;
	      targetSpeedR -= turn;
	    
	    RobotMap.driveSRXDriveLF.set(ControlMode.Velocity, targetSpeedR);
	    RobotMap.driveSRXDriveRF.set(ControlMode.Velocity, targetSpeedL);
	  
	    try {
	      logFile.write(
	          Timer.getFPGATimestamp() + ", " +
	          speedL + ", " +
	          speedR + ", " +
	          heading + ", " +
	          turn + ", " +
	          targetSpeedL + ", " +
	          targetSpeedR + ", " +
	          RobotMap.driveSRXDriveLF.getSelectedSensorVelocity(0) + ", " +
	          RobotMap.driveSRXDriveRF.getSelectedSensorVelocity(0) + ", " +
	          RobotMap.driveSRXDriveLF.getMotorOutputVoltage() + ", " +
	          RobotMap.driveSRXDriveRF.getMotorOutputVoltage() +", " +
	          angle_difference + "\n"
	      );
	    }
	    catch(IOException e) {
	        e.printStackTrace();
	    }
    }

    
    
    public void DriveRobot(Joystick driveJoystick) {
      
      double speed = driveJoystick.getRawAxis(1);
      double turn = driveJoystick.getRawAxis(2);
      
      speed = DriveMath.DeadBand(speed,Constants.kDriveSpeedDeadBand);
      turn = DriveMath.DeadBand(turn,Constants.kDriveTurnDeadBand);
      
      //turn modifier on joystick
      //x *= driveJoystick.getRawAxis(3);
      
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
      
      if (speed <= 0.15 && speed >= -0.15) {
    	  turn *= Constants.kStopedTurnModifyer;
      }
      
      if (speed < -0.15) {
    	  speed *= Constants.kReverseSpeedModifyer;
      }
      
      if(DriveControlState.OPEN_LOOP == mState)
      {
        DriveRobot(speed, turn);
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

    //--------------------------------------------------------------------
    // Purpose:
    //     Set the velocity of the robot 
    //
    // Notes:
    //     None
    //--------------------------------------------------------------------  
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

