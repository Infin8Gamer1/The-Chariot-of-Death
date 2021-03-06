// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc4089.Stealth2018;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

import org.usfirst.frc4089.Stealth2018.utilities.Modes;

import com.ctre.phoenix.sensors.PigeonIMU;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */

public class Logging {
	
	private FileWriter logMatch;
	private FileWriter logSystems;
	private FileWriter logError;
	private FileWriter logEvents;
	
	private long StartTime;
	
	private Date date;
	
	public Logging() {
		
		date = new Date();
		
		StartTime = date.getTime();
		
		try {
			logMatch = new FileWriter("/LOGS/Chariot/logMatch.csv", true);
			logSystems = new FileWriter("/LOGS/Chariot/logSystems.csv", true);
			logError = new FileWriter("/LOGS/Chariot/logError.csv", true);
			logEvents = new FileWriter("/LOGS/Chariot/logEvents.csv", true);
		} catch(IOException e) {
			e.printStackTrace();
	        System.out.println("Unable to create/find FileWriter");
	    }
	}
	
	private Modes currentMode;
	
	public void SetMode(Modes input) {
		currentMode = input;
	}
	
	
	public void Log() {
		LogMatch();
		LogSystems();
		LogErrors();
	}
	
	public void LogEvent(String input) {
		System.out.println("LogEvent Input: " + input);
		
		try {
			//start Time, System Time, input
			logEvents.write(
					StartTime + "," +
					date.getTime() + "," +
							
					input 
					
					+ "\n"
			);
		} catch(IOException e) {
			e.printStackTrace();
	        System.out.println("Unable to write to LogEvent");
	    }
		
	}
	
	
	private void LogMatch() {
		
		try {
			//System Start Time, System Time, 
			//Match Time, isFMSConnected, Event Name, Match Number, Match Type, Alliance, Replay Number, Game Specific Message
			logMatch.write(
					StartTime + "," +
					date.getTime() + "," +
					
					Timer.getMatchTime() + "," +
					DriverStation.getInstance().isFMSAttached() + "," +
					DriverStation.getInstance().getEventName() + "," +
					DriverStation.getInstance().getMatchNumber() + "," +
					DriverStation.getInstance().getMatchType() + "," +
					DriverStation.getInstance().getAlliance() + "," +
					DriverStation.getInstance().getReplayNumber() + "," +
					DriverStation.getInstance().getGameSpecificMessage() 
					
					+ "\n"
			);
		} catch(IOException e) {
			e.printStackTrace();
	        System.out.println("Unable to write to LogMatch");
	    }
	}
	
	private void LogSystems() {
		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
		double [] xyz_dps = new double [3];
	    RobotMap.pigeonIMU.getRawGyro(xyz_dps);
    	RobotMap.pigeonIMU.getFusedHeading(fusionStatus);
		
		try {
			//System Start Time, System Time, Mode, isDriverStationConnected,
			//Battery Voltage, PDP input voltage, PDP Temperature Celsius, PDP Total Energy
			//Heading, Current Angular Rate, Gyro Up Time, Gyro Temperature,
			//Front Ultrasonic distance (inches), average Speed,
			//DriveLR Power, DriveLR Output %, DriveLF Power, DriveLF Output %, DriveL Encoder Position, DriveL Encoder Velocity,
			//DriveRR Power, Drive RR Output %, DriveRF Power, DriveRF Output %, DriveR Encoder Position, DriveR Encoder Velocity,
			logSystems.write(
					StartTime + "," +
					date.getTime() + "," +
					currentMode.toString() + "," +
					DriverStation.getInstance().isDSAttached() + "," +
					
					RobotController.getBatteryVoltage() + "," +
					//DriverStation.getInstance().getBatteryVoltage()+ "," +
					RobotMap.PDP.getVoltage() + "," +
					RobotMap.PDP.getTemperature() + "," +
					RobotMap.PDP.getTotalPower() + "," +
					
					fusionStatus.heading + "," +
					xyz_dps[2] + "," +
					RobotMap.pigeonIMU.getUpTime() + "," +
					RobotMap.pigeonIMU.getTemp() + "," +
					
					Robot.sensors.GetFrontUltrasonic() + "," +
					Robot.drive.GetSpeed() + "," +
					
					RobotMap.driveSRXDriveLR.getMotorOutputVoltage() + "," +
					RobotMap.driveSRXDriveLR.getMotorOutputPercent() + "," +
					RobotMap.driveSRXDriveLF.getMotorOutputVoltage() + "," +
					RobotMap.driveSRXDriveLF.getMotorOutputPercent() + "," +
					RobotMap.driveSRXDriveLF.getSelectedSensorPosition(0) + "," +
					RobotMap.driveSRXDriveLF.getSelectedSensorVelocity(0) + "," +
					
					RobotMap.driveSRXDriveRR.getMotorOutputVoltage() + "," +
					RobotMap.driveSRXDriveRR.getMotorOutputPercent() + "," +
					RobotMap.driveSRXDriveRF.getMotorOutputVoltage() + "," +
					RobotMap.driveSRXDriveRF.getMotorOutputPercent() + "," +
					RobotMap.driveSRXDriveRF.getSelectedSensorPosition(0) + "," +
					RobotMap.driveSRXDriveRF.getSelectedSensorVelocity(0) + ","
					
					+ "\n"
					);
		} catch(IOException e) {
			e.printStackTrace();
	        System.out.println("Unable to write to LogSystems");
	    }
	}
	
	private void LogErrors() {
		
		try {
			//System Start Time, System Time, 
			//isBrownedOut, isSysActive,
			//Fault Count 3.3v, Fault Count 5v, Fault Count 6v,
			//CAN Status,
			//Gyro Last error, Gyro State
			logError.write(
					StartTime + "," +
					date.getTime() + "," +
					
					RobotController.isBrownedOut() + "," +
					RobotController.isSysActive() + "," +
					
					RobotController.getFaultCount3V3() + "," +
					RobotController.getFaultCount5V() + "," +
					RobotController.getFaultCount6V() + "," +
					
					RobotController.getCANStatus() + "," +
					
					RobotMap.pigeonIMU.getLastError() + "," +
					RobotMap.pigeonIMU.getState()
					
					+ "\n"
					);
		} catch(IOException e) {
			e.printStackTrace();
	        System.out.println("Unable to write to LogError");
	    }
	}
	
	
}
