package org.usfirst.frc4089.Stealth2018;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Constants {
    //features
	public static final boolean UseCamera = false;
	public static final boolean StopSafteyEnabled = true;
	
	//ultrasonic sensor
	public static final int FrontUltrasonicPingChanel = 1;
	public static final int FrontUltrasonicEchoChanel = 2;
	public static final double FrontUltrasonicSafetyStopInches = 6.00;
	
	// CAN Assignments
	public static final int CANTalonSRXDriveLR = 3;
	public static final int CANTalonSRXDriveLF = 1;
	public static final int CANTalonSRXDriveRR = 4;
	public static final int CANTalonSRXDriveRF = 2;
	public static final int CANPDP = 18;

	// User Interface consts
	public static final int kShiftUpButton = 6;
	public static final int kShiftDownButton = 5;
	public static final int kDisplaySensorsButton = 3;
	public static final int kEmergencyStopButton = 4;
	public static final int kDriveControlerSwapButton = 8;
	
	//Dead Bands
	public static final double kDriveSpeedDeadBand = 0.1;
	public static final double kDriveTurnDeadBand = 0.05;
	
	// Drivers Speed
	public static final double kFastSpeed = 0.6;
	public static final double kFastTurnSpeed = 0.4;
	public static final double kNormalSpeed = 0.4;
	public static final double kNormalTurnSpeed = 0.4;
	public static final double kSlowSpeed = 0.3;
	public static final double kSlowTurnSpeed = 0.4;
	
	public static final double kStopedTurnModifier = 1.2;
	public static final double kReverseSpeedModifier = 0.6;
		
	// Gyro Constants
	public static final int kGyroZ = 2;
	public static final double kGyroDeadband = 0.1;
	  
	// Motor setup constants
	public static final int kPIDLoopIdx = 0;
	public static final int kTimeoutMs = 30;
	public static final int kBaseTrajPeriodMs = 0;
	public static final double kNeutralDeadband  = 0.01;
	
	//wheel info
	public static final int wheelDiaIn = 6; //in
	public static final double wheelCircIN = 6*Math.PI; //in
	public static final double wheelCircFT = wheelCircIN/12;
	
	//encoders
	public static final int E4TEncoderPulsePerRev = 1440;
	public static final double encoderRightDistanceFTPerPulse = wheelCircFT/E4TEncoderPulsePerRev;
	public static final double encoderLeftDistanceFTPerPulse = wheelCircFT/E4TEncoderPulsePerRev;
	public static final double encoderConversionDisFT = wheelCircFT/E4TEncoderPulsePerRev; //FT/pulse
	public static final double encoderConversionDisIN = wheelCircIN/E4TEncoderPulsePerRev; //FT/pulse

}