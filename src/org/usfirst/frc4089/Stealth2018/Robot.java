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

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.print.DocFlavor.STRING;

import org.usfirst.frc4089.Stealth2018.commands.*;
import org.usfirst.frc4089.Stealth2018.subsystems.*;
import org.usfirst.frc4089.Stealth2018.utilities.*;

import com.ctre.phoenix.sensors.PigeonIMU;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {
    
    public static OI oi;
    public static Logging logging;

    public static Drive drive;
    public static Sensors sensors;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        RobotMap.init();
        logging = new Logging();
        
        drive = new Drive();
        sensors = new Sensors();
        
        logging.SetMode(Modes.Init);
        logging.LogEvent("Robot Code Init");
        
        RobotMap.pigeonIMU.setFusedHeading(0, 30);
        
        if(Constants.UseCamera) {
	        //start camera server
	        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
	        camera.setResolution(320,240);
	        camera.setFPS(30);
        }
        
        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();
        
        displayDashboard();
        
        System.out.println("robot init");
        
    }
    
    private void displayDashboard() {
    	//put speed on dashboard
    	double speed = -1.00;
    	speed = drive.GetSpeed();
    	SmartDashboard.putNumber("Speed", speed);
    	//put front ultrasonic on dashboard
    	SmartDashboard.putNumber("FrontUltrasonic", sensors.GetFrontUltrasonic());
    	//put Speed Mode on dashboard
    	SmartDashboard.putString("SpeedMode", drive.mSpeedPreset.toString());
    	//put Control Mode on dashboard
    	SmartDashboard.putString("ControlMode", drive.mControlState.toString());
    }


    @Override
    public void robotPeriodic(){
    }
    
    @Override
    public void disabledInit(){
    	
    	logging.SetMode(Modes.Disabled);
      
    }

    
    
    @Override
    public void disabledPeriodic() {
    	//logging.Log();
        Scheduler.getInstance().run();
    }

    
    
    @Override
    public void autonomousInit() {
    	logging.SetMode(Modes.Autonomus);
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
       Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
    	logging.SetMode(Modes.TeleOp);
        
        //Init teleop
        System.out.println("tele init");
        RobotMap.SetUpTalonsForTele();
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
    	logging.Log();
        Scheduler.getInstance().run();
        
        displayDashboard();
        
        Robot.drive.DriveRobot(oi.driveJoystick);
    }
    
    
    @Override
    public void testInit() {
    	logging.SetMode(Modes.Test);
    }   

    @Override
    public void testPeriodic() {
    	logging.Log();
    }   
}
