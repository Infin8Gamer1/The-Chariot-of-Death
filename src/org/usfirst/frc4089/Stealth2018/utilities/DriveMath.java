//----------------------------------------------------------------------------
//
//  $Workfile: DriveMath.java$
//
//  $Revision: X$
//
//  Project:    Stealth Libraries
//
//                            Copyright (c) 2018
//                           Cedarcrest High School
//                            All Rights Reserved
//
//  Modification History:
//  $Log:
//  $
//
//----------------------------------------------------------------------------
package org.usfirst.frc4089.Stealth2018.utilities;

import java.util.ArrayList;

import org.usfirst.frc4089.Stealth2018.Constants;

//----------------------------------------------------------------------------
//Class Declarations
//----------------------------------------------------------------------------
//
//Class Name: DriveMath
//
//Purpose:
//  Misc math functions
//
//----------------------------------------------------------------------------
public class DriveMath {

  // --------------------------------------------------------------------
  // Purpose:
  // 
  // Notes:
  // None.
  // --------------------------------------------------------------------
    public static double calcDist(double inVal, String unit){
    	double val;
    	if (unit =="FT"){
    		val = Constants.encoderConversionDisFT*inVal;
    	} else if (unit == "IN"){
    		val = Constants.encoderConversionDisIN*inVal;
    	} else {
    		val = 0;
    		System.out.println("DriveMath calcDist No Units");
    	}
    	  	    	
    	return val;
    }

    // --------------------------------------------------------------------
    // Purpose:
    // 
    // Notes:
    // None.
    // --------------------------------------------------------------------
    public static double calcVel(double inVal){
    	double val = inVal;
    	return val;
    }
    
    // --------------------------------------------------------------------
    // Purpose:
    // 
    // Notes:
    // None.
    // --------------------------------------------------------------------
    public static double calcTurns(double inVal){
    	double val;
    	val = inVal/Constants.E4TEncoderPulsePerRev;
    	return val;
	}
    
    // --------------------------------------------------------------------
    // Purpose:
    // Return the value with a dead band where it is zero
    //
    // Notes:
    // None.
    // --------------------------------------------------------------------
    static public double DeadBand(double axisVal, double value) {
      if (axisVal < -value) {
        return axisVal;
      }

      if (axisVal > value) {
        return axisVal;
      }

      return 0;
    }

    // --------------------------------------------------------------------
    // Purpose:
    // Return the value with a dead band where it is zero
    //
    // Notes:
    // None.
    // --------------------------------------------------------------------
    static public double DeadBand(double axisVal) {
      if (axisVal < -0.10) {
        return axisVal;
      }

      if (axisVal > +0.10) {
        return axisVal;
      }

      return 0;
    }

    // --------------------------------------------------------------------
    // Purpose:
    // Trim the top and bottom off of a number
    //
    // Notes:
    // None.
    // --------------------------------------------------------------------
    static public double Cap(double value, double peak) {
      if (value < -peak) {
        return -peak;
      }

      if (value > +peak) {
        return +peak;
      }
      return value;
    }

    // --------------------------------------------------------------------
    // Purpose:
    // Keep angles in a circle. if the angle goes over pi then
    // wrap it around to -pi
    //
    // Notes:
    // None.
    // --------------------------------------------------------------------
    static public double KeepAsCircle(double value) {
      if (value < -Math.PI) {
        value = value + (Math.PI * 2.0);
      }

      if (value > Math.PI) {
        value = value - (Math.PI * 2.0);
      }
      return value;
    }

    // --------------------------------------------------------------------
    // Purpose:
    // Change the angle from rad to deg
    //
    // Notes:
    // None.
    // --------------------------------------------------------------------
    static public double DegToRad(double angle) {
      return angle * Math.PI / 180.0;
    }

    // --------------------------------------------------------------------
    // Purpose:
    // Change the angle from deg to red
    //
    // Notes:
    // None.
    // --------------------------------------------------------------------
    static public double RadToDeg(double angle) {
      return angle * 180.0 / Math.PI;
    }
    
    // --------------------------------------------------------------------
    // Purpose:
    // Given the robot forward throttle and ratio, return the max
    // corrective turning throttle to adjust for heading.  This is
    // a simple method of avoiding using different gains for
    // low speed, high speed, and no-speed (zero turns).
    //
    // Notes:
    // None.
    // --------------------------------------------------------------------
    static public double MaxCorrection(double forwardThrot, double scalor) {
      /* make it positive */
      if(forwardThrot < 0) {forwardThrot = -forwardThrot;}
      /* max correction is the current forward throttle scaled down */
      forwardThrot *= scalor;
      /* ensure caller is allowed at least 10% throttle,
       * regardless of forward throttle */
      if(forwardThrot < 0.10)
        return 0.10;
      return forwardThrot;
    }
 
    /*static public Path GeneratePath(ArrayList<MPPoint> list)
    {
    	Path path = new Path();
    	path.kSpeed = 60;
    	path.kNumPoints = list.size() - 1;
    	path.kPoints = new double[list.size() - 1][3];
    	MPPoint first = list.get(0);
    	double lastLeftPos = first.ticksL;
    	double lastRightPos = first.ticksR;
    	for (int c = list.size() - 1; c >= 1; c++)
    	{
    		path.kPoints[c - 1][0] = (list.get(c).ticksL - lastLeftPos) * Constants.encoderConversionDisIN;
    		path.kPoints[c - 1][1] = (list.get(c).ticksR - lastRightPos) * Constants.encoderConversionDisIN;
    		path.kPoints[c - 1][2] = list.get(c).heading;
    	}
    	return path;
    }*/
}

