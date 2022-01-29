// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControlModeSubsystem extends SubsystemBase {

  private static ControlModeSubsystem instance = null;
  /** Creates a new ControlModeSubsystem. */

  private static boolean diagnosticMode, testMode, drivePracticeMode, developerMode, highPreformanceMode, matchMode = false;


  public ControlModeSubsystem() {

    SmartDashboard.putString("ControlMode", "NA");
    
    testMode = true;

  }


  public static ControlModeSubsystem getInstance() {
		if(instance == null)
			instance = new ControlModeSubsystem();

		return instance;	
	}

  @Override
  public void periodic() {

  //  if (diagnosticMode){
  //    SmartDashboard.putString("ControlMode", "Diagnostic");
  //  } else if (testMode){
  //    SmartDashboard.putString("ControlMode", "test");
  //  } else if (drivePracticeMode){
  //    SmartDashboard.putString("ControlMode", "Drive Practice");
  //  } else if (developerMode){
  //    SmartDashboard.putString("ControlMode", "developer");
  //  } else if (highPreformanceMode){
  //    SmartDashboard.putString("ControlMode", "highpreformance");
  //  } else if (matchMode){
  //    SmartDashboard.putString("ControlMode", "match");
  //  } else {
  //    SmartDashboard.putString("ControlMode", "NA");
  //  }

  }

  public static boolean isDeveloperOrTestMode(){
    
    if(developerMode || testMode){
      return true;
    } else {
      return false;
    }

  }

}
