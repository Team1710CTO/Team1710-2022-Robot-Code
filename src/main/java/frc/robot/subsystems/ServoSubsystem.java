// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Servo; 



public class ServoSubsystem extends SubsystemBase {
  
  private static Servo climberServo;

  private static ServoSubsystem instance = null;

  /** Creates a new ClimberServo. */
  public ServoSubsystem() {
      climberServo = new Servo(Constants.SERVO_CHANNEL);
      climberServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0); 
  }

  public static ServoSubsystem getInstance() {
		if(instance == null)
			instance = new ServoSubsystem();

		return instance;	
	}

  public static void setClimberAngle(double degrees) { 
		
		climberServo.setAngle(degrees);
		
	}

  public void setClimberAngleFromJoystick(double inValue) { 
		
    double scale = inValue * 180;

		ServoSubsystem.setClimberAngle(scale);
		
	}

  public static void setClimberActuatorIn(){

    climberServo.setSpeed(1.0);

  }

  public static void setClimberActuatorOut(){

    climberServo.setSpeed(-1.0);

  }

  public static boolean climberActuatorIsOut(){

    if(climberServo.getAngle() >= 1){
      return true;
    } else {
      return false;
    }

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
