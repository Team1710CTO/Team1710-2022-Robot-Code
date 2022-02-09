// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Servo;

public class ClimberSubsystem extends SubsystemBase {

  private TalonFX climberTalonTop, climberTalonBottom;
  private static Servo lockingServo;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    lockingServo = new Servo(Constants.CLIMBER_SERVO_PWM_CHANNEL);
    lockingServo.setPosition(Constants.CLIMBER_SERVO_LOCK_POSITION);
    lockingServo.setDisabled();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run



  }

  public static void lockClimber(){

    lockingServo.setAngle(Constants.CLIMBER_SERVO_LOCK_POSITION);

  }

  public static void disengageClimber(){

    lockingServo.setAngle(Constants.CLIMBER_SERVO_DISENGAGE_POSITION);

  }

  public static void disableClimber(){

    lockingServo.setDisabled();

  }

  public static boolean climberDrumIsRotating(){

    return false;

  }


}
