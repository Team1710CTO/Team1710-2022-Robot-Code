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
  private Servo lockingServo;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    lockingServo = new Servo(0);
    lockingServo.setPosition(Constants.CLIMBER_SERVO_LOCK_POSITION);
    lockingServo.getPosition();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
