// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class intakeRunner extends SubsystemBase {
  /** Creates a new intakeRunner. */

  public static TalonFX mytalon;

  public intakeRunner() {
    mytalon = new TalonFX(43); //fixme
  }

  @Override
  public void periodic() {
    if(RobotContainer.m_controller.getRightBumper()){
      mytalon.set(ControlMode.PercentOutput, 1);
  } else {
      mytalon.set(ControlMode.PercentOutput, 0);
  }
    // This method will be called once per scheduler run
  }
}
