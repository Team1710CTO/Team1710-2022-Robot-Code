// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ServoSubsystem;

public class climberActuatorIn extends CommandBase {
  /** Creates a new climberActuatorControl. */

  private final ServoSubsystem m_servoSubsystem;
  public climberActuatorIn(ServoSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_servoSubsystem = subsystem;
    addRequirements(m_servoSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ServoSubsystem.setClimberActuatorIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
