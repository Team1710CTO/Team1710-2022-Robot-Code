// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class ZeroIntake extends CommandBase {
  /** Creates a new ZeroIntake. */

  public static IntakeSubsystem intakeSubsystem;

  public ZeroIntake(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intakeSubsystem.runIntakeDown(-.1);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    intakeSubsystem.zeroRotations();
    intakeSubsystem.runIntakeDown(0);
    intakeSubsystem.setIntakeUp();
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(intakeSubsystem.isIntakeStalledCurrent()){

      

      return true;

  } else {

      return false;

  }

  }
}
