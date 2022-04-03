// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeForDuration extends CommandBase {
  /** Creates a new IntakeForDuration. */

  public IntakeSubsystem intakeSubsystem;

  public Timer timer;

  public double duration;
  
  public IntakeForDuration(double duration, IntakeSubsystem intakeSubsystem) {

    timer = new Timer();

    this.duration = duration;

    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.reset();
    timer.start();

    intakeSubsystem.setintakeDown();
    intakeSubsystem.runIntake();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    intakeSubsystem.setIntakeUp();
    intakeSubsystem.intakeRest();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get()>duration;
  }
}
