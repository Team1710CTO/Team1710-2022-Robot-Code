// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class shootToDeJam extends CommandBase {
  /** Creates a new shootToDeJam. */
  public ShooterSubsystem shooterSubsystem;

  public HoodSubsystem hoodSubsystem;

  public IndexerSubsystem indexerSubsystem;

  public shootToDeJam(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, IndexerSubsystem indexerSubsystem) {

    this.shooterSubsystem =shooterSubsystem;

    this.hoodSubsystem = hoodSubsystem;

    this.indexerSubsystem = indexerSubsystem;

    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooterSubsystem.setSpeed(2000);
    hoodSubsystem.setHoodPosition(1);

    indexerSubsystem.runIndexerIn();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    hoodSubsystem.setHoodPosition(.1);
    indexerSubsystem.stopIndexer();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
