// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SetOdom extends CommandBase {

  DrivetrainSubsystem drivetrainSubsystem;

  Pose2d pose2d;

  Rotation2d rotation2d;

  /** Creates a new SetOdom. */
  public SetOdom(Pose2d pose2d, Rotation2d rotation2d, DrivetrainSubsystem drivetrainSubsystem) {

    this.drivetrainSubsystem = drivetrainSubsystem;
    this.pose2d = pose2d;
    this.rotation2d = rotation2d;
    addRequirements(drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    drivetrainSubsystem.setOdometry(pose2d, rotation2d);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
