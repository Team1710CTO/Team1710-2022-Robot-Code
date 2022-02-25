// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class DriveToPositionCommand extends CommandBase {
  /** Creates a new DriveToPositionCommand. */
  public DrivetrainSubsystem drivetrainSubsystem;

  public Pose2d desiredPose2d;


  public DriveToPositionCommand(DrivetrainSubsystem drivetrainSubsystem, Pose2d desiredPose2d) {

    this.drivetrainSubsystem = drivetrainSubsystem;

    this.desiredPose2d = desiredPose2d;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    drivetrainSubsystem.resetOdometry();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drivetrainSubsystem.DriveToPosition(desiredPose2d);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(drivetrainSubsystem.isInPosition(desiredPose2d)){
      
      return true;

    }

    return false;

  }
}
