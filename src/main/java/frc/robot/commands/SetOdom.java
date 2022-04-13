// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class SetOdom extends CommandBase {
  /** Creates a new ZeroOdom. */

  DrivetrainSubsystem drivetrainSubsystem;

  GyroSubsystem gyroSubsystem;

  PathPlannerTrajectory traj;

  public SetOdom(PathPlannerTrajectory traj ,GyroSubsystem gyroSubsystem, DrivetrainSubsystem drivetrainSubsystem) {

    this.traj = traj;

    this.gyroSubsystem = gyroSubsystem;

    this.gyroSubsystem = gyroSubsystem;

    this.drivetrainSubsystem = drivetrainSubsystem;

    addRequirements(gyroSubsystem, drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    gyroSubsystem.zeroBestGyro();
    gyroSubsystem.setIsZeroingFalse();
    drivetrainSubsystem.setOdometry(traj.getInitialPose());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
