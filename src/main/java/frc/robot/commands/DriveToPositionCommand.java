// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class DriveToPositionCommand extends CommandBase {
  /** Creates a new DriveToPositionCommand. */
  public static DrivetrainSubsystem drivetrainSubsystem;

  public static Pose2d currentPos2d, desiredPose2d;

  public static double xNow, yNow, rotNow;

  public static PIDController xPidController, yPidController, thetaPidController;


  public DriveToPositionCommand(DrivetrainSubsystem drivetrainSubsystem, Pose2d desiredPose2d) {

    this.drivetrainSubsystem = drivetrainSubsystem;

    xPidController = new PIDController(.1, .01, 0);

    yPidController = new PIDController(.1, .01, 0);

    thetaPidController = new PIDController(.01, 0, 0);

    thetaPidController.enableContinuousInput(0, 360);

    

    this.desiredPose2d = desiredPose2d;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //drivetrainSubsystem.m_odometry.resetPosition(new Pose2d(0,0, null), new Rotation2d(0));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xNow = drivetrainSubsystem.m_odometry.getPoseMeters().getX();
    double yNow = drivetrainSubsystem.m_odometry.getPoseMeters().getY();
    double rotNow = GyroSubsystem.getBestRotation2d().getDegrees();
    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      xPidController.calculate(xNow, desiredPose2d.getX()), 
      yPidController.calculate(yNow, desiredPose2d.getY()),  
      0,
      GyroSubsystem.getBestRotation2d()));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if((Math.abs(xNow - desiredPose2d.getX())) < .1 && (Math.abs(yNow - desiredPose2d.getY())) < .1){
      
      return true;

    }

    return false;

  }
}
