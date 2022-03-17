// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class driveToVisionPos extends CommandBase {
  /** Creates a new driveToVisionPos. */
  public DrivetrainSubsystem drivetrainSubsystem;

  public double d;

  public PhotonVisionSubsystem photonVisionSubsystem;

  public PIDController xPidController, yPidController;

  public driveToVisionPos(DrivetrainSubsystem drivetrainSubsystem, PhotonVisionSubsystem photonVisionSubsystem) {

    this.photonVisionSubsystem = photonVisionSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;

    xPidController = new PIDController(.04, .0, 0);
    yPidController = new PIDController(.2, .15, 0);

    addRequirements(drivetrainSubsystem, photonVisionSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    d = photonVisionSubsystem.getDistanceToGoalMeters(0.0) + 8;

    if(photonVisionSubsystem.hasGoalTargets()){

      drivetrainSubsystem.drive(new ChassisSpeeds(xPidController.calculate(d-70), 0, yPidController.calculate(photonVisionSubsystem.getXDisplacementOfGoal())));

    } else {

      drivetrainSubsystem.drive(new ChassisSpeeds(-2, 0, 0));

    }

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(70-d) < 10;
  }
}
