// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase {
  /** Creates a new Shoot. */

  public ShooterSubsystem shooterSubsystem;

  public HoodSubsystem hoodSubsystem;

  public IndexerSubsystem indexerSubsystem;

  public DrivetrainSubsystem drivetrainSubsystem;

  public PhotonVisionSubsystem photonVisionSubsystem;

  public static PIDController rotationPidController, distancePidController;

  public Shoot(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, IndexerSubsystem indexerSubsystem,
      DrivetrainSubsystem drivetrainSubsystem, PhotonVisionSubsystem photonVisionSubsystem) {

    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.hoodSubsystem = hoodSubsystem;

    this.drivetrainSubsystem = drivetrainSubsystem;

    this.photonVisionSubsystem = photonVisionSubsystem;

    addRequirements(shooterSubsystem, hoodSubsystem, indexerSubsystem, drivetrainSubsystem, photonVisionSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    rotationPidController = new PIDController(.3, .0005, .00001);
    distancePidController = new PIDController(.4, .001, 0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooterSubsystem.setSpeed(3750);
    
    hoodSubsystem.setHoodPosition(.75);

    drivetrainSubsystem.drive(new ChassisSpeeds(
        0,
        0,
        -rotationPidController.calculate(photonVisionSubsystem.getXDisplacementOfGoal())));

    if (shooterSubsystem.isShooterToSpeedAndNotDisabled()) {

        indexerSubsystem.runIndexerIn();

    } else {

      indexerSubsystem.stopIndexer();

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    shooterSubsystem.disableShooter();
    hoodSubsystem.setHoodPosition(0.1);
    indexerSubsystem.stopIndexer();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
