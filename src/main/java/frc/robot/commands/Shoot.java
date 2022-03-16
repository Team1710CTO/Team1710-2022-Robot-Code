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

  public PhotonVisionSubsystem photonVisionSubsystem;

  public DrivetrainSubsystem drivetrainSubsystem;

  public PIDController rotationController;

  public Shoot(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, IndexerSubsystem indexerSubsystem, PhotonVisionSubsystem photonVisionSubsystem, DrivetrainSubsystem drivetrainSubsystem) {

    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.photonVisionSubsystem = photonVisionSubsystem;

    this.drivetrainSubsystem = drivetrainSubsystem;

    rotationController = new PIDController(.2, .15, 0);


    addRequirements(shooterSubsystem, hoodSubsystem, indexerSubsystem, photonVisionSubsystem, drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double d = photonVisionSubsystem.getDistanceToGoalMeters(0.0) + 8;


    if(photonVisionSubsystem.hasGoalTargets()){

    if(d>96){

      hoodSubsystem.setHoodPosition(1.1);

    } else {
      
      hoodSubsystem.setHoodPosition((.0073 * d) + .388);
    }

    if(d>96){

      shooterSubsystem.setSpeed(10.1*d + 2864);

    } else {

      shooterSubsystem.setSpeed((3700 + (-10.3*d) + (.129 * (d*d))));

    }
    
  }
    

    drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, -rotationController.calculate(photonVisionSubsystem.getXDisplacementOfGoal())));

    if (shooterSubsystem.isShooterToSpeedAndNotDisabled()) {

        indexerSubsystem.runindexerInFAST();

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
