// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;




import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public int shots = 0;

  public boolean Lastbol = false;

  public Shoot(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, IndexerSubsystem indexerSubsystem, PhotonVisionSubsystem photonVisionSubsystem, DrivetrainSubsystem drivetrainSubsystem) {

    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.photonVisionSubsystem = photonVisionSubsystem;

    this.drivetrainSubsystem = drivetrainSubsystem;

    rotationController = new PIDController(.08, 0.025, 0);


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

    if(Lastbol != indexerSubsystem.topBeamBreak.get() && !Lastbol){

      shots += 1;

    }

    SmartDashboard.putNumber("shots", shots);

    double d = photonVisionSubsystem.getDistanceToGoalMeters(0.0) + 5;

    if(photonVisionSubsystem.hasGoalTargets()){

        
        hoodSubsystem.setHoodPosition(.208 + .00568 * d - (.00000945 * (d*d)));
       

        shooterSubsystem.setSpeed(4.63*d + 2050);

        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, rotationController.calculate(photonVisionSubsystem.getXDisplacementOfGoal())));


      if (shooterSubsystem.isShooterToSpeedAndNotDisabled()) {

          indexerSubsystem.runIndexerInMed();
  
      } else {
  
        indexerSubsystem.stopIndexer();
  
      }

    }
    


    Lastbol = indexerSubsystem.topBeamBreak.get();

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
