// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;




import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootInAuto extends CommandBase {
  /** Creates a new Shoot. */

  public ShooterSubsystem shooterSubsystem;

  public HoodSubsystem hoodSubsystem;

  public IndexerSubsystem indexerSubsystem;

  public PhotonVisionSubsystem photonVisionSubsystem;

  public DrivetrainSubsystem drivetrainSubsystem;

  public PIDController rotationController;

  public final Timer timer, timer2, timer3;

  public boolean targetSeen = false;

  public ShootInAuto(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, IndexerSubsystem indexerSubsystem, PhotonVisionSubsystem photonVisionSubsystem, DrivetrainSubsystem drivetrainSubsystem) {

    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.photonVisionSubsystem = photonVisionSubsystem;

    this.drivetrainSubsystem = drivetrainSubsystem;

    timer = new Timer();

    timer2 = new Timer();

    timer3 = new Timer();

    rotationController = new PIDController(.2, .15, 0);


    addRequirements(shooterSubsystem, hoodSubsystem, indexerSubsystem, photonVisionSubsystem, drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.reset();
    timer2.reset();
    timer3.reset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    
    
    double d = photonVisionSubsystem.getDistanceToGoalMeters(0.0) + 8;

    if(photonVisionSubsystem.hasGoalTargets()){

      targetSeen = true;

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

        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, -rotationController.calculate(photonVisionSubsystem.getXDisplacementOfGoal())));

    } else if (!targetSeen){

      timer2.start();
      timer3.start();

      if(timer2.get() > .5){
        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, -2));
      } else {

        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 2));
      }

    }

    if(timer3.get()>.1 && targetSeen){
      targetSeen = false;
    }
    

    if (shooterSubsystem.isShooterToSpeedAndNotDisabled() && hoodSubsystem.isHoodInRange() && Math.abs(photonVisionSubsystem.getXDisplacementOfGoal()) < .5) {

        indexerSubsystem.runindexerInFAST();

        timer.start();

    } else {

      indexerSubsystem.stopIndexer();

      timer.stop();

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
    return timer.get() > .3;
  }
}
