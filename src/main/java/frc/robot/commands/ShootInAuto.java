// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;




import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public final Timer timer, timer2, timer3, timer4, timer5;

  public boolean targetSeen = false;

  public int shots = 0;

  public boolean Lastbol = false;

  public int numOfballs = 0;

  public ShootInAuto(int numOfballs, ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, IndexerSubsystem indexerSubsystem, PhotonVisionSubsystem photonVisionSubsystem, DrivetrainSubsystem drivetrainSubsystem) {

    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.photonVisionSubsystem = photonVisionSubsystem;

    this.drivetrainSubsystem = drivetrainSubsystem;

    this.numOfballs = numOfballs;

    timer = new Timer();

    timer2 = new Timer();

    timer3 = new Timer();

    timer4 = new Timer();

    timer5 = new Timer();

    rotationController = new PIDController(.08, .025, 0);

    shots = 0;

    Lastbol = false;


    addRequirements(shooterSubsystem, hoodSubsystem, indexerSubsystem, photonVisionSubsystem, drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    shots = 0;

    Lastbol = false;

    timer.reset();
    timer2.reset();
    timer3.reset();

    timer4.reset();
    timer4.start();

    timer5.reset();
    timer5.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Lastbol != indexerSubsystem.topBeamBreak.get() && !Lastbol){

      shots += 1;

    }

    Lastbol = false;

    
     
    
    double d = photonVisionSubsystem.getDistanceToGoalMeters(0.0);

    if(photonVisionSubsystem.hasGoalTargets()){

      targetSeen = true;

        
        hoodSubsystem.setHoodPosition(.208 + .00568 * d - (.00000945 * (d*d)));
       

        shooterSubsystem.setSpeed(4.63*d + 2000);

        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, rotationController.calculate(photonVisionSubsystem.getXDisplacementOfGoal())));

    } else if (!targetSeen){

      timer2.start();
      timer3.start();

      if(timer2.get() > .5){
        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 2));
      } else {

        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, -2));
      }

    }

    if(timer3.get()>.1 && targetSeen){
      targetSeen = false;
    }
    

    if(photonVisionSubsystem.hasGoalTargets()){
      
    if (shooterSubsystem.isShooterToSpeedAndNotDisabled() && (Math.abs(photonVisionSubsystem.getXDisplacementOfGoal()) < 5)) {

        indexerSubsystem.runIndexerInMed();

        timer.start();

    } else {

      indexerSubsystem.indexBallsBetweenBreaks();

      timer.stop();

    }

  }

  Lastbol = indexerSubsystem.topBeamBreak.get();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    
    hoodSubsystem.setHoodPosition(0.1);
    indexerSubsystem.stopIndexer();

    shots = 0;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() > .5 && timer4.get() > .1) || timer5.get() > 6;
  }
}
