// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.nio.file.WatchEvent;
import java.time.Duration;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shootNscoot extends SequentialCommandGroup {

  public IntakeSubsystem intakeSubsystem;

  public DrivetrainSubsystem drivetrainSubsystem;

  public PhotonVisionSubsystem photonVisionSubsystem;

  public IndexerSubsystem indexerSubsystem;

  public HoodSubsystem hoodSubsystem;
 
  public ShooterSubsystem shooterSubsystem;

  public GyroSubsystem gyroSubsystem;

  public double waitFor;

  public BooleanSupplier gSupplier;

  private PIDController xPosPidController, yPosPidController;
  private ProfiledPIDController thetaPidController;

  /** Creates a new runPathAndIntake. */
  public shootNscoot(Double waitFor, DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem, PhotonVisionSubsystem photonVisionSubsystem, IndexerSubsystem indexerSubsystem, HoodSubsystem hoodSubsystem, ShooterSubsystem shooterSubsystem, GyroSubsystem gyroSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    this.gyroSubsystem = gyroSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.photonVisionSubsystem = photonVisionSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    gSupplier = () -> photonVisionSubsystem.getDistanceToGoalMeters(0.0) > 82;

    this.waitFor = waitFor;

    xPosPidController = new PIDController(1, 0, 0);
    yPosPidController = new PIDController(1, 0, 0);
    thetaPidController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(3,3));
    thetaPidController.enableContinuousInput(-Math.PI, Math.PI);
    
    

    addCommands(

    new ZeroCommand(drivetrainSubsystem, intakeSubsystem, indexerSubsystem, hoodSubsystem, gyroSubsystem),

    new WaitCommand(waitFor),

    new DeadReckonDrive(drivetrainSubsystem, 1, 0, 0, 5).withInterrupt(gSupplier),

    new ShootInAuto(2, shooterSubsystem, hoodSubsystem, indexerSubsystem, photonVisionSubsystem, drivetrainSubsystem)
                                  
  );


  }
}
