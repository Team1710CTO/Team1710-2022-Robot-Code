// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FunkeyFiveBall extends SequentialCommandGroup {

  public IntakeSubsystem intakeSubsystem;

  public DrivetrainSubsystem drivetrainSubsystem;

  public PhotonVisionSubsystem photonVisionSubsystem;

  public IndexerSubsystem indexerSubsystem;

  public HoodSubsystem hoodSubsystem;
 
  public ShooterSubsystem shooterSubsystem;

  public GyroSubsystem gyroSubsystem;

  public PathPlannerTrajectory pathPlannerTrajectory;


  private PIDController xPosPidController, yPosPidController;
  private ProfiledPIDController thetaPidController;
  
  /** Creates a new FunkeyFiveBall. */
  public FunkeyFiveBall(String teamColor, DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem, PhotonVisionSubsystem photonVisionSubsystem, IndexerSubsystem indexerSubsystem, HoodSubsystem hoodSubsystem, ShooterSubsystem shooterSubsystem, GyroSubsystem gyroSubsystem) {

    this.gyroSubsystem = gyroSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.photonVisionSubsystem = photonVisionSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    

    xPosPidController = new PIDController(1, 0, 0);
    yPosPidController = new PIDController(1, 0, 0);
    thetaPidController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(3,3));
    thetaPidController.enableContinuousInput(-Math.PI, Math.PI);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
                  new ParallelCommandGroup(
                                            new ZeroIndexer(indexerSubsystem), 
                                            new ZeroHood(hoodSubsystem), 
                                            new ZeroIntake(intakeSubsystem),
                                            new ZeroOdom(gyroSubsystem, drivetrainSubsystem),
                                            new SetOdom(PathPlanner.loadPath("CoreysCrotch", 8, 5).getInitialState().poseMeters, drivetrainSubsystem)
                                            ),

                  new IntakeWithVision(intakeSubsystem, drivetrainSubsystem, photonVisionSubsystem, indexerSubsystem)
                                
                );
  }
}
