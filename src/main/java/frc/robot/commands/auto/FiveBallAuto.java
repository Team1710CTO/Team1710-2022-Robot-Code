package frc.robot.commands.auto;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAuto extends SequentialCommandGroup {

  public IntakeSubsystem intakeSubsystem;
  public DrivetrainSubsystem drivetrainSubsystem;
  public PhotonVisionSubsystem photonVisionSubsystem;
  public IndexerSubsystem indexerSubsystem;
  public HoodSubsystem hoodSubsystem;
  public ShooterSubsystem shooterSubsystem;
  public GyroSubsystem gyroSubsystem;
  private PIDController xPosPidController, yPosPidController;
  private ProfiledPIDController thetaPidController;

  /** Creates a new runPathAndIntake. */
  public FiveBallAuto(
    DrivetrainSubsystem drivetrainSubsystem,
    IntakeSubsystem intakeSubsystem,
    PhotonVisionSubsystem photonVisionSubsystem,
    IndexerSubsystem indexerSubsystem,
    HoodSubsystem hoodSubsystem,
    ShooterSubsystem shooterSubsystem,
    GyroSubsystem gyroSubsystem
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    this.gyroSubsystem = gyroSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.photonVisionSubsystem = photonVisionSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    xPosPidController = new PIDController(1, 0, 0);
    yPosPidController = new PIDController(1, 0, 0);

    thetaPidController =
      new ProfiledPIDController(
        0,
        0,
        0,
        new TrapezoidProfile.Constraints(3, 3)
      );

    thetaPidController.enableContinuousInput(-Math.PI, Math.PI);

    addCommands(
      // 0.5s
      new ParallelCommandGroup(
        // Zero gyro, set gyro offset to starting field angle (91.5 +- 3.0 degrees), set odometry
        new ZeroOdomAuto(gyroSubsystem, drivetrainSubsystem),
        new ZeroHood(hoodSubsystem), // 0.2s
        // Zero intake, but leave down (save time from putting up).
        new ZeroIntakeAuto(intakeSubsystem) // 0.5s
      ),
      // Shoot loaded ball immediately
      new FasterShootInAuto(
        shooterSubsystem,
        hoodSubsystem,
        indexerSubsystem,
        photonVisionSubsystem,
        0 // No delay
      ),
      // Path to the next two close balls, while running intake
      // 2s
      new ParallelCommandGroup(
        // Path
        new PPSwerveControllerCommand(
          // Lets be aggressive with our acceleration
          // Should test up until consistency starts to suffer.
          PathPlanner.loadPath(
            "FiveBallV2-Part1",
            DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            10
          ),
          drivetrainSubsystem::getOdomPose2d,
          drivetrainSubsystem.getKinematics(),
          xPosPidController,
          yPosPidController,
          thetaPidController,
          drivetrainSubsystem::setWheelStates,
          drivetrainSubsystem
        ),
        // Intake looking for at-least 2 balls or timeout after 2 seconds
        // Ball might have rolled away, failed to intake, or was displaced.
        new IntakeInPath(indexerSubsystem, intakeSubsystem, 2, 2.0)
      ),
      // Shoot ball 2 & 3
      // 2s  - Would prefer a delay'ed spinup here based on odometry data
      new FasterShootInAuto(
        shooterSubsystem,
        hoodSubsystem,
        indexerSubsystem,
        photonVisionSubsystem,
        0 // No delay
      ),
      // Path to human station
      // 2s
      new PPSwerveControllerCommand(
        PathPlanner.loadPath(
          "FiveBallV2-Part2",
          DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          10
        ),
        drivetrainSubsystem::getOdomPose2d,
        drivetrainSubsystem.getKinematics(),
        xPosPidController,
        yPosPidController,
        thetaPidController,
        drivetrainSubsystem::setWheelStates,
        drivetrainSubsystem
      ),
      // Wait for 2 more balls to intake
      // (since we did not zero ball count, wait until indexer has counted 2 more balls)
      // 2s
      new IntakeInPath(indexerSubsystem, intakeSubsystem, 4, 2.0),
      new ParallelCommandGroup(
        // Path to next shooting location (near starting point)
        // 2.5s
        new PPSwerveControllerCommand(
          PathPlanner.loadPath(
            "FiveBallV2-Part3",
            DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            10
          ),
          drivetrainSubsystem::getOdomPose2d,
          drivetrainSubsystem.getKinematics(),
          xPosPidController,
          yPosPidController,
          thetaPidController,
          drivetrainSubsystem::setWheelStates,
          drivetrainSubsystem
        ),
        // 2s - Would prefer a delay'ed spinup here based on odometry data
        new FasterShootInAuto(
          shooterSubsystem,
          hoodSubsystem,
          indexerSubsystem,
          photonVisionSubsystem,
          1.5 //s delay - before spinning up and locating target
        )
        // ), // Uncomment for red-side fire 6-ball autonomous
        // new ParallelCommandGroup(
        //   // Path to next shooting location (near starting point)
        //   // 2.5s
        //   new PPSwerveControllerCommand(
        //     PathPlanner.loadPath(
        //       "FiveBallV2-Part4",
        //       DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        //       10
        //     ),
        //     drivetrainSubsystem::getOdomPose2d,
        //     drivetrainSubsystem.getKinematics(),
        //     xPosPidController,
        //     yPosPidController,
        //     thetaPidController,
        //     drivetrainSubsystem::setWheelStates,
        //     drivetrainSubsystem
        //   ),
        //   new IntakeInPath(indexerSubsystem, intakeSubsystem, 5, 1.0)
        // ),
        // // 2s - Would prefer a delay'ed spinup here based on odometry data
        // new FasterShootInAuto(
        //   shooterSubsystem,
        //   hoodSubsystem,
        //   indexerSubsystem,
        //   photonVisionSubsystem,
        //   0 // No delay
      )
    );
  }
}
