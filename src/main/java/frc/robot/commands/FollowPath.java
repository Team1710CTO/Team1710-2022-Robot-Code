package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Trajectory following */
public class FollowPath extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DrivetrainSubsystem m_DrivetrainSubsystem;
  private PathPlannerTrajectory trajectory;

  private String trajectoryName;

  private HolonomicDriveController controller;
  private final Timer timer = new Timer();

  public FollowPath(DrivetrainSubsystem m_DrivetrainSubsystem, String trajectoryName) {
    
    this.m_DrivetrainSubsystem = m_DrivetrainSubsystem;
    this.trajectoryName = trajectoryName;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DrivetrainSubsystem);
  }

  @Override
  public void initialize() {
    trajectory = PathPlanner.loadPath(trajectoryName, 0.8, 1); // todo change max velocity and acceleration

    ProfiledPIDController thetaController = new ProfiledPIDController(6, 0, 0,
        new TrapezoidProfile.Constraints(Math.PI, Math.PI));


    PIDController xPosPidController = new PIDController(6, 0, 0);
    PIDController yPosPidController = new PIDController(6, 0, 0);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    controller = new HolonomicDriveController(xPosPidController, yPosPidController, thetaController);

    m_DrivetrainSubsystem.resetOdometry();

    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {

    PathPlannerState desiredState = (PathPlannerState) trajectory.sample(timer.get());

    ChassisSpeeds targetChassisSpeeds = controller.calculate(m_DrivetrainSubsystem.getOdomPose2d(), desiredState,
        desiredState.poseMeters.getRotation());

        
    m_DrivetrainSubsystem.drive(targetChassisSpeeds);

  }

  @Override
  public void end(boolean interrupted) {

    m_DrivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
    timer.stop();

  }

  @Override
  public boolean isFinished() {

    return timer.hasElapsed(trajectory.getTotalTimeSeconds());

  }
}