package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

/** Trajectory following */
public class FollowPath extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DrivetrainSubsystem m_DrivetrainSubsystem;
  private PathPlannerTrajectory trajectory;

  private String trajectoryName;

  private PIDController xPosPidController, yPosPidController, thetaPidController;

  private final Timer timer = new Timer();

  public FollowPath(DrivetrainSubsystem m_DrivetrainSubsystem, String trajectoryName) {
    
    this.m_DrivetrainSubsystem = m_DrivetrainSubsystem;
    this.trajectoryName = trajectoryName;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DrivetrainSubsystem);
  }

  @Override
  public void initialize() {
    trajectory = PathPlanner.loadPath(trajectoryName, 8, 2); // todo change max velocity and acceleration

    


    xPosPidController = new PIDController(3, 0, 0);
    yPosPidController = new PIDController(3, 0, 0);
    thetaPidController = new PIDController(.05, 0, 0);


    thetaPidController.enableContinuousInput(-Math.PI, Math.PI);

    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {

    PathPlannerState desiredState = (PathPlannerState) trajectory.sample(timer.get());

  
    

    ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(

          xPosPidController.calculate(m_DrivetrainSubsystem.getOdomPose2d().getX(), desiredState.poseMeters.getX()),
          yPosPidController.calculate(m_DrivetrainSubsystem.getOdomPose2d().getY(), desiredState.poseMeters.getY()),
          thetaPidController.calculate(-m_DrivetrainSubsystem.getOdomPose2d().getRotation().getRadians(), desiredState.poseMeters.getRotation().getRadians()), 
          GyroSubsystem.getBestRotation2d()
          
          );


        
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