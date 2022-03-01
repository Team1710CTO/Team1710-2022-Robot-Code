package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Trajectory following */
public class autonomousCommand extends CommandBase {

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DrivetrainSubsystem m_subsystem;
  private PathPlannerTrajectory trajectory;
  private HolonomicDriveController controller;
  private final Timer timer = new Timer();

  public autonomousCommand(DrivetrainSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    trajectory = PathPlanner.loadPath("Test", 1, 2);

    ProfiledPIDController thetaController =  new ProfiledPIDController(1.2, 0.1, 0.1,
        new TrapezoidProfile.Constraints(Math.PI, Math.PI));
    PIDController xPID = new PIDController(0.1, 0.01, 0);
    PIDController yPID = new PIDController(-0.1, 0.01, 0);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    controller = new HolonomicDriveController(xPID, yPID, thetaController);

    m_subsystem.resetOdometry();

    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    PathPlannerState desiredState = (PathPlannerState) trajectory.sample(timer.get());

    ChassisSpeeds targetChassisSpeeds = controller.calculate(m_subsystem.getPose(), desiredState,
        desiredState.holonomicRotation);

    m_subsystem.drive(targetChassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(new ChassisSpeeds(0, 0, 0));
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}