package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class ZeroOdomAuto extends CommandBase {

  DrivetrainSubsystem drivetrainSubsystem;
  GyroSubsystem gyroSubsystem;

  /** Creates a new ZeroOdom. */
  public ZeroOdomAuto(
    GyroSubsystem gyroSubsystem,
    DrivetrainSubsystem drivetrainSubsystem
  ) {
    this.gyroSubsystem = gyroSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(gyroSubsystem, drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Zero our gyro on match start.
    gyroSubsystem.zeroBestGyro();
    gyroSubsystem.setIsZeroingFalse();
    // set gyro offset to starting field angle (91.5 +- 3.0 degrees)
    // gyroSubsystem.setGyro(91.5); // May not need? Offset accounted in odo

    // Set our starting odometry position to the starting location on the field.
    drivetrainSubsystem.setOdometryAuto(
      new Pose2d(new Translation2d(7.65, 1.85), Rotation2d.fromDegrees(91.5)),
      gyroSubsystem.getNavXRotationAuto()
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
