package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ZeroIntakeAuto extends CommandBase {

  /** Creates a new ZeroIntake. */

  public IntakeSubsystem intakeSubsystem;

  public final Timer timer = new Timer();

  public ZeroIntakeAuto(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Lower intake system
    intakeSubsystem.runIntakeDown(-.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Zero when the command is finished (isFinished condition fulfilled below)
    intakeSubsystem.zeroRotations();

    // Stop running intake down (now intake is lowered.)
    intakeSubsystem.runIntakeDown(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Wait until intake is stalled to be considered 'down' then zero,
    // else zero after .5 seconds (took 15 frames at 30fps) to settle as down.
    return (
      (intakeSubsystem.isIntakeStalledCurrent() && timer.get() > .25) ||
      timer.get() > .5 // Fail-safe
    );
  }
}
