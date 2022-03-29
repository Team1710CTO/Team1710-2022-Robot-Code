package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInPath extends CommandBase {

  public IntakeSubsystem intakeSubsystem;
  public IndexerSubsystem indexerSubsystem;
  public int expectedBalls = 0;
  public Timer timer;
  public double ttl;

  /** Creates a new IntakeWithVision. */
  public IntakeInPath(
    IndexerSubsystem indexerSubsystem,
    IntakeSubsystem intakeSubsystem,
    int balls,
    double ttl
  ) {
    this.intakeSubsystem = intakeSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.expectedBalls = balls;
    this.ttl = ttl;

    addRequirements(intakeSubsystem, indexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setintakeDown();
    intakeSubsystem.runIntake();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Just periodic and index
    indexerSubsystem.indexBallsBetweenBreaks();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Raise pickup, and stop intake.
    intakeSubsystem.setIntakeUp();
    intakeSubsystem.intakeRest();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Once we have 2 balls or TTL expires
    return (
      indexerSubsystem.getBallCount() == expectedBalls || timer.get() > ttl
    );
  }
}
