package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class ZeroHoodAuto extends CommandBase {

  /** Creates a new ZeroHood. */
  public HoodSubsystem hoodSubsystem;
  public final Timer timer = new Timer();

  public ZeroHoodAuto(HoodSubsystem hoodSubsystem) {
    this.hoodSubsystem = hoodSubsystem;
    addRequirements(hoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    //hoodSubsystem.disableSoftLimits();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hoodSubsystem.runHoodDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.zeroHood();
    hoodSubsystem.setHoodPosition(.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (hoodSubsystem.isHoodCurrentOverZeroConstant() && timer.get() > .25) {
      return true;
    } else {
      return false;
    }
  }
}
