// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroCommand extends ParallelCommandGroup {

  public IntakeSubsystem intakeSubsystem;

  public DrivetrainSubsystem drivetrainSubsystem;

  public HoodSubsystem hoodSubsystem;

  public GyroSubsystem gyroSubsystem;

  /** Creates a new ZeroCommand. */
  public ZeroCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    IntakeSubsystem intakeSubsystem,
    IndexerSubsystem indexerSubsystem,
    HoodSubsystem hoodSubsystem,
    GyroSubsystem gyroSubsystem
  ) {
    this.gyroSubsystem = gyroSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.hoodSubsystem = hoodSubsystem;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ZeroOdom(gyroSubsystem, drivetrainSubsystem),
      new ZeroHood(hoodSubsystem),
      new ZeroIntake(intakeSubsystem)
    );
  }
}
