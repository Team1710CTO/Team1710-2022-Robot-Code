// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase {
  /** Creates a new Shoot. */

  public static HoodSubsystem hoodSubsystem;
  public static ShooterSubsystem shooterSubsystem;
  public Shoot(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem) {

    this.hoodSubsystem = hoodSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(hoodSubsystem, shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooterSubsystem.setSpeed(3000);

  



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    shooterSubsystem.disableShooter();

  

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}