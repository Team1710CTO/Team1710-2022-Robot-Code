// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DeadReckonDrive extends CommandBase {
  /** Creates a new DeadReckonDrive. */

  private DrivetrainSubsystem drivetrainSubsystem;

  public double vx,vy,vtheta,duration;

  public Timer timer;

  public DeadReckonDrive(DrivetrainSubsystem drivetrainSubsystem, double vx, double vy, double vtheta, double duration) {

    this.drivetrainSubsystem = drivetrainSubsystem;
    this.vx = vx;
    this.vy = vy;
    this.vtheta = vtheta;
    this.duration = duration;
    
    timer = new Timer();


    addRequirements(drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
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

    drivetrainSubsystem.drive(new ChassisSpeeds(vx, vy, vtheta));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > duration;
  }
}
