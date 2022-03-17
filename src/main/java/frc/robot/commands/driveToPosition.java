// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class driveToPosition extends CommandBase {
  /** Creates a new driveToPosition. */
  

  private String trajectoryName;

  private DrivetrainSubsystem drivetrainSubsystem;
  
  private final Timer timer = new Timer();

  private Pose2d dPose2d;

  private PIDController xPosPidController, yPosPidController, thetaPidController;

  public driveToPosition(DrivetrainSubsystem drivetrainSubsystem, Pose2d dPose2d) {
    
    this.drivetrainSubsystem = drivetrainSubsystem;

    this.dPose2d = dPose2d;
    

    addRequirements(drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xPosPidController = new PIDController(3, 0, 0);
    yPosPidController = new PIDController(3, 0, 0);
    thetaPidController = new PIDController(5, 0, 0);

    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
