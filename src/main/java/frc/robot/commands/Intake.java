// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
 
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Intake extends CommandBase {

  public IntakeSubsystem intakeSubsystem;

  public DrivetrainSubsystem drivetrainSubsystem;

  

  public PIDController rotationPidController, movePidController;

  /** Creates a new IntakeDown. */
  public Intake(IntakeSubsystem intakeSubsystem, DrivetrainSubsystem drivetrainSubsystem) {

    this.intakeSubsystem = intakeSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;

    addRequirements(intakeSubsystem, drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationPidController = new PIDController(.05, .0005, 0); //TODO

    movePidController = new PIDController(.2, 0, 0); //TODO
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // indexerSubsystem.runIndexerIn();
        intakeSubsystem.setintakeDown();
        intakeSubsystem.runIntake();


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    intakeSubsystem.setIntakeUp();
    intakeSubsystem.intakeRest();
    // indexerSubsystem.stopIndexer();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
