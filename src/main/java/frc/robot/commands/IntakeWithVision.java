// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class IntakeWithVision extends CommandBase {

  public IntakeSubsystem intakeSubsystem;

  public DrivetrainSubsystem drivetrainSubsystem;

  public PhotonVisionSubsystem photonVisionSubsystem;

  public IndexerSubsystem indexerSubsystem;

  public PIDController xPidController, yPidController;
  /** Creates a new IntakeWithVision. */
  public IntakeWithVision(IntakeSubsystem intakeSubsystem, DrivetrainSubsystem drivetrainSubsystem, PhotonVisionSubsystem photonVisionSubsystem, IndexerSubsystem indexerSubsystem) {

    this.intakeSubsystem = intakeSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.photonVisionSubsystem = photonVisionSubsystem;
    this.indexerSubsystem = indexerSubsystem;

    xPidController = new PIDController(.06, .01, 0);
    yPidController = new PIDController(.06, .01, 0);

    addRequirements(intakeSubsystem, drivetrainSubsystem, photonVisionSubsystem, indexerSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    intakeSubsystem.setIntakeUp();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xDisplacement = photonVisionSubsystem.getXDisplacementOfBall();
    double yDisplacement = photonVisionSubsystem.getYDisplacementOfBall();

    double vx = xPidController.calculate(xDisplacement, 0);
    double vy = yPidController.calculate(yDisplacement, 0);

    drivetrainSubsystem.drive(new ChassisSpeeds(-vy,vx,0));

    SmartDashboard.putNumber("poopoo", (xPidController.getPositionError() + yPidController.getPositionError()));

    if(Math.abs(xPidController.getPositionError()) < 8 && Math.abs(yPidController.getPositionError()) < 2){

      intakeSubsystem.setintakeDown();
      intakeSubsystem.runIntake();

    } else {

      intakeSubsystem.setIntakeUp();
      intakeSubsystem.intakeRest();

    }

    indexerSubsystem.indexBallsBetweenBreaks();


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    intakeSubsystem.setIntakeUp();
    intakeSubsystem.intakeRest();
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !indexerSubsystem.isBottomBeakBreakTripped();
  }
}
