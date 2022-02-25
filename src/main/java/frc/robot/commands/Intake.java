// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Intake extends CommandBase {

  public static IntakeSubsystem intakeSubsystem;

  public static DrivetrainSubsystem drivetrainSubsystem;

  public static PhotonVisionSubsystem photonVisionSubsystem;

  public static PIDController rotationPidController, movePidController;

  /** Creates a new IntakeDown. */
  public Intake(IntakeSubsystem intakeSubsystem, DrivetrainSubsystem drivetrainSubsystem,
      PhotonVisionSubsystem photonVisionSubsystem) {

    this.intakeSubsystem = intakeSubsystem;
    this.photonVisionSubsystem = photonVisionSubsystem;
    this.drivetrainSubsystem = drivetrainSubsystem;

    addRequirements(intakeSubsystem, drivetrainSubsystem, photonVisionSubsystem);
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
    if (PhotonVisionSubsystem.doesIntakeSeeBall()) {
      boolean consume;
      if (PhotonVisionSubsystem.getYDisplacementOfBall() < 0) { //CHANGE THE DISTANCE
        intakeSubsystem.setintakeDown();
        //intakeSubsystem.runIntake();
        consume = true;
      } else {
        intakeSubsystem.setIntakeUp();
        //intakeSubsystem.stopIntakeRunner();
        drivetrainSubsystem.drive(new ChassisSpeeds(
            -movePidController.calculate(PhotonVisionSubsystem.getYDisplacementOfBall()),
            0,
            -rotationPidController.calculate(photonVisionSubsystem.getXDisplacementOfBall())));
            consume = false;
      }
      SmartDashboard.putBoolean("CONSUME", consume);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    intakeSubsystem.setIntakeUp();
    intakeSubsystem.stopIntakeRunner();
    // indexerSubsystem.stopIndexer();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
