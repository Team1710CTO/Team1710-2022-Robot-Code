// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ledSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import edu.wpi.first.wpilibj.DriverStation;

public class LEDcommand extends CommandBase {
  /** Creates a new LEDcommand. */

  public ledSubsystem ledSubsystem;
  public ShooterSubsystem shooterSubsystem;
  public IndexerSubsystem indexerSubsystem;
  public PhotonVisionSubsystem photonVisionSubsystem;

  public LEDcommand(ledSubsystem ledSubsystem, ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem,
      PhotonVisionSubsystem photonVisionSubsystem) {

    this.ledSubsystem = ledSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.photonVisionSubsystem = photonVisionSubsystem;

    addRequirements(ledSubsystem, shooterSubsystem, indexerSubsystem, photonVisionSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    ledSubsystem.setLength();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (DriverStation.isAutonomous()) {

      ledSubsystem.auto();
      
    } else if (DriverStation.isTeleop()) {

      if (ShooterSubsystem.isDisabled == false) {

        if (photonVisionSubsystem.getXDisplacementOfGoal() > -1 && photonVisionSubsystem.getXDisplacementOfGoal() < 1) {

          ledSubsystem.tripleOrbit(0, 200, 0, 0, 150, 0, 2);

          if (shooterSubsystem.isShooterToSpeedAndNotDisabled()) {

            ledSubsystem.tripleOrbit(0, 0, 200, 0, 0, 150, 2);

          }

        } else {

          ledSubsystem.tripleOrbit(200, 200, 0, 150, 150, 0, 2);

        }

      }

      if (IndexerSubsystem.bottomBeamBreak.get() == true && indexerSubsystem.topBeamBreak.get() == true) {

        ledSubsystem.tripleOrbit(100, 100, 100, 200, 100, 0, 2);

      } else if (IndexerSubsystem.bottomBeamBreak.get() == false && indexerSubsystem.topBeamBreak.get() == false) {

        ledSubsystem.tripleOrbit(200, 200, 200, 0, 0, 0, 2);

      } else {

        ledSubsystem.tripleOrbit(200, 100, 0, 0, 0, 0, 2);

      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
