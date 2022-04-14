// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LedCommand extends CommandBase {
  /** Creates a new LEDcommand. */

  public LedSubsystem ledSubsystem;
  public ShooterSubsystem shooterSubsystem;
  public IndexerSubsystem indexerSubsystem;
  public PhotonVisionSubsystem photonVisionSubsystem;

  public LedCommand(LedSubsystem ledSubsystem, ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem,
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
    ledSubsystem.solid(255, 225, 53);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (DriverStation.isAutonomousEnabled()) {

      ledSubsystem.auto();

    } else {

      ledSubsystem.tripleOrbit(200, 200, 0, 150, 150, 0, 2);

    }

    if (ShooterSubsystem.isDisabled == true) {

      if (IndexerSubsystem.bottomBeamBreak.get() == true && indexerSubsystem.topBeamBreak.get() == true) {

        ledSubsystem.tripleOrbit(200, 200, 0, 100, 100, 0, 2);

      } else if (IndexerSubsystem.bottomBeamBreak.get() == false && indexerSubsystem.topBeamBreak.get() == false) {

        if (DriverStation.getAlliance() == Alliance.Blue) {

          ledSubsystem.tripleOrbit(0, 0, 200, 0, 0, 0, 2);

        } else {

          ledSubsystem.tripleOrbit(200, 0, 0, 0, 0, 0, 2);

        }

      } else {

        ledSubsystem.tripleOrbit(200, 200, 0, 0, 0, 0, 2);

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