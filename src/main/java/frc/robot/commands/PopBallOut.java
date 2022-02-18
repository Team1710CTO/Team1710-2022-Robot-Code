// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PopBallOut extends CommandBase {
  /** Creates a new PopBallOut. */

  public static ShooterSubsystem shooterSubsystem;
  public static IndexerSubsystem indexerSubsystem;

  public static boolean jamCleared = false;

  public PopBallOut(IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem) {

    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(indexerSubsystem, shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    
    if(indexerSubsystem.isTopBeakBreakTripped()){


      jamCleared = false;

    } else {

      jamCleared = true;

    } 

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(indexerSubsystem.isTopBeakBreakTripped() && !jamCleared){

        shooterSubsystem.disableShooter();
        indexerSubsystem.runIndexerOut();

    } else {

        jamCleared = true;

        shooterSubsystem.setSpeed(300);

        if(shooterSubsystem.isShooterToSpeedAndNotDisabled()){

          indexerSubsystem.runIndexerIn();

        }

    } 

    

    
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    shooterSubsystem.disableShooter();
    indexerSubsystem.stopIndexer();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
