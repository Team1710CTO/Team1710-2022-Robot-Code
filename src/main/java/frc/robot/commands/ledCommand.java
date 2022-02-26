// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ledSubsystem;


public class ledCommand extends CommandBase {
  /** Creates a new ledCommand. */

  public ledSubsystem ledSubsystem; 

  public ledCommand(ledSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    ledSubsystem = subsystem; 
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   ledSubsystem.setLength();
   ledSubsystem.solid(200, 80, 100);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //ledSubsystem.orbit(255, 255, 0);
    //ledSubsystem.knightRider(255, 255, 0);
    //ledSubsystem.alt();
    //ledSubsystem.rainbow();
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

