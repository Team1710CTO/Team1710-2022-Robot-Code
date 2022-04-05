// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ZeroClimber extends CommandBase {
  /** Creates a new ZeroClimber. */

  public final Timer timer = new Timer();
  
  public ClimberSubsystem climberSubsystem;
  public ZeroClimber(ClimberSubsystem climberSubsystem) {


    this.climberSubsystem = climberSubsystem;

    addRequirements(climberSubsystem);

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

    climberSubsystem.runDown();

    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
    climberSubsystem.zeroEncoder();
    
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climberSubsystem.isOverZeroLimitCurrentLimit() && timer.get() > .1;
  }
}
