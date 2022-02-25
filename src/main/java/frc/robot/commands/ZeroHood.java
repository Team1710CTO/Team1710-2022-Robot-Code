// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class ZeroHood extends CommandBase {
  /** Creates a new ZeroHood. */

  public HoodSubsystem hoodSubsystem;

  public ZeroHood(HoodSubsystem hoodSubsystem) {

    this.hoodSubsystem = hoodSubsystem;

    addRequirements(hoodSubsystem);
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //hoodSubsystem.disableSoftLimits();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    hoodSubsystem.runHoodDown();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(hoodSubsystem.isHoodCurrentOverZeroConstant()){

        hoodSubsystem.zeroHood();
        //hoodSubsystem.setSoftLimits();

        hoodSubsystem.setHoodPosition(.5);
        return true;

    } else {

        return false;

    }
    
  }

}
