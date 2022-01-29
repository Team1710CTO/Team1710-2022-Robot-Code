// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private static IntakeSubsystem instance = null;

  public IntakeSubsystem() {

    

  }

  public static IntakeSubsystem getInstance() {
		if(instance == null)
			instance = new IntakeSubsystem();

		return instance;	
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
