// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GyroSubsystem extends SubsystemBase {
  /** Creates a new GyroSubsystem. */
  private final static PigeonIMU m_rightPigeon = new PigeonIMU(Constants.RIGHT_PIGEON_ID);
  // FIXME Uncomment if you are using a NavX
  private final static AHRS m_navx = new AHRS(); // NavX connected over MXP

  public GyroSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static Rotation2d getRightPigeonGyroscopeRotation() {
    // FIXME Remove if you are using a Pigeon
    return Rotation2d.fromDegrees(-m_rightPigeon.getFusedHeading());

    // FIXME Uncomment if you are using a NavX
    //if (m_navx.isMagnetometerCalibrated()) {
//      // We will only get valid fused headings if the magnetometer is calibrated
      //return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    //}
//
//    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    //return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void zeroRightPigeonGyroscope() {
    // FIXME Remove if you are using a Pigeon
    m_rightPigeon.setFusedHeading(0.0);

    // FIXME Uncomment if you are using a NavX
    //m_navx.zeroYaw();
  }
  
}
