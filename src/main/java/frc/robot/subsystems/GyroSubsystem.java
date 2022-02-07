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
  //private final static PigeonIMU m_leftPigeon = new PigeonIMU(Constants.LEFT_PIGEON_ID);
  private final static AHRS m_navx = new AHRS(); 

  public static boolean isZeroing = false;

  public GyroSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static Rotation2d getBestRotation2d(){ //TODO

    return getNavXGyroscopeRotation();

  }

  public static Rotation2d getBestRotationInDegrees(){ //TODO

    return getNavXGyroscopeRotation();

  }

  public void zeroBestGyro(){
    
    m_navx.zeroYaw();

    isZeroing = true;


  }


  public static Rotation2d getRightPigeonGyroscopeRotation() {

    return Rotation2d.fromDegrees(-m_rightPigeon.getFusedHeading());

  }

  public void zeroRightPigeonGyroscope() { // keep non static!

    m_rightPigeon.setFusedHeading(0.0);

  }

  public static Rotation2d getLeftPigeonGyroscopeRotation() {

    //Rotation2d.fromDegrees(-m_leftPigeon.getFusedHeading());

    return Rotation2d.fromDegrees(0.0); //fixme

  }

  public void zeroLeftPigeonGyroscope() { // keep non static!

    //m_leftPigeon.setFusedHeading(0.0);

  }

  public static Rotation2d getNavXGyroscopeRotation() {
    
    if (m_navx.isMagnetometerCalibrated()) {
    // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }

    return Rotation2d.fromDegrees(-(360.0 - m_navx.getYaw()));
  }

  public void zeroNavXGyroscope() { // keep non static!
    
    m_navx.zeroYaw();

  }
  
  public void setIsZeroingFalse(){
    
    isZeroing = false;

  }

}
