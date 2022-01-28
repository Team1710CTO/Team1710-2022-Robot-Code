// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistribution;


public class hoodSubsystem extends SubsystemBase {
  /** Creates a new hoodSubsystem. */
  private static hoodSubsystem instance = null;

  private CANSparkMax m_hood;
  private SparkMaxPIDController m_hoodPidController;
  private RelativeEncoder m_hoodEncoder;
  public static double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, rotAcctual;

  public hoodSubsystem() {
    m_hood = new CANSparkMax(Constants.HOOD_CAN_ID, MotorType.kBrushless);
    m_hood.restoreFactoryDefaults();

    m_hoodPidController = m_hood.getPIDController();

    m_hoodEncoder = m_hood.getEncoder();

    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    m_hoodPidController.setP(kP);
    m_hoodPidController.setI(kI);
    m_hoodPidController.setD(kD);
    m_hoodPidController.setIZone(kIz);
    m_hoodPidController.setFF(kFF);
    m_hoodPidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    
    SmartDashboard.putNumber("Hood P Gain", kP);
    SmartDashboard.putNumber("Hood I Gain", kI);
    SmartDashboard.putNumber("Hood D Gain", kD);
    SmartDashboard.putNumber("Hood I Zone", kIz);
    SmartDashboard.putNumber("Hood Feed Forward", kFF);
    SmartDashboard.putNumber("Hood Max Output", kMaxOutput);
    SmartDashboard.putNumber("Hood Min Output", kMinOutput);
    SmartDashboard.putNumber("Hood Set Rotations", 0);

    rotAcctual = m_hoodEncoder.getPosition();
    m_hoodPidController.setReference(rotAcctual, CANSparkMax.ControlType.kPosition);

  }

  public static hoodSubsystem getInstance() {
		if(instance == null)
			instance = new hoodSubsystem();

		return instance;	
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_hoodPidController.setP(p); kP = p; }
    if((i != kI)) { m_hoodPidController.setI(i); kI = i; }
    if((d != kD)) { m_hoodPidController.setD(d); kD = d; }
    if((iz != kIz)) { m_hoodPidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_hoodPidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_hoodPidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
      }
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", m_hoodEncoder.getPosition());
  }

  public void setHoodAngle(double pos){
    m_hoodPidController.setReference(pos, CANSparkMax.ControlType.kPosition);
  }

  public void zeroHood(){
    
  }

}