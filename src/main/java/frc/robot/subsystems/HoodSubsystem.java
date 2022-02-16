// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.XboxController;

public class HoodSubsystem extends SubsystemBase {

  /** Creates a new ExampleSubsystem. */
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  double rotations;

  public HoodSubsystem() {
    // initialize motor
    m_motor = new CANSparkMax(Constants.HOOD_CAN_ID, MotorType.kBrushless);
    m_motor.setIdleMode(IdleMode.kBrake);

    /**
     * The restoreFactoryDefaults method can be used to reset the configuration
     * parameters
     * in the SPARK MAX to their factory default state. If no argument is passed,
     * these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();

    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController
     * object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_motor.getPIDController();

    // Encoder object created to display position values
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    kP = Constants.HOOD_kP;
    kI = Constants.HOOD_kI;
    kD = Constants.HOOD_kD;
    kIz = Constants.HOOD_kIz;
    kFF = Constants.HOOD_kFF;
    kMaxOutput = Constants.HOOD_kMaxOutput;
    kMinOutput = Constants.HOOD_kMinOutput;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Hood Setpoint", 0);

  }

  @Override
  public void periodic() {
    } 
  public void hoodAngle(){
    double rotations = SmartDashboard.getNumber("Hood Setpoint", 0);
    double position = m_encoder.getPosition();
    m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    SmartDashboard.putNumber("ProcessVariable", position);
  }
  
}