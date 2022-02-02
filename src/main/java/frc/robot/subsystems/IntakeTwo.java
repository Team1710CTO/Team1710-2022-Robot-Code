// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeTwo extends SubsystemBase {
  /** Creates a new Intake. */
  private static final int deviceID = 30;
    private static CANSparkMax m_motor;
    private static SparkMaxPIDController m_pidController;
    private static RelativeEncoder m_encoder;
    public static double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public IntakeTwo() {

    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
        
        m_motor.setIdleMode(IdleMode.kBrake);

        /**
         * The restoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        m_motor.restoreFactoryDefaults();
    
        /**
         * In order to use PID functionality for a controller, a SparkMaxPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_pidController = m_motor.getPIDController();
    
        // Encoder object created to display position values
        m_encoder = m_motor.getEncoder();
    
        // PID coefficients
        kP = 0.7; 
        kI = 5e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;
    
    
        //testing git
        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    
        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("Indexer P Gain", kP);
        SmartDashboard.putNumber("Indexer I Gain", kI);
        SmartDashboard.putNumber("Indexer D Gain", kD);
        SmartDashboard.putNumber("Indexer I Zone", kIz);
        SmartDashboard.putNumber("Indexer Feed Forward", kFF);
        SmartDashboard.putNumber("Indexer Max Output", kMaxOutput);
        SmartDashboard.putNumber("Indexer Min Output", kMinOutput);
        SmartDashboard.putNumber("Indexer Set Rotations", 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double p = SmartDashboard.getNumber("Indexer P Gain", 0);
        double i = SmartDashboard.getNumber("Indexer I Gain", 0);
        double d = SmartDashboard.getNumber("Indexer D Gain", 0);
        double iz = SmartDashboard.getNumber("Indexer I Zone", 0);
        double ff = SmartDashboard.getNumber("Indexer Feed Forward", 0);
        double max = SmartDashboard.getNumber("Indexer Max Output", 0);
        double min = SmartDashboard.getNumber("Indexer Min Output", 0);
        double rotations = SmartDashboard.getNumber("Indexer Set Rotations", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { m_pidController.setP(p); kP = p; }
        if((i != kI)) { m_pidController.setI(i); kI = i; }
        if((d != kD)) { m_pidController.setD(d); kD = d; }
        if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
        m_pidController.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
        }

        /**
         * PIDController objects are commanded to a set point using the 
         * SetReference() method.
         * 
         * The first parameter is the value of the set point, whose units vary
         * depending on the control type set in the second parameter.
         * 
         * The second parameter is the control type can be set to one of four 
         * parameters:
         *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
         *  com.revrobotics.CANSparkMax.ControlType.kPosition
         *  com.revrobotics.CANSparkMax.ControlType.kVelocity
         *  com.revrobotics.CANSparkMax.ControlType.kVoltage
         */
        
        
        
        m_pidController.setReference(-(RobotContainer.m_controller.getRightTriggerAxis() - RobotContainer.m_controller.getRightTriggerAxis()), CANSparkMax.ControlType.kDutyCycle);
      
        
        
        SmartDashboard.putNumber("Indexer SetPoint", rotations);
        SmartDashboard.putNumber("Indexer ProcessVariable", m_encoder.getPosition());

  }
}
