// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  
  public static TalonFX m_intakeRunner;
  private static CANSparkMax m_actuatorLeft, m_actuatorRight;
  private static SparkMaxPIDController m_actuatorLeft_PidController, m_actuatorRight_PidController;
  private static RelativeEncoder m_actuatorLeft_encoder, m_actuatorRight_encoder;
  public static double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public IntakeSubsystem() {

    m_actuatorLeft = new CANSparkMax(Constants.LEFT_INTAKE_ACTUATOR_CAN_ID, MotorType.kBrushless);
    m_actuatorLeft.restoreFactoryDefaults();
    m_actuatorLeft.setIdleMode(IdleMode.kCoast);
    m_actuatorLeft_PidController = m_actuatorLeft.getPIDController();
    m_actuatorLeft_encoder = m_actuatorLeft.getEncoder();
    m_actuatorLeft_PidController.setI(Constants.INTAKE_LEFT_kI);
    m_actuatorLeft_PidController.setP(Constants.INTAKE_LEFT_kP);
    m_actuatorLeft_PidController.setIZone(Constants.INTAKE_LEFT_kIz);
    m_actuatorLeft_PidController.setFF(Constants.INTAKE_LEFT_kFF);
    m_actuatorLeft_PidController.setOutputRange(Constants.INTAKE_LEFT_kMinOutput, Constants.INTAKE_LEFT_kMaxOutput);
    m_actuatorLeft_PidController.setD(Constants.INTAKE_LEFT_kD);
    
    m_actuatorRight = new CANSparkMax(Constants.RIGHT_INTAKE_ACTUATOR_CAN_ID, MotorType.kBrushless);
    m_actuatorRight.restoreFactoryDefaults();
    m_actuatorRight.setIdleMode(IdleMode.kCoast);
    m_actuatorRight_PidController.setI(Constants.INTAKE_RIGHT_kI);
    m_actuatorRight_PidController.setP(Constants.INTAKE_RIGHT_kP);
    m_actuatorRight_PidController.setIZone(Constants.INTAKE_RIGHT_kIz);
    m_actuatorRight_PidController.setFF(Constants.INTAKE_RIGHT_kFF);
    m_actuatorRight_PidController.setOutputRange(Constants.INTAKE_RIGHT_kMinOutput, Constants.INTAKE_RIGHT_kMaxOutput);
    m_actuatorRight_PidController.setD(Constants.INTAKE_RIGHT_kD);

    m_intakeRunner = new TalonFX(43);


    



    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    


    
        
    SmartDashboard.putNumber("Indexer SetPoint", (-(RobotContainer.m_controller.getLeftTriggerAxis() - RobotContainer.m_controller.getRightTriggerAxis())));
    SmartDashboard.putNumber("Indexer ProcessVariable", m_actuatorRight_encoder.getPosition());
  }


  public static void runIntake(){

    m_intakeRunner.set(ControlMode.PercentOutput, .5);

  }

  public static void stopIntake(){

    m_intakeRunner.set(ControlMode.PercentOutput, 0);

  }

  public static void setIntakeUp(){

    m_actuatorLeft_PidController.setReference(-Constants.Intake_LEFT_up, CANSparkMax.ControlType.kDutyCycle);
    m_actuatorRight_PidController.setReference(Constants.Intake_LEFT_up, CANSparkMax.ControlType.kDutyCycle);
 
  }

  public static void setintakeDown(){

    m_actuatorLeft_PidController.setReference(-Constants.INTAKE_LEFT_down, CANSparkMax.ControlType.kDutyCycle);
    m_actuatorRight_PidController.setReference(Constants.INTAKE_LEFT_down, CANSparkMax.ControlType.kDutyCycle);

  }

  



}
