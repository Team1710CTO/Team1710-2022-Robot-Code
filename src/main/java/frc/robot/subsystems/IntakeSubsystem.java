// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.io.Console;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  
  public static TalonFX m_intakeRunner;
  public static CANSparkMax m_actuatorLeft, m_actuatorRight;
  public static SparkMaxPIDController m_actuatorLeft_PidController, m_actuatorRight_PidController;
  public static RelativeEncoder m_actuatorLeft_encoder, m_actuatorRight_encoder;
  public static double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public static boolean isZeroed = false;

  public static double lastPositionLeft, lastPositionRight = 0;

  public IntakeSubsystem() {

    m_actuatorLeft = new CANSparkMax(Constants.LEFT_INTAKE_ACTUATOR_CAN_ID, MotorType.kBrushless);
    m_actuatorLeft.restoreFactoryDefaults();
    m_actuatorLeft.setIdleMode(IdleMode.kBrake);
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
    m_actuatorRight.setIdleMode(IdleMode.kBrake);
    m_actuatorRight_PidController = m_actuatorRight.getPIDController();
    m_actuatorRight_encoder = m_actuatorRight.getEncoder();
    m_actuatorRight_PidController.setI(Constants.INTAKE_RIGHT_kI);
    m_actuatorRight_PidController.setP(Constants.INTAKE_RIGHT_kP);
    m_actuatorRight_PidController.setIZone(Constants.INTAKE_RIGHT_kIz);
    m_actuatorRight_PidController.setFF(Constants.INTAKE_RIGHT_kFF);
    m_actuatorRight_PidController.setOutputRange(Constants.INTAKE_RIGHT_kMinOutput, Constants.INTAKE_RIGHT_kMaxOutput);
    m_actuatorRight_PidController.setD(Constants.INTAKE_RIGHT_kD);

    m_intakeRunner = new TalonFX(Constants.INTAKE_RUNNER_CAN_ID);
    
    SmartDashboard.putString("Intake Status", "!!Not Zeroed!!");
    SmartDashboard.putNumber("intake Current draw", PowerDistributionSubsystem.getintakeActuatorCurrent());

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("intake Current draw", PowerDistributionSubsystem.getintakeActuatorCurrent());

    SmartDashboard.putNumber("intake rotations position", m_actuatorLeft_encoder.getPosition());
    
  }



  public static void runIntake(){

    if(isZeroed){

      if(Math.abs(m_actuatorRight_encoder.getPosition()) > .1){

          runIntakeRunner();
    
        } else {
    
          stopIntakeRunner();
    
        }

    } else {

      stopIntakeRunner();

    }

  }

  public static void stopIntakeRunner(){

    m_intakeRunner.set(ControlMode.PercentOutput, Constants.INTAKE_RUNNER_SPEED_OFF);

    
  }

  public static void runIntakeRunner(){

    m_intakeRunner.set(ControlMode.PercentOutput, Constants.INTAKE_RUNNER_SPEED_ON);

    
  }


  public static void setIntakeUp(){

    if(isZeroed){

      m_actuatorLeft_PidController.setReference(Constants.Intake_LEFT_up, CANSparkMax.ControlType.kPosition);
      m_actuatorRight_PidController.setReference(Constants.INTAKE_RIGHT_up, CANSparkMax.ControlType.kPosition);

      SmartDashboard.putString("Intake Status", "Up");
  
      } else {
      
        SmartDashboard.putString("Intake Status", "!!Not Zeroed!!");
  
      }
 
  }

  public static void setintakeDown(){

    if(isZeroed){

    m_actuatorLeft_PidController.setReference(Constants.INTAKE_LEFT_down, CANSparkMax.ControlType.kPosition);
    m_actuatorRight_PidController.setReference(Constants.INTAKE_RIGHT_down, CANSparkMax.ControlType.kPosition);

    SmartDashboard.putString("Intake Status", "Down");

    } else {
    
      SmartDashboard.putString("Intake Status", "!!Not Zeroed!!");

    }

  }

  public static void zeroRotations(){

    m_actuatorRight_encoder.setPosition(0);
    m_actuatorLeft_encoder.setPosition(0);

    isZeroed = true;

    SmartDashboard.putString("Intake Status", "!!Zeroing!!");

  }

  public static void runIntakeUp(){

    m_actuatorLeft_PidController.setReference(1, CANSparkMax.ControlType.kDutyCycle);
    m_actuatorRight_PidController.setReference(-1, CANSparkMax.ControlType.kDutyCycle);

    lastPositionLeft = m_actuatorLeft_encoder.getPosition();
    lastPositionRight = m_actuatorRight_encoder.getPosition();

    SmartDashboard.putString("Intake Status", "!!Manual OverRide!!");

  }

  public static void runIntakeDown(){

    m_actuatorLeft_PidController.setReference(-1, CANSparkMax.ControlType.kDutyCycle);
    m_actuatorRight_PidController.setReference(1, CANSparkMax.ControlType.kDutyCycle);


    lastPositionLeft = m_actuatorLeft_encoder.getPosition();
    lastPositionRight = m_actuatorRight_encoder.getPosition();

    SmartDashboard.putString("Intake Status", "!!Manual OverRide!!");

  }

  public static void holdIntakePosition(){

    m_actuatorLeft_PidController.setReference(lastPositionLeft, CANSparkMax.ControlType.kPosition);
    m_actuatorRight_PidController.setReference(lastPositionRight, CANSparkMax.ControlType.kPosition);

    SmartDashboard.putString("Intake Status", "Holding Position");

  }


}
