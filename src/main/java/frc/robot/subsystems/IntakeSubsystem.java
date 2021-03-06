// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  
  public static TalonFX m_intakeRunner;
  public static CANSparkMax m_actuatorLeft, m_actuatorRight;
  public static SparkMaxPIDController m_actuatorLeft_PidController, m_actuatorRight_PidController;
  public static RelativeEncoder m_actuatorLeft_encoder, m_actuatorRight_encoder;
  public static double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public static String intakeState = "NotZeroed";

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
    
  //  SmartDashboard.putString("Intake Status", "!!Not Zeroed!!");
  //  SmartDashboard.putNumber("intake Current draw", getIntakeActuatorCurrent());

  }

  @Override
  public void periodic() {

   // SmartDashboard.putNumber("intake Current draw", getIntakeActuatorCurrent());
//
   // SmartDashboard.putNumber("intake rotations position", m_actuatorLeft_encoder.getPosition());
   // 
   // SmartDashboard.putNumber("intake rotations velo", m_actuatorLeft_encoder.getVelocity());
//
   // SmartDashboard.putBoolean("is current bol treu", isIntakeStalledCurrent());
//
   // SmartDashboard.putBoolean("is velociuty basically zero", isIntakeVelocityBasicallyZero());
   // SmartDashboard.putString("intake STAtes", intakeState);

    if(Math.abs(m_actuatorRight_encoder.getPosition()- Constants.INTAKE_RIGHT_up) < 1){
      intakeState = "Up";
    } else if(Math.abs(m_actuatorRight_encoder.getPosition()- Constants.INTAKE_RIGHT_down) < 1){
      intakeState = "Down";
    } else {
      intakeState = "iiii or not zeroed";
    }

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

  private static void stopIntakeRunner(){

    m_intakeRunner.set(ControlMode.PercentOutput, Constants.INTAKE_RUNNER_SPEED_OFF);

    
  }

  public void intakeRest(){

    m_intakeRunner.set(ControlMode.PercentOutput, Constants.INTAKE_RUNNER_SPEED_OFF);
    

  }

  public static void runIntakeRunner(){

    m_intakeRunner.set(ControlMode.PercentOutput, Constants.INTAKE_RUNNER_SPEED_ON);

    
  }


  public void setIntakeUp(){

    if(isZeroed){

      m_actuatorLeft_PidController.setReference(Constants.Intake_LEFT_up, CANSparkMax.ControlType.kPosition);
      m_actuatorRight_PidController.setReference(Constants.INTAKE_RIGHT_up, CANSparkMax.ControlType.kPosition);

      
  
      } else {
      
        SmartDashboard.putString("Intake Status", "!!Not Zeroed!!");
  
      }
 
  }

  public void setintakeDown(){

    if(isZeroed){

    m_actuatorLeft_PidController.setReference(Constants.INTAKE_LEFT_down, CANSparkMax.ControlType.kPosition);
    m_actuatorRight_PidController.setReference(Constants.INTAKE_RIGHT_down, CANSparkMax.ControlType.kPosition);

    

    } else {
    
      SmartDashboard.putString("Intake Status", "!!Not Zeroed!!");

    }

  }

  public void zeroRotations(){

    m_actuatorRight_encoder.setPosition(-4);
    m_actuatorLeft_encoder.setPosition(4);

    isZeroed = true;

    SmartDashboard.putString("Intake Status", "!!Zeroing!!");

  }

  public static void runIntakeUp(double cycle){

    m_actuatorLeft_PidController.setReference(cycle, CANSparkMax.ControlType.kDutyCycle);
    m_actuatorRight_PidController.setReference(-cycle, CANSparkMax.ControlType.kDutyCycle);

    lastPositionLeft = m_actuatorLeft_encoder.getPosition();
    lastPositionRight = m_actuatorRight_encoder.getPosition();

    SmartDashboard.putString("Intake Status", "!!DutyCycle OverRide!!");

  }

  public void runIntakeDown(double cycle){

    m_actuatorLeft_PidController.setReference(-cycle, CANSparkMax.ControlType.kDutyCycle);
    m_actuatorRight_PidController.setReference(cycle, CANSparkMax.ControlType.kDutyCycle);


    lastPositionLeft = m_actuatorLeft_encoder.getPosition();
    lastPositionRight = m_actuatorRight_encoder.getPosition();

    SmartDashboard.putString("Intake Status", "!!DutyCycle OverRide!!");

  }

  public static void holdIntakePosition(){

    m_actuatorLeft_PidController.setReference(lastPositionLeft, CANSparkMax.ControlType.kPosition);
    m_actuatorRight_PidController.setReference(lastPositionRight, CANSparkMax.ControlType.kPosition);

    SmartDashboard.putString("Intake Status", "Holding Position");

  }

  public static double getIntakeActuatorAverageVelocity(){

    return (getIntakeLeftActuatorVelocity() + getIntakeRightActuatorVelocity())/2;

  }

  public static double getIntakeLeftActuatorVelocity(){

    return m_actuatorLeft_encoder.getVelocity();

  }

  public static double getIntakeRightActuatorVelocity(){

    return m_actuatorRight_encoder.getVelocity();
    
  }

  public static boolean isIntakeVelocityBasicallyZero(){

    if(Math.abs(getIntakeLeftActuatorVelocity()) < Constants.INTAKE_ZERO_VELOCITY_THRESHOLD_UB){
      
      return true;

    } else {

      return false;

    }

  }

  public boolean isIntakeStalledCurrent(){

    if(Math.abs(getIntakeActuatorCurrent()) > (Constants.INTAKE_CURRENT_LIMIT-1)){
      
      return true;

    } else { 
      
      return false;
    
    }


  }

  public static double getIntakeActuatorCurrent(){

    return m_actuatorLeft.getOutputCurrent() + m_actuatorRight.getOutputCurrent();

  }

  
  public String getIntakeState(){

    return intakeState;
    
  }


}
