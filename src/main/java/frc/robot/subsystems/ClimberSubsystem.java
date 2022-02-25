// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberSubsystem extends SubsystemBase {

  private TalonFX climberTalonTop, climberTalonBottom;
  private static Servo lockingServo;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    climberTalonTop = new TalonFX(Constants.CLIMBER_TOP_TALON_CAN_ID);
    climberTalonBottom = new TalonFX(Constants.CLIMBER_BOTTOM_TALON_CAN_ID);

    climberTalonTop.configFactoryDefault();
    climberTalonBottom.configFactoryDefault();

    climberTalonTop.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kTimeoutMs);

    climberTalonBottom.follow(climberTalonTop);

    climberTalonTop.configAllowableClosedloopError(0, 0, Constants.kTimeoutMs);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		climberTalonTop.config_kF(0, 0, Constants.kTimeoutMs);
		climberTalonTop.config_kP(0, .1, Constants.kTimeoutMs);
		climberTalonTop.config_kI(0, 0, Constants.kTimeoutMs);
		climberTalonTop.config_kD(0, 0, Constants.kTimeoutMs);

    climberTalonTop.setSelectedSensorPosition(0);

    

    lockingServo = new Servo(Constants.CLIMBER_SERVO_PWM_CHANNEL);
    //lockingServo.setPosition(Constants.CLIMBER_SERVO_LOCK_POSITION);
    lockingServo.setDisabled();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("climber Position", climberTalonTop.getSelectedSensorPosition());

  }

  public static void lockClimber(){

    lockingServo.setAngle(Constants.CLIMBER_SERVO_LOCK_POSITION);

  }

  public static void disengageClimber(){

    lockingServo.setAngle(Constants.CLIMBER_SERVO_DISENGAGE_POSITION);

  }

  public static void disableClimber(){

    lockingServo.setDisabled();

  }

  public void setPosition(double position){

    climberTalonTop.set(ControlMode.Position, position);

  }

  public void runUp(){

    climberTalonTop.set(ControlMode.PercentOutput, .5);

  }

  public void runDown(){

    climberTalonTop.set(ControlMode.PercentOutput, -.5);

  }

  public void stop(){

    climberTalonTop.set(ControlMode.PercentOutput, 0);
    
  }

  public void setClimberUp(){

    setPosition(Constants.CLIMBER_POSITION_UP);

  }

  public void setClimberDown(){

    setPosition(Constants.CLIMBER_POSITION_DOWN);

  }

  public double getClimberCurrent(){

    return (climberTalonTop.getSupplyCurrent() + climberTalonBottom.getSupplyCurrent()) / 2;
  }

  public static boolean climberDrumIsRotating(){

    return false;

  }


}
