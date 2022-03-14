// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberSubsystem extends SubsystemBase {

  private TalonFX climberTalonTop, climberTalonBottom;
  private Servo lockingServo;

  private Timer timer;
  private boolean engageBol = true;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    
    timer = new Timer();
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

    

    lockingServo = new Servo(3);

    timer.reset();


  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(engageBol){

      lockingServo.setAngle(180);

    } else {

      lockingServo.setAngle(165.5);
    }

    SmartDashboard.putNumber("climber Position", climberTalonTop.getSelectedSensorPosition());

    SmartDashboard.putNumber("climber  current", getClimberCurrent());

  }

  public void lockClimber(){

    engageBol = false;

  }

  public void disengageClimber(){

    engageBol = true;

  }

  public void disableClimber(){

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

    if(engageBol || timer.get() < .05){

      timer.start();
      disengageClimber();
      runDown();

    } else {

      setPosition(Constants.CLIMBER_POSITION_UP);

    }

    

  }

  public void setClimberDown(){

    setPosition(Constants.CLIMBER_POSITION_DOWN);

  }

  public double getClimberCurrent(){

    return (climberTalonTop.getSupplyCurrent() + climberTalonBottom.getSupplyCurrent()) / 2;

  }

  public boolean isOverZeroLimitCurrentLimit(){

    if(getClimberCurrent() > Constants.CLIMBER_ZERO_THRESHOLD){

      return true;

    } else {

      return false;

    }

  }


  public void zeroEncoder(){

    climberTalonTop.setSelectedSensorPosition(0);

  }

  

  public static boolean climberDrumIsRotating(){

    return false;

  }


}
