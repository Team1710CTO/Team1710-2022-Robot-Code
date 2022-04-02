// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {

  public static CANSparkMax m_hood_motor;
  public static SparkMaxPIDController m_hood_pidController;
  private static RelativeEncoder m_hood_encoder;

  private static boolean isZeroed = false;

  public double goalPos = 0;

  /** Creates a new Hood. */
  public HoodSubsystem() {

    isZeroed = false;

    m_hood_motor = new CANSparkMax(Constants.HOOD_CAN_ID, MotorType.kBrushless);

    m_hood_motor.restoreFactoryDefaults();

    m_hood_pidController = m_hood_motor.getPIDController();

    m_hood_encoder = m_hood_motor.getEncoder();



    m_hood_pidController.setP(Constants.HOOD_kP);
    m_hood_pidController.setI(Constants.HOOD_kI);
    m_hood_pidController.setD(Constants.HOOD_kD);
    m_hood_pidController.setIZone(Constants.HOOD_kIz);
    m_hood_pidController.setFF(Constants.HOOD_kFF);
    m_hood_pidController.setOutputRange(Constants.HOOD_kMinOutput, Constants.HOOD_kMaxOutput);

    // m_hood_pidController.setReference(Constants.HOOD_POSITION_MIN,
    // ControlType.kPosition);
    SmartDashboard.putString("Hood Status", "!!Not Zeroed!!");
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("hood pos", m_hood_encoder.getPosition());

  }

  public void setHoodPosition(double position) {

    if (isZeroed) {

      position = goalPos;

      m_hood_pidController.setReference(position, ControlType.kPosition);

    } else {

      SmartDashboard.putString("Hood Status", "Set to Pos");

    }

  }

  public static void setSoftLimits() {

    m_hood_motor.setSoftLimit(SoftLimitDirection.kForward, Constants.HOOD_POSITION_MAX_FLOAT);
    m_hood_motor.setSoftLimit(SoftLimitDirection.kForward, Constants.HOOD_POSITION_MIN_FLOAT);

    m_hood_motor.enableSoftLimit(SoftLimitDirection.kForward, true);

  }

  public static void disableSoftLimits() {

    m_hood_motor.enableSoftLimit(SoftLimitDirection.kForward, false);

  }

  public static void runHoodUp() {

    m_hood_pidController.setReference(.1, ControlType.kDutyCycle);

    SmartDashboard.putString("Hood Status", "!!Manual Override!!");

  }

  public void runHoodDown() {

    m_hood_pidController.setReference(-.1, ControlType.kDutyCycle);

    SmartDashboard.putString("Hood Status", "!!Manual Override!!");

  }

  public static double getHoodCurrentDraw() {

    return m_hood_motor.getOutputCurrent();

  }

  public boolean isHoodCurrentOverZeroConstant() {

    if (getHoodCurrentDraw() > Constants.HOOD_ZERO_CURRENT_DRAW) {

      return true;

    } else {

      return false;

    }

  }

  public static boolean isVelocityBasicallyZero() {

    if (Math.abs(getHoodVelocity()) < Constants.HOOD_ZERO_VELOCITY_THRESHOLD_UB) { // taking absolure value makes zero
                                                                                   // lower bound

      return true;

    } else {

      return false;

    }

  }

  public boolean isHoodInRange(){
    return Math.abs(goalPos - m_hood_encoder.getPosition()) < .1;
  }

  public static double getHoodVelocity() {

    return m_hood_encoder.getVelocity();

  }

  public void zeroHood() {

    isZeroed = true;

    m_hood_encoder.setPosition(0.0);

    SmartDashboard.putString("Hood Status", "Zeroed");

  }

}
