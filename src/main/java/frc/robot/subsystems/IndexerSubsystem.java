// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IndexerSubsystem extends SubsystemBase {

  public static DigitalInput bottomBeamBreak, topBeamBreak;
  public static CANSparkMax m_indexerRunner;


  public static double rotations = 0;
  private static SparkMaxPIDController m_indexerRunner_PidController;
  private static RelativeEncoder m_indexerRunner_encoder;
  

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {

    topBeamBreak = new DigitalInput(Constants.topBeamBreak_CAN_ID);
    bottomBeamBreak = new DigitalInput(Constants.bottomBeamBreak_CAN_ID);
  
    m_indexerRunner = new CANSparkMax(Constants.INDEXER_CAN_ID, MotorType.kBrushless);

    m_indexerRunner.restoreFactoryDefaults();
    m_indexerRunner.setIdleMode(IdleMode.kCoast);

    m_indexerRunner_PidController = m_indexerRunner.getPIDController();
    
    m_indexerRunner_encoder = m_indexerRunner.getEncoder();
    m_indexerRunner_PidController.setI(Constants.INDEXER_kI);
    m_indexerRunner_PidController.setP(Constants.INDEXER_kP);
    m_indexerRunner_PidController.setIZone(Constants.INDEXER_kIz);
    m_indexerRunner_PidController.setFF(Constants.INDEXER_kFF);
    m_indexerRunner_PidController.setOutputRange(Constants.INDEXER_kMinOutput, Constants.INDEXER_kMaxOutput);
    m_indexerRunner_PidController.setD(Constants.INDEXER_kD);

  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("bottomBeamBreak", bottomBeamBreak.get());
    SmartDashboard.putBoolean("topBeamBreak", topBeamBreak.get());


    
    //if(bottomBeamBreak.get() && !topBeamBreak.get()){
    //  m_indexerRunner_PidController.setReference(.1, ControlType.kDutyCycle);
    //} else {
    //  m_indexerRunner_PidController.setReference(0, ControlType.kDutyCycle);
    //}

  }

  public static void runIn(){

    m_indexerRunner_PidController.setReference(1, ControlType.kDutyCycle);

  }

  public static void runOut(){

    m_indexerRunner_PidController.setReference(-1, ControlType.kDutyCycle);

  }
  
}
