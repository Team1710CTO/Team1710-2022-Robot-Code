// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IndexerSubsystem extends SubsystemBase {

  public static DigitalInput bottomBeamBreak;
  public static DigitalInput topBeamBreak;
  public static CANSparkMax m_indexerRunner;


  public static double rotations = 0;
  private static SparkMaxPIDController m_indexerRunner_PidController;
  private static RelativeEncoder m_indexerRunner_encoder;

  public static int balls = 0;

  public static int ttfCounter = 0;

  public static boolean lastUpperBeamBreak, Lastbol = false;

  public static int ballintegralBottom, ballintegralTop = 0;

  public static Timer bottomTimer, topTimer, deBounceTimer;


  

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {



    topBeamBreak = new DigitalInput(Constants.topBeamBreak_CAN_ID);
    bottomBeamBreak = new DigitalInput(Constants.bottomBeamBreak_CAN_ID);
  
    m_indexerRunner = new CANSparkMax(Constants.INDEXER_CAN_ID, MotorType.kBrushless);

    m_indexerRunner.restoreFactoryDefaults();
    m_indexerRunner.setIdleMode(IdleMode.kBrake);

    m_indexerRunner_PidController = m_indexerRunner.getPIDController();
    
    m_indexerRunner_encoder = m_indexerRunner.getEncoder();
    m_indexerRunner_PidController.setI(Constants.INDEXER_kI);
    m_indexerRunner_PidController.setP(Constants.INDEXER_kP);
    m_indexerRunner_PidController.setIZone(Constants.INDEXER_kIz);
    m_indexerRunner_PidController.setFF(Constants.INDEXER_kFF);
    m_indexerRunner_PidController.setOutputRange(Constants.INDEXER_kMinOutput, Constants.INDEXER_kMaxOutput);
    m_indexerRunner_PidController.setD(Constants.INDEXER_kD);

    m_indexerRunner_PidController.setReference(rotations, ControlType.kPosition);

    topTimer = new Timer();
    bottomTimer = new Timer();
    deBounceTimer = new Timer();


    topTimer.reset();
    bottomTimer.reset();
    deBounceTimer.reset();
    deBounceTimer.start();

  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("bottomBeamBreak", bottomBeamBreak.get());
    SmartDashboard.putBoolean("topBeamBreak", topBeamBreak.get());

    

    //SmartDashboard.putNumber("indexer Rotations", getIndexerRotations());

    //SmartDashboard.putBoolean("is cycled", isIndexerCycled());

    m_indexerRunner_encoder.getVelocity();

   SmartDashboard.putNumber("indxer velocity", m_indexerRunner_encoder.getVelocity());

   SmartDashboard.putNumber("balls", balls);

   // SmartDashboard.putNumber("ball integral", ballintegralBottom);
    

    countBallsPeriodic();


    countTtf();
    


  }

  public static int getBallCount(){

    return balls;
    
  }

  public static void countBallsPeriodic(){

    if(!bottomBeamBreak.get() && !topBeamBreak.get()){

      balls = 2;     

    } else if(!bottomBeamBreak.get() && m_indexerRunner_encoder.getVelocity() > 0){

      bottomTimer.start();

      if(bottomTimer.get() > .25){

        balls = 1;

      } 

    } else {

      bottomTimer.reset();
      topTimer.reset();

    }    

    if(Lastbol != topBeamBreak.get() && !Lastbol && m_indexerRunner_encoder.getVelocity() > 0 && deBounceTimer.get() > .8){

      balls -= 1;

      deBounceTimer.stop();
      deBounceTimer.reset();
      deBounceTimer.start();

    }

    Lastbol = topBeamBreak.get();
    

    SmartDashboard.putNumber("debounce timer", deBounceTimer.get());

  }

  public static void runIndexerIn(){

    m_indexerRunner_PidController.setReference(Constants.INDEXER_IN_SPEED, ControlType.kDutyCycle);

  }

  public static void runIndexerInMed(){

    m_indexerRunner_PidController.setReference(.5, ControlType.kDutyCycle);

  }

  public static void runindexerInFAST(){

    m_indexerRunner_PidController.setReference(1, ControlType.kDutyCycle);

  }
  public static void runIndexerOut(){

    m_indexerRunner_PidController.setReference(Constants.INDEXER_OUT_SPEED, ControlType.kDutyCycle);

  }

  public static void runIndexerOutNORM(){

    m_indexerRunner_PidController.setReference(-.3, ControlType.kDutyCycle);

  }


  public void stopIndexer(){

    m_indexerRunner_PidController.setReference(Constants.INDEXER_STOP_SPEED, ControlType.kDutyCycle);

    m_indexerRunner_encoder.setPosition(0.0);

  }

  public static boolean isBottomBeakBreakTripped(){

    return bottomBeamBreak.get();

  }

  public boolean isTopBeakBreakTripped(){

    return topBeamBreak.get();

  }

  public void indexBallsBetweenBreaks(){

    if(!isBottomBeakBreakTripped() && isTopBeakBreakTripped()){

        runIndexerIn();

    } else if(isBottomBeakBreakTripped() && !isTopBeakBreakTripped()) { 

        runIndexerOutNORM();

    } else {

        stopIndexer();

    }


  }

  public void zeroBallCount(){

    balls = 0;

  }

  public static double getIndexerRotations(){

    return m_indexerRunner_encoder.getPosition();

  }

  public boolean isIndexerCycled(){

    return (getIndexerRotations() > Constants.INDEXER_CYCLE_ROTATIONS);

  }

  public void resetTtfCounter(){

    ttfCounter = 0;

  }

  public void countTtf(){

    if(lastUpperBeamBreak = false && lastUpperBeamBreak != topBeamBreak.get()){

      ttfCounter += 1;

    }

    lastUpperBeamBreak = topBeamBreak.get();

  }


  
}
