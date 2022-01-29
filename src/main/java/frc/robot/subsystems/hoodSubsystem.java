// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistribution;


public class hoodSubsystem extends SubsystemBase {
  /** Creates a new hoodSubsystem. */
  private static hoodSubsystem instance = null;

  private CANSparkMax m_hood;
  private static SparkMaxPIDController m_hoodPidController;
  private RelativeEncoder m_hoodEncoder;
  public static double rotAcctual, dutyCyclePile, dutyCylcePos, initialRot, rotationalError, zeroPos;
  private static boolean isZeroed;

  public hoodSubsystem() {

    dutyCylcePos = 0;

    m_hood = new CANSparkMax(Constants.HOOD_CAN_ID, MotorType.kBrushless);
    m_hood.restoreFactoryDefaults();

    m_hoodPidController = m_hood.getPIDController();

    m_hoodEncoder = m_hood.getEncoder();

    initialRot = getHoodPos();
    
    isZeroed = false;

    // set PID coefficients
    m_hoodPidController.setP(Constants.HOOD_kP);
    m_hoodPidController.setI(Constants.HOOD_kI);
    m_hoodPidController.setD(Constants.HOOD_kD);
    m_hoodPidController.setIZone(Constants.HOOD_kIz);
    m_hoodPidController.setFF(Constants.HOOD_kFF);
    m_hoodPidController.setOutputRange(Constants.HOOD_kMinOutput, Constants.HOOD_kMaxOutput);

    // display PID coefficients on SmartDashboard
    
    SmartDashboard.putNumber("Hood P Gain", Constants.HOOD_kP);
    SmartDashboard.putNumber("Hood I Gain", Constants.HOOD_kI);
    SmartDashboard.putNumber("Hood D Gain", Constants.HOOD_kD);
    SmartDashboard.putNumber("Hood I Zone", Constants.HOOD_kIz);
    SmartDashboard.putNumber("Hood Feed Forward", Constants.HOOD_kFF);
    SmartDashboard.putNumber("Hood Max Output", Constants.HOOD_kMaxOutput);
    SmartDashboard.putNumber("Hood Min Output", Constants.HOOD_kMinOutput);
    SmartDashboard.putNumber("Hood Set Rotations", 0);

    rotAcctual = getHoodPos();

    m_hoodPidController.setReference(rotAcctual, CANSparkMax.ControlType.kPosition);

  }

  public static hoodSubsystem getInstance() {
		if(instance == null)
			instance = new hoodSubsystem();

		return instance;	
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    if(ControlModeSubsystem.isDeveloperOrTestMode()) {

    // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((p != Constants.HOOD_kP)) { m_hoodPidController.setP(p); Constants.HOOD_kP = p; } //bro I klnow we are changing "constants" but this fucntionality is only for testing
      if((i != Constants.HOOD_kI)) { m_hoodPidController.setI(i); Constants.HOOD_kI = i; } //once we enter comps we can disable this feature
      if((d != Constants.HOOD_kD)) { m_hoodPidController.setD(d); Constants.HOOD_kD = d; } // if we can get over the convetions of it all then we can all be happy :)
      if((iz != Constants.HOOD_kIz)) { m_hoodPidController.setIZone(iz); Constants.HOOD_kIz = iz; }
      if((ff != Constants.HOOD_kFF)) { m_hoodPidController.setFF(ff); Constants.HOOD_kFF = ff; }
      if((max != Constants.HOOD_kMaxOutput) || (min != Constants.HOOD_kMinOutput)) { 
        m_hoodPidController.setOutputRange(min, max); 
        Constants.HOOD_kMinOutput = min; Constants.HOOD_kMaxOutput = max; 
        }

    }
    
    zeroHoodPeriodic();

    SmartDashboard.putNumber("Hood SetPoint", rotations);
    SmartDashboard.putNumber("Hood Rotations", m_hoodEncoder.getPosition());

  }

  public void setHoodAngle(double pos){
    m_hoodPidController.setReference(pos, CANSparkMax.ControlType.kPosition);
  }

  

  public void zeroHoodPeriodic(){

    if(!isZeroed){
      
      dutyCylcePos += Constants.HOOD_zero_dutyCycle__gain;
      m_hoodPidController.setReference(dutyCylcePos, CANSparkMax.ControlType.kDutyCycle);

      if(PowerDistributionSubsystem.getRightIntakeActuatorCurrent() >= Constants.HOOD_abnormal_abnormal_current_draw){

        zeroPos = getHoodPos();
        isZeroed = true;

      }
    } 

  }

  public void zeroHood(){

    isZeroed = false;

  }

  public double getHoodPos(){
    return m_hoodEncoder.getPosition();
  }

}