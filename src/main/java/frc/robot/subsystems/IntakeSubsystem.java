// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervedrivespecialties.swervelib.DriveController;

import edu.wpi.first.wpilibj.PowerDistribution;


public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new intakeSubsystem. */
  private static IntakeSubsystem instance = null;

  private CANSparkMax m_intakeLeft;
  private static SparkMaxPIDController m_intakeLeftPidController;
  private RelativeEncoder m_intakeLeftEncoder;
  public static double rotAcctual, dutyCyclePile, dutyCylcePos, initialRot, rotationalError, zeroPos;
  private static boolean isZeroed;

  public IntakeSubsystem() {

    dutyCylcePos = 0;
    isZeroed = false;


    m_intakeLeft = new CANSparkMax(Constants.LEFT_INTAKE_ACTUATOR_CAN_ID, MotorType.kBrushless);
    m_intakeLeft.restoreFactoryDefaults();

    m_intakeLeftPidController = m_intakeLeft.getPIDController();

    m_intakeLeftEncoder = m_intakeLeft.getEncoder();

    initialRot = getIntakePos();
    
    // set PID coefficients
    m_intakeLeftPidController.setP(Constants.INTAKE_LEFT_kP);
    m_intakeLeftPidController.setI(Constants.INTAKE_LEFT_kI);
    m_intakeLeftPidController.setD(Constants.INTAKE_LEFT_kD);
    m_intakeLeftPidController.setIZone(Constants.INTAKE_LEFT_kIz);
    m_intakeLeftPidController.setFF(Constants.INTAKE_LEFT_kFF);
    m_intakeLeftPidController.setOutputRange(Constants.INTAKE_LEFT_kMinOutput, Constants.INTAKE_LEFT_kMaxOutput);

    // display PID coefficients on SmartDashboard
    
    SmartDashboard.putNumber("Intake P Gain", Constants.INTAKE_LEFT_kP);
    SmartDashboard.putNumber("Intake I Gain", Constants.INTAKE_LEFT_kI);
    SmartDashboard.putNumber("Intake D Gain", Constants.INTAKE_LEFT_kD);
    SmartDashboard.putNumber("Intake I Zone", Constants.INTAKE_LEFT_kIz);
    SmartDashboard.putNumber("Intake Feed Forward", Constants.INTAKE_LEFT_kFF);
    SmartDashboard.putNumber("Intake Max Output", Constants.INTAKE_LEFT_kMaxOutput);
    SmartDashboard.putNumber("Intake Min Output", Constants.INTAKE_LEFT_kMinOutput);
    SmartDashboard.putNumber("Intake Set Rotations", 0);

    rotAcctual = getIntakePos();

    m_intakeLeftPidController.setReference(rotAcctual, CANSparkMax.ControlType.kPosition);

  }

  public static IntakeSubsystem getInstance() {
		if(instance == null)
			instance = new IntakeSubsystem();

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
      if((p != Constants.INTAKE_LEFT_kP)) { m_intakeLeftPidController.setP(p); Constants.INTAKE_LEFT_kP = p; } //bro I klnow we are changing "constants" but this fucntionality is only for testing
      if((i != Constants.INTAKE_LEFT_kI)) { m_intakeLeftPidController.setI(i); Constants.INTAKE_LEFT_kI = i; } //once we enter comps we can disable this feature
      if((d != Constants.INTAKE_LEFT_kD)) { m_intakeLeftPidController.setD(d); Constants.INTAKE_LEFT_kD = d; } // if we can get over the convetions of it all then we can all be happy :)
      if((iz != Constants.INTAKE_LEFT_kIz)) { m_intakeLeftPidController.setIZone(iz); Constants.INTAKE_LEFT_kIz = iz; }
      if((ff != Constants.INTAKE_LEFT_kFF)) { m_intakeLeftPidController.setFF(ff); Constants.INTAKE_LEFT_kFF = ff; }
      if((max != Constants.INTAKE_LEFT_kMaxOutput) || (min != Constants.INTAKE_LEFT_kMinOutput)) { 
        m_intakeLeftPidController.setOutputRange(min, max); 
        Constants.INTAKE_LEFT_kMinOutput = min; Constants.INTAKE_LEFT_kMaxOutput = max; 
        }

    }
    
    zeroIntakePeriodic();

    SmartDashboard.putNumber("Intake SetPoint", rotations);
    SmartDashboard.putNumber("Intake Rotations", m_intakeLeftEncoder.getPosition());

  }

  public void setIntakeAngle(double pos){

    m_intakeLeftPidController.setReference(pos, CANSparkMax.ControlType.kPosition);

  }

  public void setIntakeDutyCycle(double cycle){

    m_intakeLeftPidController.setReference(cycle, CANSparkMax.ControlType.kDutyCycle);

  }

  

  public void zeroIntakePeriodic(){

    if(!isZeroed){
      
     dutyCylcePos += Constants.INTAKE_LEFT_zero_dutyCycle__gain;

     dutyCylcePos = RobotContainer.m_controller.getRightTriggerAxis(); //fix me for automated
     
     setIntakeDutyCycle(dutyCylcePos);

      if(RobotContainer.m_controller.getXButton()){//PowerDistributionSubsystem.getRightIntakeActuatorCurrent() >= Constants.INTAKE_LEFT_abnormal_abnormal_current_draw

        zeroPos = getIntakePos();
        
        Constants.INTAKE_LEFT_down = zeroPos;

        isZeroed = true;

      }
    } 

    if(RobotContainer.m_controller.getYButton()){
      zeroIntake();                                         // keep to trigger action
    }

  }

  public void zeroIntake(){

    isZeroed = false;

  }

  public double getIntakePos(){
    return m_intakeLeftEncoder.getPosition();
  }

}