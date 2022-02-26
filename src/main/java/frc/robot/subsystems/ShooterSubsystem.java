package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;
  private static RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  public static double goalSpeed = 0;

  public static boolean isDisabled = true;

  public ShooterSubsystem(){
    // initialize motor
    m_motor = new CANSparkMax(Constants.SHOOTER_CAN_ID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();



    m_pidController = m_motor.getPIDController();

    

    // Encoder object created to track and display current RPM
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    // WILL GET ALL PID VALUES FROM FROM THE CONSTANTS FILE AFTER TESTING
    kP = Constants.SHOOTER_kP; 
    kI = Constants.SHOOTER_kI; 
    kD = Constants.SHOOTER_kD; 
    kIz = Constants.SHOOTER_kIz; 
    kFF = Constants.SHOOTER_kFF;
    kMaxOutput = Constants.SHOOTER_kMaxOutput;
    kMinOutput = Constants.SHOOTER_kMinOutput;
    maxRPM = 5700;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

 
  }

  @Override
  public void periodic(){
   
    SmartDashboard.putNumber("Velo shooter", m_encoder.getVelocity()); // Puts the actual RPM to SmartDashboard
    SmartDashboard.putNumber("GVelo shooter", goalSpeed);
  }

  public void setSpeed(double setPoint){

    if(setPoint != goalSpeed){

      goalSpeed = setPoint;

      m_pidController.setReference(goalSpeed, CANSparkMax.ControlType.kVelocity);

    }
    
    isDisabled = false;

  }

  public void disableShooter(){

    isDisabled = true;

    goalSpeed = 0;
    // Sets the RPM to 0 for when we aren't shooting
    //setSpeed(0);

    m_pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
    
  }
  public boolean isShooterToSpeedAndNotDisabled(){

    

    if(!isDisabled && isShooterToSpeed()){

      return true;

    } else {

      return false;

    }

  }

  public static boolean isShooterToSpeed(){

    SmartDashboard.putNumber("error", Math.abs(goalSpeed - m_encoder.getVelocity()));

    if(Math.abs(goalSpeed - m_encoder.getVelocity()) < Constants.SHOOTER_GO_THRESHHOLD){

      return true;

    } else {

      return false;

    }

  }

  public static double getShooterSpeed(){

    return m_encoder.getVelocity();
  }


  public void runShooterIn(){


    m_pidController.setReference(-.35, ControlType.kDutyCycle);

  }

  public void runShooterOut(){

    m_pidController.setReference(.35, ControlType.kDutyCycle);
    
  }
  
  
}
