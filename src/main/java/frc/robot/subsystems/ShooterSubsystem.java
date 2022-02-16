package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  public ShooterSubsystem(){
    // initialize motor
    m_motor = new CANSparkMax(Constants.SHOOTER_CAN_ID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();

    m_pidController = m_motor.getPIDController();

    // Encoder object created to track and display current RPM
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    // WILL GET ALL PID VALUES FROM FROM THE CONSTANTS FILE AFTER TESTING
    kP = 0.0001; // TODO
    kI = 0.000001; // TODO
    kD = 0; 
    kIz = 0; // TODO?
    kFF = 0; // TODO
    kMaxOutput = .5; // TODO
    kMinOutput = 0; // TODO
    maxRPM = 5700; // TODO

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
    // WILL REMOVE ALL OF PERIODIC AFTER TESTING 
    // read PID coefficients from SmartDashboard
   
    
  }

  public void setSpeed(double setPoint){
    // Sets the requested RPM in the PID
    m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    //SmartDashboard.putNumber("Setpoint", setPoint); // Puts the requested RPM to SmartDashboard
    SmartDashboard.putNumber("CurrentPoint shooter", m_encoder.getVelocity()); // Puts the actual RPM to SmartDashboard
  }

  public void disableShooter(){
    // Sets the RPM to 0 for when we aren't shooting
    m_pidController.setReference(0, CANSparkMax.ControlType.kVelocity);
  }
}
