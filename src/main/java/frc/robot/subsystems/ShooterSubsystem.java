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

    // WILL REMOVE AFTER TESTING
    // display PID coefficients on SmartDashboard 
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
  }

  @Override
  public void periodic(){
    // WILL REMOVE ALL OF PERIODIC AFTER TESTING 
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    
    // WILL REMOVE AFTER TESTING
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if(p != kP) { m_pidController.setP(p); kP = p; }
    if(i != kI) { m_pidController.setI(i); kI = i; }
    if(d != kD) { m_pidController.setD(d); kD = d; }
    if(iz != kIz) { m_pidController.setIZone(iz); kIz = iz; }
    if(ff != kFF) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
  }

  public void setSpeed(double setPoint){
    // Sets the requested RPM in the PID
    m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    SmartDashboard.putNumber("Setpoint", setPoint); // Puts the requested RPM to SmartDashboard
    SmartDashboard.putNumber("CurrentPoint", m_encoder.getVelocity()); // Puts the actual RPM to SmartDashboard
  }

  public void disableShooter(){
    // Sets the RPM to 0 for when we aren't shooting
    m_pidController.setReference(0, CANSparkMax.ControlType.kVelocity);
  }
}
