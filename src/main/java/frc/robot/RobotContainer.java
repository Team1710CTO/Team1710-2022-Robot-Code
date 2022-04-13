// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*; // import all commands 


import frc.robot.subsystems.*; // import all subsystems

public class RobotContainer {


  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final GyroSubsystem m_GyroSubsystem = new GyroSubsystem();

  public final static XboxController d_controller = new XboxController(0);

  public static final XboxController m_controller = new XboxController(1);

  public static IndexerSubsystem m_iIndexerSubsystem = new IndexerSubsystem();

  public static HoodSubsystem mHoodSubsystem = new HoodSubsystem();
  
  public static ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();

  public static IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();

 // public static LedSubsystem ledSubsystem = new LedSubsystem();

  public static PhotonVisionSubsystem mphotonVisionSubsystem = new PhotonVisionSubsystem();
  

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  
  
  //public static PhotonVisionSubsystem mPhotonVisionSubsystem = new PhotonVisionSubsystem();

  public static ClimberSubsystem mClimberSubsystem = new ClimberSubsystem();

  public Command fourBallAutoBlue = new FourBallAutoAtCrotchHudson("BLUE", m_drivetrainSubsystem, mIntakeSubsystem, mphotonVisionSubsystem, m_iIndexerSubsystem, mHoodSubsystem, mShooterSubsystem, m_GyroSubsystem);
  
  public Command fourBallAutoRed = new FourBallAutoAtCrotchHudson("RED", m_drivetrainSubsystem, mIntakeSubsystem, mphotonVisionSubsystem, m_iIndexerSubsystem, mHoodSubsystem, mShooterSubsystem, m_GyroSubsystem);
  
  public Command threeBallAutoBlue = new ThreeBallAutoAtCrotch("BLUE", m_drivetrainSubsystem, mIntakeSubsystem, mphotonVisionSubsystem, m_iIndexerSubsystem, mHoodSubsystem, mShooterSubsystem, m_GyroSubsystem);

  public Command threeBallAutoRed = new ThreeBallAutoAtCrotch("RED", m_drivetrainSubsystem, mIntakeSubsystem, mphotonVisionSubsystem, m_iIndexerSubsystem, mHoodSubsystem, mShooterSubsystem, m_GyroSubsystem);

  public Command twoBallAutoBlue = new TwoBallFromWherever("BLUE", m_drivetrainSubsystem, mIntakeSubsystem, mphotonVisionSubsystem, m_iIndexerSubsystem, mHoodSubsystem, mShooterSubsystem, m_GyroSubsystem);

  public Command twoBallAutoRed = new TwoBallFromWherever("RED", m_drivetrainSubsystem, mIntakeSubsystem, mphotonVisionSubsystem, m_iIndexerSubsystem, mHoodSubsystem, mShooterSubsystem, m_GyroSubsystem);

  public Command FunkeyFive = new FunkyFiveBall("RED",m_drivetrainSubsystem, mIntakeSubsystem, mphotonVisionSubsystem, m_iIndexerSubsystem, mHoodSubsystem, mShooterSubsystem, m_GyroSubsystem);

  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

    m_chooser.setDefaultOption("4 ball BLUE", fourBallAutoBlue);
    m_chooser.setDefaultOption("4 ball RED", fourBallAutoRed);

    m_chooser.setDefaultOption("3 ball BLUE", threeBallAutoBlue);
    m_chooser.setDefaultOption("3 ball RED", threeBallAutoRed);

    m_chooser.setDefaultOption("2 ball BLUE", twoBallAutoBlue);
    m_chooser.setDefaultOption("2 ball RED", twoBallAutoRed);

    m_chooser.setDefaultOption("funkyFive", FunkeyFive);

    

// Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(d_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(d_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(d_controller.getRightX() * .675) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    mShooterSubsystem.setDefaultCommand(new DefaultShooterCommand(mShooterSubsystem));

    m_iIndexerSubsystem.setDefaultCommand(new DefaultIndexerCommand(m_iIndexerSubsystem));

  //ledSubsystem.setDefaultCommand(new LEDcommand(ledSubsystem));

    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Button(d_controller::getBackButton)
            .whenPressed(m_GyroSubsystem::zeroBestGyro)
            .whenReleased(m_GyroSubsystem::setIsZeroingFalse)
            .whenReleased(m_drivetrainSubsystem::enableHeadingControl);


   
    

    new Button(d_controller::getStartButton)
            .whenPressed(new ZeroIntake(mIntakeSubsystem))
            .whenPressed(new ZeroHood(mHoodSubsystem))
            .whenPressed(m_drivetrainSubsystem::resetOdometry)
            .whenPressed(m_iIndexerSubsystem::zeroBallCount);
           

            

    new Button(d_controller::getRightBumper)
            .whenHeld(new Intake(mIntakeSubsystem));

    new Button(d_controller::getLeftBumper)
            .whenHeld(new outtake(mIntakeSubsystem, m_iIndexerSubsystem));
    
    new Button(d_controller::getAButton)
            //.whenHeld(new ClimbHalf(mClimberSubsystem));
            .whenHeld(new Shoot(mShooterSubsystem, mHoodSubsystem, m_iIndexerSubsystem, mphotonVisionSubsystem, m_drivetrainSubsystem));

    new Button(d_controller::getBButton)
            //.whenHeld(new ClimbHalf(mClimberSubsystem));
            .whenHeld(new ShootWithoutVision(mShooterSubsystem, mHoodSubsystem, m_iIndexerSubsystem));
    

    new Button(d_controller::getXButton)
            //.whenHeld(new ClimbHalf(mClimberSubsystem));
            .whenHeld(new IntakeWithVision(mIntakeSubsystem, m_drivetrainSubsystem, mphotonVisionSubsystem, m_iIndexerSubsystem));



    new Button(m_controller::getXButton)
                .whenPressed(new climberBootSequence(mClimberSubsystem));
    
    new Button(m_controller::getYButton)
                .whenHeld(new ClimbUp(mClimberSubsystem));
     //
    new Button(m_controller::getBButton)
                .whenHeld(new ClimbHalf(mClimberSubsystem));

    new Button(m_controller::getAButton)
                .whenHeld(new ClimbDown(mClimberSubsystem));

    new Button(m_controller::getStartButton)
                .whenHeld(new ShootInLow(mShooterSubsystem, mHoodSubsystem, m_iIndexerSubsystem));

    new Button(m_controller::getRightBumper).whenHeld(new ClimbOverrideUp(mClimberSubsystem));
    new Button(m_controller::getLeftBumper).whenHeld(new ClimbOverrideDown(mClimberSubsystem));


    new Button(m_controller::getBackButton).whenHeld(new indexerInoverride(m_iIndexerSubsystem));
    

    
    //new Button(d_controller::getStartButton)
    //        .whenPressed(new ZeroIntake());
            
    //new Button(d_controller::getAButton).whenPressed(new climberActuatorIn(servoSubsystem));
    
    //new Button(d_controller::getBButton).whenPressed(new climberActuatorOut(servoSubsystem));
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();

  }

  

  private static double deadband(double value, double deadband) {

    if (Math.abs(value) > deadband) {

      if (value > 0.0) {

        return (value - deadband) / (1.0 - deadband);

      } else {

        return (value + deadband) / (1.0 - deadband);

      }

    } else {

      return 0.0;
      
    }
  }

  public static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.15);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }


}
