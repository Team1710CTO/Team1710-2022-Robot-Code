// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(23.0); // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(23.0); // FIXME Measure and set wheelbase

    

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; // FIXME Set front left module drive motor ID g
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 2; // FIXME Set front left steer encoder ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3; // FIXME Set front left module steer motor ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(75); // 75 FIXME Measure and set front left steer offset
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_PDP_SLOT = 0; //fixme
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR_PDP_SLOT = 4; //fixme

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 5; // FIXME Set front right steer encoder ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6; // FIXME Set front right steer motor ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(16 + 180); //  // FIXME Measure and set front right steer offset
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_PDP_SLOT = 1; // FIXME 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_PDP_SLOT = 5; // FIXME 

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 8; // FIXME Set back right steer encoder ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 9; // FIXME Set back right steer motor ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(335 + 180); //  335 + 180 FIXME Measure and set back right steer offset
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_PDP_SLOT = 2; //fixme
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR_PDP_SLOT = 6; //fixme

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11; // FIXME Set back left steer encoder ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 12; // FIXME Set back left steer motor ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(60); // 60
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR_PDP_SLOT = 3; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR_PDP_SLOT = 7;  
    
    //Gyros

    public static final int RIGHT_PIGEON_ID = 13; // FIXME Set Pigeon ID
    public static final int LEFT_PIGEON_ID = 15; // FIXME Set Pigeon ID
    

    public static double ROTATION_PID_CONTOLLER_kP = .12;
    public static double ROTATION_PID_CONTOLLER_kI = 0.01;
    public static double ROTATION_PID_CONTOLLER_kD = 0;
    public static int ROTATION_PID_ITERATOR_ACTIVATION_THRESHOLD = 17;
    public static double ROTATION_PID_SUPPLIER_ACTIVATION_THRESHOLD = 0.5;

    public static double DRIVE_SLEW_RATE_LIMIIER_BASE = 100;

    //intake

    // intake runner
    public static final int INTAKE_RUNNER_PDP_SLOT = 15;
    public static final int INTAKE_RUNNER_CAN_ID = 43;

    //intake Left
    public static final int LEFT_INTAKE_ACTUATOR_PDP_SLOT = 8;
    public static final int LEFT_INTAKE_ACTUATOR_CAN_ID = 31;
    public static double INTAKE_LEFT_kP = 0.15; 
    public static double INTAKE_LEFT_kI = 0.0001;
    public static double INTAKE_LEFT_kD = 1; 
    public static double INTAKE_LEFT_kIz = 0; 
    public static double INTAKE_LEFT_kFF = 0; 
    public static double INTAKE_LEFT_kMaxOutput = 1; 
    public static double INTAKE_LEFT_kMinOutput = -1;
    public static double INTAKE_LEFT_zero_dutyCycle__gain = 1e-5;
    public static double INTAKE_LEFT_abnormal_abnormal_current_draw = 10;
    public static final double Intake_LEFT_up = 0.5; // roations from zero FIXME
    public static final double INTAKE_LEFT_down = 3.3; // set by the zero functionality


    //intake Right
    public static final int RIGHT_INTAKE_ACTUATOR_PDP_SLOT = 11;  
    public static final int RIGHT_INTAKE_ACTUATOR_CAN_ID = 30;
    public static double INTAKE_RIGHT_kP = INTAKE_LEFT_kP; //copy values from left. 
    public static double INTAKE_RIGHT_kI = INTAKE_LEFT_kI;
    public static double INTAKE_RIGHT_kD = INTAKE_LEFT_kD; 
    public static double INTAKE_RIGHT_kIz = INTAKE_LEFT_kIz; 
    public static double INTAKE_RIGHT_kFF = INTAKE_LEFT_kFF; 
    public static double INTAKE_RIGHT_kMaxOutput = INTAKE_LEFT_kMaxOutput; 
    public static double INTAKE_RIGHT_kMinOutput = INTAKE_LEFT_kMinOutput;
    public static double INTAKE_RIGHT_zero_dutyCycle__gain = INTAKE_LEFT_zero_dutyCycle__gain;
    public static double INTAKE_RIGHT_abnormal_abnormal_current_draw = 10;
    public static final double INTAKE_RIGHT_up = -Intake_LEFT_up; // set by the zero functionality
    public static final double INTAKE_RIGHT_down = -INTAKE_LEFT_down; // set by the zero functionality

    public static double INTAKE_RUNNER_SPEED_ON = .75; //speed intake runner runs at
    public static double INTAKE_RUNNER_SPEED_OFF = 0; // read the var
    public static double INTAKE_ZERO_VELOCITY_THRESHOLD_UB = .01; // this is the upper bound for the is intake velocity near or at Zero threshold



    //hood
    public static final int HOOD_CAN_ID = 41;
    public static final int HOOD_PDP_SLOT = 9; //fixme
    public static double HOOD_kP = 0.6; 
    public static double HOOD_kI = 0.001; //or 0.00005? test
    public static double HOOD_kD = 0; 
    public static double HOOD_kIz = 0; 
    public static double HOOD_kFF = 0; 
    public static double HOOD_kMaxOutput = 1; 
    public static double HOOD_kMinOutput = -1;
    public static double HOOD_zero_dutyCycle__gain = 1e-5;
    public static double HOOD_abnormal_abnormal_current_draw = 10;
    public static final double HOOD_POSITION_MAX = 1.2 ; // roations from zero FIXME
    public static final double HOOD_POSITION_MIN = .05; // set by the zero functionality
    public static float HOOD_POSITION_MAX_FLOAT = (float)HOOD_POSITION_MAX; // needed for soft limits
    public static float HOOD_POSITION_MIN_FLOAT = (float)HOOD_POSITION_MIN;// needed for soft limits

    public static final double HOOD_ZERO_CURRENT_DRAW = 32;
    public static final double HOOD_ZERO_VELOCITY_THRESHOLD_UB = .05;

 //shooter

 public static double SHOOTER_Kc_PERIOD = .7;

 public static double SHOOTER_Kc = .01;


 public static final int SHOOTER_PDP_SLOT = 12; // GOOD
 public static final int SHOOTER_CAN_ID = 40; // GOOD
 public static double SHOOTER_kP = 0.000536; // GOOD
 public static double SHOOTER_kI = 0.00000; // GOOD
 public static double SHOOTER_kD = 0.0; // GOOD
 public static double SHOOTER_kIz = 0; 
 public static double SHOOTER_kFF = 0.00018;  // GOOD
 public static double SHOOTER_kMaxOutput = 1; // GOOD
 public static double SHOOTER_kMinOutput = -1; // GOOD
 public static double SHOOTER_zero_dutyCycle__gain = 1e-5;
 public static double SHOOTER_abnormal_abnormal_current_draw = 10;      

    public static final double SHOOTER_GO_THRESHHOLD = 300;
    
    

    public static final int CLIMBER_SERVO_PWM_CHANNEL = 0;

    public static final double CLIMBER_SERVO_LOCK_POSITION = 180;
    public static final double CLIMBER_SERVO_DISENGAGE_POSITION = 165.5;

    public static final int CLIMBER_TOP_TALON_CAN_ID = 50;
    public static final int CLIMBER_BOTTOM_TALON_CAN_ID = 51;
    public static final int kTimeoutMs = 30;




    public static final int INDEXER_CAN_ID = 42;

    public static final int bottomBeamBreak_CAN_ID = 1;

    public static final int topBeamBreak_CAN_ID = 0;
    public static final int INTAKE_CURRENT_LIMIT = 60;
    public static final double INDEXER_IN_SPEED = .3
    ;
    public static final double INDEXER_OUT_SPEED = -1;
    public static final double INDEXER_STOP_SPEED = 0;
    public static final double INDEXER_CYCLE_ROTATIONS = 15;
    public static final double CLIMBER_POSITION_UP = 1.7e5 * .9;

    public static final double CLIMBER_POSITION_HALF = (1.7e5)/4;
    
    public static final double CLIMBER_POSITION_DOWN = 5000;
    public static final double CLIMBER_ZERO_THRESHOLD = 30;
    
    public static final double CLIMBER_OUT_POSITION = 1.65e5;

    public static final double CLIMBER_IN_POSITION = 0;
    
    

    public static double INDEXER_kP = 0.01; //TODO
    public static double INDEXER_kI = 1e-4; //TODO
    public static double INDEXER_kD = 1; 
    public static double INDEXER_kIz = 0; 
    public static double INDEXER_kFF = 0; 
    public static double INDEXER_kMaxOutput = 1; 
    public static double INDEXER_kMinOutput = -1;
      
    


}
