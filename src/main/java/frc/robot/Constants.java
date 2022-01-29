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
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(24.0); // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(24.0); // FIXME Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 13; // FIXME Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; // FIXME Set front left module drive motor ID g
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 2; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(255); // FIXME Measure and set front left steer offset
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_PDP_SLOT = 1; //fixme
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR_PDP_SLOT = 3; //fixme

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 5; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(196.7 - 180); // FIXME Measure and set front right steer offset
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_PDP_SLOT = 4; // FIXME 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_PDP_SLOT = 6; // FIXME 



    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 9; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 8; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0); // FIXME Measure and set back right steer offset
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_PDP_SLOT = 7; //fixme
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR_PDP_SLOT = 9; //fixme


    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 12; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0); // FIXME Measure and set back left steer offset
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR_PDP_SLOT = 10; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR_PDP_SLOT = 12;


    public static final int SERVO_CHANNEL = 0; //Test servo channel on PWM
    
    public static final int HOOD_CAN_ID = 30;
    public static final int HOOD_PDP_SLOT = 11; //fixme

}
