// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PowerDistributionSubsystem extends SubsystemBase {
  /** Creates a new PowerDistributionSubsystem. */

  private static PowerDistribution pdp;

  private static PowerDistributionSubsystem instance = null;






  public PowerDistributionSubsystem() {


    pdp = new PowerDistribution(0, ModuleType.kCTRE);

    SmartDashboard.putNumber("Battery Voltage", pdp.getVoltage());
    

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Battery Voltage", pdp.getVoltage());
    


  }
  public static double getTotalSwerveCurrentDraw() {


    double BLD = pdp.getCurrent(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR_PDP_SLOT);
    double BLS = pdp.getCurrent(Constants.BACK_LEFT_MODULE_STEER_MOTOR_PDP_SLOT); 

    double FLD = pdp.getCurrent(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR_PDP_SLOT);
    double FLS = pdp.getCurrent(Constants.FRONT_LEFT_MODULE_STEER_MOTOR_PDP_SLOT); 

    double BRD = pdp.getCurrent(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR_PDP_SLOT);
    double BRS = pdp.getCurrent(Constants.BACK_RIGHT_MODULE_STEER_MOTOR_PDP_SLOT); 

    double FRD = pdp.getCurrent(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR_PDP_SLOT);
    double FRS = pdp.getCurrent(Constants.FRONT_RIGHT_MODULE_STEER_MOTOR_PDP_SLOT); 

    return (BLD + BLS + FLD + FLS + BRD + BRS + FRD + FRS);

  }

  public static boolean driveCurrentDrawIsAbnormal(){

    

    return false;
  }




}
