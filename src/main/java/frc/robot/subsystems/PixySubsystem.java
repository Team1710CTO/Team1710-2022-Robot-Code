// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2Line;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import io.github.pseudoresonance.pixy2api.Pixy2Line.Vector;


public class PixySubsystem extends SubsystemBase {
  /** Creates a new PixySubsystem. */
  //variables
  public Pixy2 pixy;
  private static PixySubsystem instance;
  private static final int minLineLength = 25;
  private int frameMid = 0;
  private double lineAngle = 0.0;
  private int linePosition = 0;
  private boolean hasLine = false;

  private static PixySubsystem getInstance() {
    if (instance == null)
        instance = new PixySubsystem();
        return instance;
  }
  

  public PixySubsystem() {
    pixy = Pixy2.createInstance(new SPILink());
    pixy.init(); // Initializes the camera and prepares to send/receive

    pixy.getLine().setMode(Pixy2Line.LINE_MODE_WHITE_LINE);
    pixy.setLamp((byte) 1, (byte) 1);
    pixy.setLED(255, 255, 255);
    int frameMid = pixy.getFrameHeight() / 2;
  }

  public void execute() {
    pixy.getLine().getMainFeatures();
    Vector v[] = pixy.getLine().getVectorCache();
    double testAngle = 0.0;
    double testLength = 0.0;
    int testPosition = 0;
    if (v != null) {
      for (int i = 0; i < v.length; i++) {
        double x = v[i].getX1() - v[i].getX0();
        double y = v[i].getY1() - v[i].getY0();
        double angle = 90.0;
        if (x != 0)
          angle = -Math.toDegrees(Math.atan(y / x));
        double length = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        int position = (v[i].getY0() + v[i].getY1()) / 2;
        if (length > testLength) {
          testLength = length;
          testAngle = angle;
          testPosition = position;
        }
      }
    }
    if (testLength >= minLineLength) {
      lineAngle = testAngle;
      hasLine = true;
      linePosition = frameMid - testPosition;
    } else {
      lineAngle = 0;
      hasLine = false;
      linePosition = frameMid;
    }
  }

  public double getLineAngle() {
    return lineAngle;
  }

  public double getLinePosition() {
    return linePosition;
  }

  public boolean hasLine() {
    return hasLine;
  }

  public boolean isFinished() {
    return false;
  }

  public void setRunWhenDisabled(boolean b) {

  }

  public void start(){

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
