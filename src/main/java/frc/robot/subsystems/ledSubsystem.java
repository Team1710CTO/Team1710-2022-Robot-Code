//1.5.2

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AddressableLED;

//import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.ShooterSubsystem;

public class LedSubsystem extends SubsystemBase {
  /** Creates a new ledSubsystem. */
  public LedSubsystem() {
  }

  public static AddressableLED m_led = new AddressableLED(1);
  public static AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(19);
  public final Timer m_timer = new Timer();

  int count = 0;

  private int r;
  private int g;
  private int b;

  private int rr;
  private int gg;
  private int bb;

  private int X = 0;
  private int n = 1;
  private int counter = 0;

  private int kitt = 0;

  public void setLength() {
    m_timer.reset();
    m_timer.start();
    m_led.setLength(m_ledBuffer.getLength());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void auto() {

    if (Timer.getMatchTime() <= 5) {
      blink(0, 200, 0, 1);
    }
    if (Timer.getMatchTime() <= 10) {
      blink(200, 200, 0, 1);
    }
    if (Timer.getMatchTime() <= 15) {
      blink(200, 0, 0, 1);
    }
  }

  public void solid(int red, int green, int blue) {

    for (int i = 0; i < (m_ledBuffer.getLength()); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, red, green, blue);
    }

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void fillPercent(int red, int green, int blue, int percent, boolean fromTop) {

    if (fromTop) {

      for (int i = 0; i < (m_ledBuffer.getLength()); i++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, red, green, blue);
      }

      for (int i = 0; i < (m_ledBuffer.getLength() - ((m_ledBuffer.getLength()) / percent)); i++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, 0, 0, 0);
      }

    } else {

      for (int i = 0; i < (m_ledBuffer.getLength()); i++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, 0, 0, 0);
      }

      for (int i = 0; i < ((m_ledBuffer.getLength()) / percent); i++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, red, green, blue);
      }

    }

  }

  public void orbit(int red, int green, int blue, int TPS, boolean up) {

    int num = (int) (TPS * m_timer.get());

    for (int j = 0; j < (m_ledBuffer.getLength()); j++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(j, 0, 0, 0);
    }

    if (up) {

      int i = (num % m_ledBuffer.getLength());
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, red, green, blue);

      // int j = (num % m_ledBuffer.getLength());
      //// Sets the specified LED to the RGB values for red
      // m_ledBuffer.setRGB(j, 0, 0, 0);

    } else {

      int i = (m_ledBuffer.getLength() - (num % m_ledBuffer.getLength()));
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, red, green, blue);

    }

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void tripleOrbit(int red, int green, int blue, int red_bg, int green_bg, int blue_bg, int TPS) {
    int num = (int) (TPS * m_timer.get());

    if (num % 3 == 0) {
      for (int e = 0; e < (m_ledBuffer.getLength()); e += 3) {
        m_ledBuffer.setRGB(e, red, green, blue);
      }
      for (int o = 1; o < (m_ledBuffer.getLength()); o += 3) {
        m_ledBuffer.setRGB(o, red_bg, green_bg, blue_bg);
      }
      for (int o = 2; o < (m_ledBuffer.getLength()); o += 3) {
        m_ledBuffer.setRGB(o, red_bg, green_bg, blue_bg);
      }

    } else if (num % 3 == 1) {
      for (int o = 0; o < (m_ledBuffer.getLength()); o += 3) {
        m_ledBuffer.setRGB(o, red_bg, green_bg, blue_bg);
      }
      for (int o = 1; o < (m_ledBuffer.getLength()); o += 3) {
        m_ledBuffer.setRGB(o, red_bg, green_bg, blue_bg);
      }
      for (int e = 2; e < (m_ledBuffer.getLength()); e += 3) {
        m_ledBuffer.setRGB(e, red, green, blue);
      }

    } else if (num % 3 == 2) {
      for (int o = 0; o < (m_ledBuffer.getLength()); o += 3) {
        m_ledBuffer.setRGB(o, red_bg, green_bg, blue_bg);
      }
      for (int e = 1; e < (m_ledBuffer.getLength()); e += 3) {
        m_ledBuffer.setRGB(e, red, green, blue);
      }
      for (int o = 2; o < (m_ledBuffer.getLength()); o += 3) {
        m_ledBuffer.setRGB(o, red_bg, green_bg, blue_bg);
      }

    }

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void knightRider(int red, int green, int blue) {
    if (counter >= m_ledBuffer.getLength()) {
      // counter == 2.4 * (m_ledBuffer.getLength())) {
      m_timer.reset();
      m_timer.start();
      solid(0, 0, 0);
      kitt++;
      counter = 0;
    }

    int num = (int) (10 * m_timer.get());

    for (int j = 0; j < (m_ledBuffer.getLength()); j++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(j, 0, 0, 0);
    }

    if (kitt % 2 == 0) {

      int i = (num % m_ledBuffer.getLength()) + 1;
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, red, green, blue);

      int j = (num % m_ledBuffer.getLength());
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(j, 0, 0, 0);

    } else if (kitt % 2 == 1) {

      int k = (m_ledBuffer.getLength() - (num % m_ledBuffer.getLength())) - 0;
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(k, red, green, blue);

      // int j = (m_ledBuffer.getLength() - (num % m_ledBuffer.getLength()));
      //// Sets the specified LED to the RGB values for red
      // m_ledBuffer.setRGB(j, 0, 0, 0);

      // int i = (num % m_ledBuffer.getLength()) + 1;
      // // Sets the specified LED to the RGB values for red
      // m_ledBuffer.setRGB(i, red, green, blue);

      // int j = (num % m_ledBuffer.getLength());
      // // Sets the specified LED to the RGB values for red
      // m_ledBuffer.setRGB(j, 0, 0, 0);

    }

    m_led.setData(m_ledBuffer);
    m_led.start();

    counter++;
  }

  public void blink(int red, int green, int blue, int blinksPerSecond) {
    int num = (int) (2 * (blinksPerSecond) * m_timer.get());

    if (num % 2 == 0) {
      for (int i = 0; i < (m_ledBuffer.getLength()); i++) {
        // Sets the specified LED to the RGB values for red
        // Maybe actually rbg? code is weird
        m_ledBuffer.setRGB(i, red, green, blue);
      }
    } else {
      for (int i = 0; i < (m_ledBuffer.getLength()); i++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, 0, 0, 0);
      }
    }
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void rainbow() {

    int funTimer = (int) (32 * m_timer.get());

    X = +(funTimer * 8) % 1536;

    if (X >= 0 && X <= 255) {

      r = 256;
      g = n * X;
      b = 0;

    }

    if (X >= 256 && X <= 511) {

      r = -(n * (X - 256)) + 256;
      g = 256;
      b = 0;

    }

    if (X >= 512 && X <= 767) {

      r = 0;
      g = 256;
      b = n * (X - 512);

    }

    if (X >= 768 && X <= 1023) {

      r = 0;
      g = -(n * (X - 768)) + 256;
      b = 256;

    }

    if (X >= 1024 && X <= 1279) {

      r = n * (X - 1024);
      g = 0;
      b = 256;

    }

    if (X >= 1280 && X <= 1535) {

      r = 256;
      g = 0;
      b = -(n * (X - 1280)) + 256;

    }

    if (r == 256) {
      rr = 255;
    } else {
      rr = r;
    }
    if (g == 256) {
      gg = 255;
    } else {
      gg = g;
    }
    if (b == 256) {
      bb = 255;
    } else {
      bb = b;
    }

    for (var bg = 0; bg < (m_ledBuffer.getLength()); bg++) {

      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(bg, rr, gg, bb);

    }

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void alt() {
    int num = (int) (8 * m_timer.get());

    if (num % 3 == 0) {
      for (int e = 0; e < (m_ledBuffer.getLength()); e += 3) {
        m_ledBuffer.setRGB(e, 200, 0, 0);
      }
      for (int o = 1; o < (m_ledBuffer.getLength()); o += 3) {
        m_ledBuffer.setRGB(o, 0, 200, 0);
      }
      for (int e = 2; e < (m_ledBuffer.getLength()); e += 3) {
        m_ledBuffer.setRGB(e, 0, 0, 200);
      }

    } else if (num % 3 == 1) {
      for (int e = 0; e < (m_ledBuffer.getLength()); e += 3) {
        m_ledBuffer.setRGB(e, 0, 200, 0);
      }
      for (int o = 1; o < (m_ledBuffer.getLength()); o += 3) {
        m_ledBuffer.setRGB(o, 0, 0, 200);
      }
      for (int e = 2; e < (m_ledBuffer.getLength()); e += 3) {
        m_ledBuffer.setRGB(e, 200, 0, 0);
      }

    } else if (num % 3 == 2) {
      for (int e = 0; e < (m_ledBuffer.getLength()); e += 3) {
        m_ledBuffer.setRGB(e, 0, 0, 200);
      }
      for (int o = 1; o < (m_ledBuffer.getLength()); o += 3) {
        m_ledBuffer.setRGB(o, 200, 0, 0);
      }
      for (int e = 2; e < (m_ledBuffer.getLength()); e += 3) {
        m_ledBuffer.setRGB(e, 0, 200, 0);
      }

    }

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void teamNum() {
    // if (counter == 120) {
    // m_timer.reset();
    // m_timer.start();
    // counter = 0;
    // solid(0, 0, 0);
    // }
    int funTimer = (int) (32 * m_timer.get());

    X = +(funTimer * 8) % 1536;

    if (X >= 0 && X <= 255) {

      r = 256;
      g = n * X;
      b = 0;

    }

    if (X >= 256 && X <= 511) {

      r = -(n * (X - 256)) + 256;
      g = 256;
      b = 0;

    }

    if (X >= 512 && X <= 767) {

      r = 0;
      g = 256;
      b = n * (X - 512);

    }

    if (X >= 768 && X <= 1023) {

      r = 0;
      g = -(n * (X - 768)) + 256;
      b = 256;

    }

    if (X >= 1024 && X <= 1279) {

      r = n * (X - 1024);
      g = 0;
      b = 256;

    }

    if (X >= 1280 && X <= 1535) {

      r = 256;
      g = 0;
      b = -(n * (X - 1280)) + 256;

    }

    if (r == 256) {
      rr = 255;
    } else {
      rr = r;
    }
    if (g == 256) {
      gg = 255;
    } else {
      gg = g;
    }
    if (b == 256) {
      bb = 255;
    } else {
      bb = b;
    }

    int position = (m_ledBuffer.getLength() - 11) / 2;

    for (int i = 0; i < (m_ledBuffer.getLength()); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 50, 200);
    }

    int z = position - 3;
    // Sets the specified LED to the RGB values for red
    m_ledBuffer.setRGB(z, rr, gg, bb);

    int o = position + 13;
    // Sets the specified LED to the RGB values for red
    m_ledBuffer.setRGB(o, rr, gg, bb);

    int a = 0 + position;
    // Sets the specified LED to the RGB values for red
    m_ledBuffer.setRGB(a, 100, 100, 100);

    int b = 1 + position;
    // Sets the specified LED to the RGB values for red
    m_ledBuffer.setRGB(b, 100, 100, 100);

    int c = 2 + position;
    // Sets the specified LED to the RGB values for red
    m_ledBuffer.setRGB(c, 0, 0, 0);

    int d = 3 + position;
    // Sets the specified LED to the RGB values for red
    m_ledBuffer.setRGB(d, 100, 100, 100);

    int e = 4 + position;
    // Sets the specified LED to the RGB values for red
    m_ledBuffer.setRGB(e, 0, 0, 0);

    int f = 5 + position;
    // Sets the specified LED to the RGB values for red
    m_ledBuffer.setRGB(f, 100, 100, 100);

    int g = 6 + position;
    // Sets the specified LED to the RGB values for red
    m_ledBuffer.setRGB(g, 0, 0, 0);

    int h = 7 + position;
    // Sets the specified LED to the RGB values for red
    m_ledBuffer.setRGB(h, 100, 100, 100);

    int i = 8 + position;
    // Sets the specified LED to the RGB values for red
    m_ledBuffer.setRGB(i, 100, 100, 100);

    int j = 9 + position;
    // Sets the specified LED to the RGB values for red
    m_ledBuffer.setRGB(j, 100, 100, 100);

    int k = 10 + position;
    // Sets the specified LED to the RGB values for red
    m_ledBuffer.setRGB(k, 0, 0, 0);

    // int m = (num % m_ledBuffer.getLength());
    //// Sets the specified LED to the RGB values for red
    // m_ledBuffer.setRGB(m, 0, 50, 200);

    m_led.setData(m_ledBuffer);
    m_led.start();
    counter++;

  }

  // public void idleMode() {
  //
  // if (ShooterSubsystem.isShooterToSpeedAndNotDisabled)
  //
  // if (IndexerSubsystem.bottomBeamBreak.get() == true &&
  // IndexerSubsystem.topBeamBreak.get() == true) {
  //
  // tripleOrbit(100, 100, 100, 200, 100, 0, 2);
  //
  // } else if (IndexerSubsystem.bottomBeamBreak.get() == false &&
  // IndexerSubsystem.topBeamBreak.get() == false) {
  //
  // tripleOrbit(200, 200, 200, 0, 0, 0, 2);
  //
  // } else {
  //
  // tripleOrbit(200, 100, 0, 0, 0, 0, 2);
  //
  // }

  // }

}