// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.lib.utility.LEDPattern;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDs extends SubsystemBase {
   //////////////////////////////////
  /// *** CONSTANTS ***
  //////////////////////////////////

  // TODO: The following LEDStrip objects need to be configured with
  //       the correct start index and count
  private static final LEDStrip kLowerLeftStrip = new LEDStrip(0, 33);
  private static final LEDStrip kUpperLeftStrip = new LEDStrip(33, 33);
  private static final LEDStrip kLowerRightStrip = new LEDStrip(66, 33);
  private static final LEDStrip kUpperRightStrip = new LEDStrip(99, 33);

  //private static final Color8Bit kOff = new Color8Bit(0, 0, 0);
  //private static final Color8Bit kGreen = new Color8Bit(0, 255, 0);
  public static final Color8Bit kRed = new Color8Bit(255, 0, 0);
  public static final Color8Bit kBlue = new Color8Bit(0, 0, 255);
  public static final Color8Bit kYellow = new Color8Bit(255, 120, 0);
  public static final Color8Bit kWhite = new Color8Bit(255, 255, 255);
  public static final Color8Bit kPurple = new Color8Bit(255, 0, 255);

  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;
  private static int m_rainbowFirstPixelHue;
  private static int m_pixeloffset;

  //private static int l_shootColor;
  //private static int currentColor;
  static int iPos=0;
  private LEDPattern m_lowerLeftSweep;
  private LEDPattern m_upperLeftSweep;
  private LEDPattern m_lowerRightSweep;
  private LEDPattern m_upperRightSweep;

   /////////////////////////////////////////////////////////////////////////////
  /** Helper class used to store a description of a strip of addressable
   *  LEDs
   */
  private static class LEDStrip {
    public final int startIndex, numLEDs;

    /** Creates an instance of the object
     * @param start   Start address (index) of the strip
     * @param count   Number of LED's in the strip
     */
    public LEDStrip(int start, int count) {
      startIndex = start;
      numLEDs = count;
    }
  }



  /** Creates a new LEDs. */
  public LEDs() {
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(132);
    m_lowerLeftSweep = new LEDPattern(kLowerLeftStrip.startIndex, kLowerLeftStrip.numLEDs, kYellow);
    m_upperLeftSweep = new LEDPattern(kUpperLeftStrip.startIndex, kUpperLeftStrip.numLEDs, kYellow);
    m_lowerRightSweep = new LEDPattern(kLowerRightStrip.startIndex, kLowerRightStrip.numLEDs, kYellow);
    m_upperRightSweep = new LEDPattern(kUpperRightStrip.startIndex, kUpperRightStrip.numLEDs, kYellow);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    m_pixeloffset = 0;
    //updateAllianceColor();
  }

    /////////////////////////////////////////////////////////////////////////////
  /** Updates the active color */
  public void updateColor(Color8Bit desiredColor) {
    m_lowerLeftSweep.setColor(desiredColor);
    m_upperLeftSweep.setColor(desiredColor);
    m_lowerRightSweep.setColor(desiredColor);
    m_upperRightSweep.setColor(desiredColor);
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Resets the LED sweeps */
  public void resetSweep() {
    m_lowerLeftSweep.reset();
    m_upperLeftSweep.reset();
    m_lowerRightSweep.reset();
    m_upperRightSweep.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
