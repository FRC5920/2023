////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023 FIRST and other WPILib contributors.
// http://github.com/FRC5920
// Open Source Software; you can modify and/or share it under the terms of the
// license given in WPILib-License.md in the root directory of this project.
////////////////////////////////////////////////////////////////////////////////

/*-----------------------------------------------------------------------------\
|                                                                              |
|                       ================================                       |
|                       **    TEAM 5920 - Vikotics    **                       |
|                       ================================                       |
|                                                                              |
|                            °        #°                                       |
|                            *O       °@o                                      |
|                            O@ °o@@#° o@@                                     |
|                           #@@@@@@@@@@@@@@                                    |
|                           @@@@@@@@@@@@@@@                                    |
|                           @@@@@@@@@@@@@@°                                    |
|                             #@@@@@@@@@@@@@O....   .                          |
|                             o@@@@@@@@@@@@@@@@@@@@@o                          |
|                             O@@@@@@@@@@@@@@@@@@@#°                    *      |
|                             O@@@@@@@@@@@@@@@@@@@@@#O                O@@    O |
|                            .@@@@@@@@°@@@@@@@@@@@@@@@@#            °@@@    °@@|
|                            #@@O°°°°  @@@@@@@@@@@@@@@@@@°          @@@#*   @@@|
|                         .#@@@@@  o#oo@@@@@@@@@@@@@@@@@@@@@.       O@@@@@@@@@@|
|                        o@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@°     @@@@@@@@@°|
|                        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@   .@@@@@o°   |
|          °***          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@  @@@@@o     |
|     o#@@@@@@@@@@@@.   *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@o@@@@@@      |
|OOo°@@@@@@@@@@@@O°#@#   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@       |
|@@@@@@@@@@@@@@@@    o°  .@@@@@@@@@@@@@@@@@@@@@@@@#*@@@@@@@@@@@@@@@@@@@@       |
|@@@@@@@@@@@@@@@*         O@@@@@@@@@@@@@@@@@@@@@@@   °@@@@@@@@@@@@@@@@@@o      |
|@@@@#@@@@@@@@@            @@@@@@@@@@@@@@@@@@@@@@       .*@@@@@@@@@@@@@@.      |
|@@@°      @@@@O           @@@@@@@@@@@@@@@@@@@@o           °@@@@@@@@@@@o       |
|          @@@@@          .@@@@@@@@@@@@@@@@@@@*               O@@@@@@@*        |
|           @@@@@        o@@@@@@@@@@@@@@@@@@@@.               #@@@@@O          |
|           *@@@@@@@*  o@@@@@@@@@@@@@@@@@@@@@@°              o@@@@@            |
|           @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.              @@@@@#            |
|          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@O             #@@@@@             |
|          .@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#           .@@@@@°             |
|           @@@@@@@@@@O*    @@@@@@@@@@@@@@@@@@@@@°         °O@@@°              |
|            °O@@@@@@       @@@@@@@@@@@@@@@@@@@@@@@                            |
|              o@@@@@°      @@@@@@@@@@@@@@@@@@@@@@@@                           |
|               @@@@@@.     @@@@@@@@@@@@@@@@@@@@@@@@@o                         |
|                @@@@@@*    @@@@@@@@@@@@@@@@@@@@@@@@@@                         |
|                o@@@@@@.  o@@@@@@@@@@@@@@@@@@@@@@@@@@@                        |
|                 #@@@@@@  *@@@@@@@@@@@@@@@@@@@@@@@@@@@@                       |
|                  °***    @@@@@@@@@@@@@@@@@@@@@@@@@@@@@O                      |
|                         .OOOOOOOOOOOOOOOOOOOOOOOOOOOOOO                      |
\-----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utility.LEDPattern;

public class LEDs extends SubsystemBase {
  //////////////////////////////////
  /// *** CONSTANTS ***
  //////////////////////////////////

  // TODO: The following LEDStrip objects need to be configured with
  //       the correct start index and count
  private static final LEDStrip kLowerLeftStrip = new LEDStrip(0, 24);
  private static final LEDStrip kUpperLeftStrip = new LEDStrip(25, 49);
  private static final LEDStrip kLowerRightStrip = new LEDStrip(50, 74);
  private static final LEDStrip kUpperRightStrip = new LEDStrip(75, 99);

  // private static final Color8Bit kOff = new Color8Bit(0, 0, 0);
  // private static final Color8Bit kGreen = new Color8Bit(0, 255, 0);
  public static final Color8Bit kRed = new Color8Bit(255, 0, 0);
  public static final Color8Bit kBlue = new Color8Bit(0, 0, 255);
  public static final Color8Bit kYellow = new Color8Bit(255, 120, 0);
  public static final Color8Bit kWhite = new Color8Bit(255, 255, 255);
  public static final Color8Bit kPurple = new Color8Bit(255, 0, 255);

  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;
  private static int m_rainbowFirstPixelHue;
  private static int m_pixeloffset;

  // private static int l_shootColor;
  // private static int currentColor;
  static int iPos = 0;
  private LEDPattern m_lowerLeftSweep;
  private LEDPattern m_upperLeftSweep;
  private LEDPattern m_lowerRightSweep;
  private LEDPattern m_upperRightSweep;

  /////////////////////////////////////////////////////////////////////////////
  /** Helper class used to store a description of a strip of addressable LEDs */
  private static class LEDStrip {
    public final int startIndex, numLEDs;

    /**
     * Creates an instance of the object
     *
     * @param start Start address (index) of the strip
     * @param count Number of LED's in the strip
     */
    public LEDStrip(int start, int count) {
      startIndex = start;
      numLEDs = count;
    }
  }

  /** Creates a new LEDs. */
  public LEDs() {
    m_led = new AddressableLED(9);
    m_ledBuffer = new AddressableLEDBuffer(100);
    m_lowerLeftSweep = new LEDPattern(kLowerLeftStrip.startIndex, kLowerLeftStrip.numLEDs, kYellow);
    m_upperLeftSweep = new LEDPattern(kUpperLeftStrip.startIndex, kUpperLeftStrip.numLEDs, kYellow);
    m_lowerRightSweep =
        new LEDPattern(kLowerRightStrip.startIndex, kLowerRightStrip.numLEDs, kYellow);
    m_upperRightSweep =
        new LEDPattern(kUpperRightStrip.startIndex, kUpperRightStrip.numLEDs, kYellow);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    m_pixeloffset = 0;
    // updateAllianceColor();
  }

  /////////////////////////////////////////////////////////////////////////////
  /** Updates the active color */
  public void updateColor(Color8Bit desiredColor) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, kYellow);
    }
    // m_lowerLeftSweep.setColor(desiredColor);
    // m_upperLeftSweep.setColor(desiredColor);
    // m_lowerRightSweep.setColor(desiredColor);
    // m_upperRightSweep.setColor(desiredColor);
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
    // All_LEDRainbow();
    AllWhite();

    // Apply the LED buffer states to the LED strip
    m_led.setData(m_ledBuffer);
  }

  private void AllYellow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, kYellow);
    }
  }

  private void AllWhite() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, kWhite);
    }
  }
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Make LED's a rainbow pattern
   *
   * @note Unicorns and sprinkles will be added in a future update...
   */
  private void All_LEDRainbow() {
    // --- make a rainbow pattern on LEDs ---//
    int ShowLEDs = m_ledBuffer.getLength();
    for (var i = 0; i < ShowLEDs; i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }

    m_rainbowFirstPixelHue += 3;
    m_rainbowFirstPixelHue %= 180;
  }
}
