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
package frc.robot.subsystems.Pneumatics;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {

  PneumaticHub m_PHub = new PneumaticHub(Constants.PneumaticsConstants.kPDHCAN);
  /** Creates a new Pneumatics. */
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

  boolean enabled = phCompressor.isEnabled();
  boolean pressureSwitch = phCompressor.getPressureSwitchValue();
  // double currentCompressor = phCompressor.getCompressorCurrent();
  // private final DoubleSolenoid m_PWrist =
  //    new DoubleSolenoid(
  //        PneumaticsModuleType.REVPH,
  //        Constants.PneumaticsConstants.kArmLeftRotatorPort,
  //        Constants.PneumaticsConstants.kArmRightRotatorPort);
  private DoubleSolenoid m_PWrist =
      m_PHub.makeDoubleSolenoid(
          Constants.PneumaticsConstants.kArmLeftRotatorPort,
          Constants.PneumaticsConstants.kArmRightRotatorPort);

  public Pneumatics() {
    phCompressor.enableDigital();
    m_PWrist.set(kOff);
  }

  // Sets the desired wrist position
  public void setWristPosition(WristPosition pos) {
    m_PWrist.set(pos.value);
  }

  // Returns the present wrist position
  public WristPosition getWristPosition() {
    return (m_PWrist.get() == kForward) ? WristPosition.Normal : WristPosition.Inverted;
  }

  public void toggleWristPosition() {
    Pneumatics.WristPosition position =
        getWristPosition() == WristPosition.Normal ? WristPosition.Inverted : WristPosition.Normal;
    setWristPosition(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  /** Wrist positions */
  public enum WristPosition {
    /** Normal position */
    Normal(DoubleSolenoid.Value.kForward),

    /** Wrist inverted */
    Inverted(DoubleSolenoid.Value.kReverse);

    public DoubleSolenoid.Value value;

    private WristPosition(DoubleSolenoid.Value val) {
      value = val;
    }

    /** Get the human-readable name of the position */
    @Override
    public String toString() {
      return this.name();
    }
  };
}
