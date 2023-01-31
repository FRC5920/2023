////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023 FIRST and other WPILib contributors.
// http://github.com/FRC5920
// Open Source Software; you can modify and/or share it under the terms of the
// license given in WPILib-License.md in the root directory of this project.
////////////////////////////////////////////////////////////////////////////////

/*-----------------------------------------------------------------------------\
|                                                                              |
|                       ================================                       |
|                       **    TEAM 5290 - Vikotics    **                       |
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
package frc.lib.Joystick;

import java.util.ArrayList;

/**
 * AxisProcChain provides a wrapper for a chain of processing that can be applied to values from a
 * controller axis. Built-in processing includes feathering and deadbanding. Additional processing
 * can be applied by extending the class
 */
public class AxisProcChain implements AxisProcessor {

  /** kFeathering gives the index of the feathering processor in the chain */
  private static final int kFeathering = 0;
  /** kDeadbanding gives the index of the deadbanding processor in the chain */
  private static final int kDeadbanding = 1;

  /** Configuration for an AxisProcChain */
  public static class Config {
    /** Sensitivity of the joystick as a value between 0.0 and 1.0 */
    public final double sens;

    /** Lower deadband applied to axis values as a value between 0.0 and 1.0 */
    public final double dbLower;

    /** Upper deadband applied to axis values as a value between 0.0 and 1.0 */
    public final double dbUpper;

    /**
     * Initializes axis processing configuration
     *
     * @param sensitivity Sensitivity of axis values as a value between 0.0 and 1.0
     * @param deadband Array of deadband values (0.0 to 1.0) with indices {0=Lower, 1=upper}
     * @param upperDeadband Upper deadband applied to axis values as a value between 0.0 and 1.0
     */
    public Config(double sensitivity, double deadband[]) {
      sens = sensitivity;
      dbLower = deadband[0];
      dbUpper = deadband[1];
    }
  }

  /** Processing applied to incoming Axis values */
  private ArrayList<AxisProcessor> m_procChain;

  /**
   * Creates the processor with a specified configuration
   *
   * @param config Axis processing configuration to use
   */
  AxisProcChain(Config config) {
    m_procChain.add(new AxisFeathering(config.sens));
    m_procChain.add(new AxisDeadband(config.dbLower, config.dbUpper));
  }

  /** Process a value from a controller axis */
  @Override
  public double process(double axisValue) {
    double result = axisValue;
    for (AxisProcessor proc : m_procChain) {
      result = proc.process(result);
    }
    return result;
  }

  /** Configures feathering and deadband values in the processing chain */
  public void configure(Config config) {
    feathering().setSensitivity(config.sens);
    deadbanding().setLower(config.dbLower).setUpper(config.dbUpper);
  }

  /** Returns a mutable reference to the feathering processor in the chain */
  public AxisFeathering feathering() {
    return (AxisFeathering) m_procChain.get(kFeathering);
  }

  /** Returns a mutable reference to the deadbanding processor in the chain */
  public AxisDeadband deadbanding() {
    return (AxisDeadband) m_procChain.get(kDeadbanding);
  }

  /** Derived classes can use this to access an axis processor in the chain */
  protected AxisProcessor getProc(int index) {
    return m_procChain.get(index);
  }

  /**
   * Derived classes can use this method to append an axis processor to the end of the processing
   * chain.
   */
  protected void appendProc(AxisProcessor proc) {
    m_procChain.add(proc);
  }
}
