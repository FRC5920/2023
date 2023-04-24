////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2022 FIRST and other WPILib contributors.
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
package frc.lib.Joystick;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

/**
 * Base class for a subsystem that manages two Xbox controllers for driving the robot manually in
 * teleop mode
 */
public abstract class JoystickSubsystemBase extends SubsystemBase {

  // ---------- Default joystick processing configuration -----------------
  /** Default sensitivity to use for the left stick on the driver controller */
  public static final double kDriverLeftStickSensitivity = 0.5;
  /** Default deadbands applied to the left stick on the driver controller */
  public static final double kDriverLeftStickDeadbands[] = {0.1, 0.95};
  /** Default sensitivity to use for the right stick on the driver controller */
  public static final double kDriverRightStickSensitivity = 0.5;
  /** Default deadbands applied to the right stick on the driver controller */
  public static final double kDriverRightStickDeadbands[] = {0.1, 0.95};
  /** Default sensitivity to use for triggers on the driver controller */
  public static final double kDriverTriggerSensitivity = 1.0;
  /** Default deadbands applied to triggers on the driver controller */
  public static final double kDriverTriggerDeadbands[] = {0.1, 0.95};

  /** Default sensitivity to use for the left stick on the operator controller */
  public static final double kOperatorLeftStickSensitivity = 0.3;
  /** Default deadbands applied to the left stick on the operator controller */
  public static final double kOperatorLeftStickDeadbands[] = {0.1, 0.95};
  /** Default sensitivity to use for the right stick on the operator controller */
  public static final double kOperatorRightStickSensitivity = 0.3;
  /** Default deadbands applied to the right stick on the operator controller */
  public static final double kOperatorRightStickDeadbands[] = {0.1, 0.95};
  /** Default sensitivity to use for triggers on the operator controller */
  public static final double kOperatorTriggerSensitivity = 1.0;
  /** Default deadbands applied to triggers on the operator controller */
  public static final double kOperatorTriggerDeadbands[] = {0.1, 0.95};

  /** Joystick port used to communicate with the Driver Xbox controller */
  private static final int kDriverControllerPort = 0;

  /** Joystick port used to communicate with the Operator Xbox controller */
  private static final int kOperatorControllerPort = 1;

  /** Empty command used as a placeholder for button mapping */
  public static final InstantCommand kDoNothing = new InstantCommand();

  /** Xbox controller used by the robot driver */
  private ProcessedXboxController m_driverController;

  /** Xbox controller used by the robot operator */
  private ProcessedXboxController m_operatorController;

  /** true when the driver controller is enabled; else false to disable it */
  private final boolean m_driverControllerIsEnabled;

  /** true when the operator controller is enabled; else false to disable it */
  private final boolean m_operatorControllerIsEnabled;

  /**
   * Creates an instance of the subsystem and sets up joystick axis processing
   *
   * @param enableDriverController true to enable the driver Xbox controller; false to disable
   * @param enableOperatorController true to enable the operator Xbox controller; false to disable
   */
  public JoystickSubsystemBase(boolean enableDriverController, boolean enableOperatorController) {
    m_driverControllerIsEnabled = enableDriverController;
    m_operatorControllerIsEnabled = enableOperatorController;
    m_driverController = new ProcessedXboxController(kDriverControllerPort);
    m_operatorController = new ProcessedXboxController(kOperatorControllerPort);

    if (m_driverControllerIsEnabled) {
      configureAxisProcessing(
          m_driverController,
          new AxisProcChain.Config(kDriverLeftStickSensitivity, kDriverLeftStickDeadbands),
          new AxisProcChain.Config(kDriverRightStickSensitivity, kDriverRightStickDeadbands),
          new AxisProcChain.Config(kDriverTriggerSensitivity, kDriverTriggerDeadbands));
    }

    if (m_operatorControllerIsEnabled) {
      configureAxisProcessing(
          m_operatorController,
          new AxisProcChain.Config(kOperatorLeftStickSensitivity, kOperatorLeftStickDeadbands),
          new AxisProcChain.Config(kOperatorRightStickSensitivity, kOperatorRightStickDeadbands),
          new AxisProcChain.Config(kOperatorTriggerSensitivity, kOperatorTriggerDeadbands));
    }
  }

  /**
   * Sets up commands to be executed when a button is pressed and when it is pressed in combination
   * with a second button.
   *
   * @param targetButton A joystick button to map
   * @param comboButton A second joystick button that can be pressed to modify the command executed
   *     by targetButton
   * @param soloCommand Command to run when targetButton is pressed but comboButton is not
   * @param comboCommand Command to run when targetButton is pressed while comboButton is held
   */
  public static void setupButtonCombo(
      Trigger targetButton,
      Trigger comboButton,
      CommandBase soloCommand,
      CommandBase comboCommand) {
    CommandBase noComboCommand = soloCommand.unless(comboButton::getAsBoolean);
    CommandBase withComboCommand = comboCommand.unless(() -> !comboButton.getAsBoolean());
    targetButton.onTrue(
        Commands.either(withComboCommand, noComboCommand, comboButton::getAsBoolean));
  }

  /**
   * Derived classes must override this method to define button bindings.
   *
   * @param botContainer Object providing access to robot subsystems
   */
  public abstract void configureButtonBindings(RobotContainer botContainer);

  /** Returns the subsystem's driver controller */
  public ProcessedXboxController getDriverController() {
    return m_driverController;
  }

  /** Returns the subsystem's operator controller */
  public ProcessedXboxController getOperatorController() {
    return m_operatorController;
  }

  /**
   * Sets up axis processing for an Xbox controller
   *
   * @param controller The Xbox controller to configure
   * @param leftStickConfig Axis processing configuration for the controller's left stick
   * @param rightStickConfig Axis processing configuration for the controller's right stick
   * @param triggerConfig Axis processing configuration for the controller's triggers
   */
  protected static void configureAxisProcessing(
      ProcessedXboxController controller,
      AxisProcChain.Config leftStickConfig,
      AxisProcChain.Config rightStickConfig,
      AxisProcChain.Config triggerConfig) {
    // Configure left stick axis processing
    controller.getStickProcessing(XboxController.Axis.kLeftX).configure(leftStickConfig);
    controller.getStickProcessing(XboxController.Axis.kLeftY).configure(leftStickConfig);

    // Configure right stick axis processing
    controller.getStickProcessing(XboxController.Axis.kRightX).configure(rightStickConfig);
    controller.getStickProcessing(XboxController.Axis.kRightY).configure(rightStickConfig);

    // Configure trigger axis processing
    controller.getTriggerProcessing(XboxController.Axis.kLeftTrigger).configure(triggerConfig);
    controller.getTriggerProcessing(XboxController.Axis.kRightTrigger).configure(triggerConfig);
  }
}
