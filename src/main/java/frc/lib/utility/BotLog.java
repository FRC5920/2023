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
package frc.lib.utility;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Consumer;

/** Add your docs here. */
public class BotLog {
  public static BotLog instance = new BotLog();

  /** The maximum logging level */
  public static MessageType verbosity = MessageType.Debug;

  /** Consumer that receives non-error log messages */
  public Consumer<String> logSink = (str) -> System.out.println(str);
  /** Consumer that receives error log messages */
  public Consumer<String> errorSink = (str) -> System.err.println(str);

  /** Prints an error message to the log in runtime and simulation mode */
  public void Error(String message) {
    Print(MessageType.Error, Scope.Runtime, message);
  }
  /** Prints a warning message to the log in runtime and simulation mode */
  public void Warning(String message) {
    Print(MessageType.Warning, Scope.Runtime, message);
  }
  /** Prints an info message to the log in runtime and simulation mode */
  public void Info(String message) {
    Print(MessageType.Warning, Scope.Runtime, message);
  }
  /** Prints a Debug message to the log in runtime and simulation mode */
  public void Debug(String message) {
    Print(MessageType.Debug, Scope.Runtime, message);
  }

  /** Prints an error message to the log in simulation mode only */
  public void SimError(String message) {
    Print(MessageType.Error, Scope.SimulationOnly, message);
  }
  /** Prints a warning message to the log in simulation mode only */
  public void SimWarning(String message) {
    Print(MessageType.Warning, Scope.SimulationOnly, message);
  }
  /** Prints an info message to the log in simulation mode only */
  public void SimInfo(String message) {
    Print(MessageType.Warning, Scope.SimulationOnly, message);
  }
  /** Prints a Debug message to the log in simulation mode only */
  public void SimDebug(String message) {
    Print(MessageType.Debug, Scope.SimulationOnly, message);
  }

  /** Prints an error message to the log in runtime and simulation mode */
  public void Errorf(String format, Object... args) {
    Format(MessageType.Error, Scope.Runtime, format, args);
  }
  /** Prints a warning message to the log in runtime and simulation mode */
  public void Warningf(String format, Object... args) {
    Format(MessageType.Warning, Scope.Runtime, format, args);
  }
  /** Prints an info message to the log in runtime and simulation mode */
  public void Infof(String format, Object... args) {
    Format(MessageType.Warning, Scope.Runtime, format, args);
  }
  /** Prints a Debug message to the log in runtime and simulation mode */
  public void Debugf(String format, Object... args) {
    Format(MessageType.Debug, Scope.Runtime, format, args);
  }

  /** Prints an error message to the log in runtime and simulation mode */
  public void SimErrorf(String format, Object... args) {
    Format(MessageType.Error, Scope.Runtime, format, args);
  }
  /** Prints a warning message to the log in runtime and simulation mode */
  public void SimWarningf(String format, Object... args) {
    Format(MessageType.Warning, Scope.Runtime, format, args);
  }
  /** Prints an info message to the log in runtime and simulation mode */
  public void SimInfof(String format, Object... args) {
    Format(MessageType.Warning, Scope.Runtime, format, args);
  }
  /** Prints a Debug message to the log in runtime and simulation mode */
  public void SimDebugf(String format, Object... args) {
    Format(MessageType.Debug, Scope.Runtime, format, args);
  }

  /** Formats and prints a log message using a given format string and arguments */
  public void Format(MessageType type, Scope scope, String format, Object... args) {
    Print(type, scope, String.format(format, args));
  }

  /** Prints a log message */
  public void Print(MessageType type, Scope scope, String message) {
    // Handle verbosity and simulation-only messages
    if ((type.id > verbosity.id) || ((scope == Scope.SimulationOnly) && RobotBase.isReal())) {
      return;
    }

    // Print error messages to error output; everything else to regular log output
    if (type == MessageType.Error) {
      errorSink.accept(message);
    } else {
      logSink.accept(message);
    }
  }

  public enum MessageType {
    Error(0),
    Warning(1),
    Info(2),
    Debug(3);

    public final int id;

    private MessageType(int levelId) {
      id = levelId;
    }
  }

  public enum Scope {
    Runtime(1), // Log message appears in runtime and simulation logs
    SimulationOnly(2); // Log message appears only in simulation logs

    public final int id;

    private Scope(int scopeId) {
      id = scopeId;
    }
  }

  /** BotLogPrinter is a command that prints to the BotLog in its init phase */
  public static class PrintCommand extends CommandBase {
    protected MessageType m_type;
    protected Scope m_scope;
    protected final String m_message;

    /** Creates a command that prints a debug message in runtime and simulation */
    public PrintCommand(String msg) {
      this(MessageType.Debug, msg);
    }

    /** Creates a command that prints a debug message in runtime and simulation */
    public PrintCommand(String format, Object... args) {
      this(MessageType.Debug, String.format(format, args));
    }

    /** Creates a command that prints a message in runtime and simulation */
    public PrintCommand(MessageType type, String msg) {
      m_type = type;
      m_message = msg;
    }

    /** Creates a command that prints a message in runtime and simulation */
    public PrintCommand(MessageType type, String format, Object... args) {
      m_type = type;
      m_message = String.format(format, args);
    }

    @Override
    public void initialize() {
      BotLog.instance.Print(m_type, Scope.Runtime, m_message);
    }

    @Override
    public boolean isFinished() {
      return true;
    }
  }

  /**
   * BotLogPrinter is a command that prints to the BotLog in Simulation mode during its init phase
   */
  public static class SimulationPrinter extends PrintCommand {

    /** Creates a command that prints a debug message in runtime and simulation */
    public SimulationPrinter(String msg) {
      this(MessageType.Debug, msg);
    }

    /** Creates a command that prints a debug message in runtime and simulation */
    public SimulationPrinter(String format, Object... args) {
      this(MessageType.Debug, String.format(format, args));
    }

    /** Creates a command that prints a message in runtime and simulation */
    public SimulationPrinter(MessageType type, String msg) {
      super(type, msg);
      m_scope = Scope.SimulationOnly;
    }

    /** Creates a command that prints a message in runtime and simulation */
    public SimulationPrinter(MessageType type, String format, Object... args) {
      super(type, String.format(format, args));
      m_scope = Scope.SimulationOnly;
    }

    @Override
    public void initialize() {
      BotLog.instance.Print(m_type, Scope.Runtime, m_message);
    }

    @Override
    public boolean isFinished() {
      return true;
    }
  }
}
