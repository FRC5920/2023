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
package frc.lib.utility.BotLogger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Consumer;

/** Add your docs here. */
public class BotLog {
  public static BotLog globalInstance = new BotLog();

  /** An enumeration of log message types */
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

  /** The maximum logging level */
  public static MessageType verbosity = MessageType.Debug;

  /** Consumer that receives non-error log messages */
  public Consumer<String> logSink = (str) -> System.out.println(str);
  /** Consumer that receives error log messages */
  public Consumer<String> errorSink = (str) -> System.err.println(str);

  /** Prints an error message to the log in runtime and simulation mode */
  public static void Error(String message) {
    globalInstance.Print(MessageType.Error, Scope.Runtime, message);
  }
  /** Prints a warning message to the log in runtime and simulation mode */
  public static void Warning(String message) {
    globalInstance.Print(MessageType.Warning, Scope.Runtime, message);
  }
  /** Prints an info message to the log in runtime and simulation mode */
  public static void Info(String message) {
    globalInstance.Print(MessageType.Warning, Scope.Runtime, message);
  }
  /** Prints a Debug message to the log in runtime and simulation mode */
  public static void Debug(String message) {
    globalInstance.Print(MessageType.Debug, Scope.Runtime, message);
  }

  /** Prints an error message to the log in simulation mode only */
  public static void SimError(String message) {
    globalInstance.Print(MessageType.Error, Scope.SimulationOnly, message);
  }
  /** Prints a warning message to the log in simulation mode only */
  public static void SimWarning(String message) {
    globalInstance.Print(MessageType.Warning, Scope.SimulationOnly, message);
  }
  /** Prints an info message to the log in simulation mode only */
  public static void SimInfo(String message) {
    globalInstance.Print(MessageType.Warning, Scope.SimulationOnly, message);
  }
  /** Prints a Debug message to the log in simulation mode only */
  public static void SimDebug(String message) {
    globalInstance.Print(MessageType.Debug, Scope.SimulationOnly, message);
  }

  /** Prints an error message to the log in runtime and simulation mode */
  public static void Errorf(String format, Object... args) {
    globalInstance.Format(MessageType.Error, Scope.Runtime, format, args);
  }
  /** Prints a warning message to the log in runtime and simulation mode */
  public static void Warningf(String format, Object... args) {
    globalInstance.Format(MessageType.Warning, Scope.Runtime, format, args);
  }
  /** Prints an info message to the log in runtime and simulation mode */
  public static void Infof(String format, Object... args) {
    globalInstance.Format(MessageType.Warning, Scope.Runtime, format, args);
  }
  /** Prints a Debug message to the log in runtime and simulation mode */
  public static void Debugf(String format, Object... args) {
    globalInstance.Format(MessageType.Debug, Scope.Runtime, format, args);
  }

  /** Prints an error message to the log in runtime and simulation mode */
  public static void SimErrorf(String format, Object... args) {
    globalInstance.Format(MessageType.Error, Scope.Runtime, format, args);
  }
  /** Prints a warning message to the log in runtime and simulation mode */
  public static void SimWarningf(String format, Object... args) {
    globalInstance.Format(MessageType.Warning, Scope.Runtime, format, args);
  }
  /** Prints an info message to the log in runtime and simulation mode */
  public static void SimInfof(String format, Object... args) {
    globalInstance.Format(MessageType.Warning, Scope.Runtime, format, args);
  }
  /** Prints a Debug message to the log in runtime and simulation mode */
  public static void SimDebugf(String format, Object... args) {
    globalInstance.Format(MessageType.Debug, Scope.Runtime, format, args);
  }

  /** Formats and prints a log message using a given format string and arguments */
  public void Format(MessageType type, Scope scope, String format, Object... args) {
    globalInstance.Print(type, scope, String.format(format, args));
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

  /** BotLogPrinter is a command that prints to the BotLog in its init phase */
  private static class BotLogPrintCommand extends CommandBase {
    protected MessageType m_type;
    protected Scope m_scope;
    protected final String m_message;

    /** Creates a command that prints a message in runtime and simulation */
    public BotLogPrintCommand(MessageType type, Scope scope, String message) {
      m_type = type;
      m_scope = scope;
      m_message = message;
    }

    /** Creates a command that prints a message in runtime and simulation */
    public BotLogPrintCommand(MessageType type, Scope scope, String format, Object... args) {
      this(type, scope, String.format(format, args));
    }

    @Override
    public void initialize() {
      BotLog.globalInstance.Print(m_type, Scope.Runtime, m_message);
    }

    @Override
    public boolean isFinished() {
      return true;
    }
  }

  /** A command that prints an Error message to the bot log */
  public static class ErrorPrintCommand extends BotLogPrintCommand {
    /** Creates a command that prints a given message string */
    public ErrorPrintCommand(String message) {
      super(MessageType.Error, Scope.Runtime, message);
    }

    /** Creates a command that prints a given formatted string */
    public ErrorPrintCommand(String format, Object... args) {
      super(MessageType.Error, Scope.Runtime, format, args);
    }
  }

  /** A command that prints a Warning message to the bot log */
  public static class WarningPrintCommand extends BotLogPrintCommand {
    /** Creates a command that prints a given message string */
    public WarningPrintCommand(String message) {
      super(MessageType.Warning, Scope.Runtime, message);
    }

    /** Creates a command that prints a given formatted string */
    public WarningPrintCommand(String format, Object... args) {
      super(MessageType.Warning, Scope.Runtime, format, args);
    }
  }

  /** A command that prints an Info message to the bot log */
  public static class InfoPrintCommand extends BotLogPrintCommand {
    /** Creates a command that prints a given message string */
    public InfoPrintCommand(String message) {
      super(MessageType.Info, Scope.Runtime, message);
    }

    /** Creates a command that prints a given formatted string */
    public InfoPrintCommand(String format, Object... args) {
      super(MessageType.Info, Scope.Runtime, format, args);
    }
  }

  /** A command that prints a Debug message to the bot log */
  public static class DebugPrintCommand extends BotLogPrintCommand {
    /** Creates a command that prints a given message string */
    public DebugPrintCommand(String message) {
      super(MessageType.Debug, Scope.Runtime, message);
    }

    /** Creates a command that prints a given formatted string */
    public DebugPrintCommand(String format, Object... args) {
      super(MessageType.Debug, Scope.Runtime, format, args);
    }
  }

  /** A command that prints an Error message to the bot log */
  public static class SimErrorPrintCommand extends BotLogPrintCommand {
    /** Creates a command that prints a given message string */
    public SimErrorPrintCommand(String message) {
      super(MessageType.Error, Scope.SimulationOnly, message);
    }

    /** Creates a command that prints a given formatted string */
    public SimErrorPrintCommand(String format, Object... args) {
      super(MessageType.Error, Scope.SimulationOnly, format, args);
    }
  }

  /** A command that prints a Warning message to the bot log */
  public static class SimWarningPrintCommand extends BotLogPrintCommand {
    /** Creates a command that prints a given message string */
    public SimWarningPrintCommand(String message) {
      super(MessageType.Warning, Scope.SimulationOnly, message);
    }

    /** Creates a command that prints a given formatted string */
    public SimWarningPrintCommand(String format, Object... args) {
      super(MessageType.Warning, Scope.SimulationOnly, format, args);
    }
  }

  /** A command that prints an Info message to the bot log */
  public static class SimInfoPrintCommand extends BotLogPrintCommand {
    /** Creates a command that prints a given message string */
    public SimInfoPrintCommand(String message) {
      super(MessageType.Info, Scope.SimulationOnly, message);
    }

    /** Creates a command that prints a given formatted string */
    public SimInfoPrintCommand(String format, Object... args) {
      super(MessageType.Info, Scope.SimulationOnly, format, args);
    }
  }

  /** A command that prints a Debug message to the bot log */
  public static class SimDebugPrintCommand extends BotLogPrintCommand {
    /** Creates a command that prints a given message string */
    public SimDebugPrintCommand(String message) {
      super(MessageType.Debug, Scope.SimulationOnly, message);
    }

    /** Creates a command that prints a given formatted string */
    public SimDebugPrintCommand(String format, Object... args) {
      super(MessageType.Debug, Scope.SimulationOnly, format, args);
    }
  }
}
