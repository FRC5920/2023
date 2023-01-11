package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.runtimeState.BotStateSubsystem;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    private BotStateSubsystem BotState;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop, BotStateSubsystem m_BotStateSubsystem) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.BotState = m_BotStateSubsystem;
    }

    @Override
    public void execute() {
        double yAxis = -controller.getRawAxis(translationAxis);
        double xAxis = -controller.getRawAxis(strafeAxis); 
        double rAxis = -controller.getRawAxis(rotationAxis);
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.DriverConstants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.DriverConstants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.DriverConstants.stickDeadband) ? 0 : rAxis;

        translation = new Translation2d(yAxis, xAxis).times(BotState.MaxSpeed);
        rotation = rAxis * BotState.MaxRotate;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}
