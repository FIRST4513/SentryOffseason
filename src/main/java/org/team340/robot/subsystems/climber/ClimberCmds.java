package org.team340.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.climber.ClimberSubsys.ClimberState;

public class ClimberCmds {

    public static final double TIMEOUT = 10;

    public static void setupDefaultCommand() {
        //    Robot.climber.setDefaultCommand(intakeStopCmd());
    }

    /* ----- climber Stop Command ----- */
    public static Command climberStopCmd() {
        return new InstantCommand(() -> Robot.climber.stopMotors(), Robot.climber);
    }

    /* ----- climber Set State Commands ----- */
    public static Command climberSetState(ClimberState newState) {
        return new InstantCommand(() -> Robot.climber.setNewState(newState));
    }

    /* ----- Intake Set State Command Shortcuts ----- */
    public static Command climberSetExtend() {
        return climberSetState(ClimberState.EXTEND);
    }

    public static Command climberSetStow() {
        return climberSetState(ClimberState.STOW);
    }

    public static Command climberSetStartup() {
        return climberSetState(ClimberState.STARTUP);
    }

    public static Command lockWinch() {
        return new InstantCommand(() -> ClimberSubsys.WinchLock.setAngle(90)); // lock value unknown
    }

    public static Command unlockWinch() {
        return new InstantCommand(() -> ClimberSubsys.WinchLock.setAngle(113)); // unlock value 124.08
    }
}
