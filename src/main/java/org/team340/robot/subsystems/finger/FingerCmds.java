package org.team340.robot.subsystems.finger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.finger.FingerSubsys.FingerState;

public class FingerCmds {

    public static Command fingerStopCmd() {
        return new InstantCommand(() -> Robot.finger.stopMotors());
    }

    public static Command fingerSetState(FingerState newState) {
        return new InstantCommand(() -> Robot.finger.setNewState(newState));
    }

    public static Command fingerSetOn() {
        return fingerSetState(FingerState.ON);
    }

    public static Command fingerSetIn() {
        return fingerSetState(FingerState.IN);
    }

    public static Command fingerSetHold() {
        return fingerSetState(FingerState.HOLD);
    }

    public static Command fingerSetStopped() {
        return fingerSetState(FingerState.STOPPED);
    }

    public static Command fingerSetTemp() {
        return new SequentialCommandGroup(fingerSetOn(), new WaitCommand(0.3), fingerSetStopped());
    }
}
