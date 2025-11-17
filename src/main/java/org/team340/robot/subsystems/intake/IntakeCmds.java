package org.team340.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.intake.IntakeSubsys.IntakeState;

public class IntakeCmds {

    /* ----- Intake Set State Commands ----- */

    /* ----- Intake Stop Command ----- */
    public static Command intakeStopCmd() {
        return new InstantCommand(() -> Robot.intake.stopMotors(), Robot.intake);
    }

    public static Command intakeSetState(IntakeState newState) {
        return new InstantCommand(() -> Robot.intake.setNewState(newState));
    }

    /* ----- Intake Set State Command Shortcuts ----- */
    public static Command intakeSetFeedCmd() {
        return intakeSetState(IntakeState.FEED);
    }

    public static Command intakeSetShootCmd() {
        return intakeSetState(IntakeState.SHOOT);
    }

    public static Command intakeSetHoldCmd() {
        return intakeSetState(IntakeState.HOLD);
    }

    public static Command intakeSetStoppedCmd() {
        return intakeSetState(IntakeState.STOPPED);
    }

    /* ----- Intake Command with Until Conditions */
    public static boolean holdOn = false;

    public static void holdOnSet() {
        holdOn = !holdOn;
    }

    public static Command holdOnCmd() {
        return new InstantCommand(() -> holdOnSet());
    }

    public static boolean getHoldOn() {
        return holdOn;
    }

    public static Command intakeGroundUntilGamepieceCmd() {
        return new SequentialCommandGroup(
            intakeSetFeedCmd(),
            new WaitUntilCommand(() -> Robot.intake.getGamepieceDetected()).withTimeout(4),
            new WaitCommand(0.5),
            //intakeStopCmd(),
            //holdOnCmd(),
            intakeSetHoldCmd()
        );
    }

    /**
     * Run the intake at the shooter feed gamepiece speed until the gamepiece has left the intake, plus a given amount of time.
     * <p>
     * Will timeout after 10 seconds, plus your given time.
     * @param secondsAfterGamepieceDeparture Time to keep running after the gamepiece has left the intake's sensor
     * @return A SequentialCommandGroup
     */
    public static Command intakeFeedCmd(double secondsAfterGamepieceDeparture) {
        return new SequentialCommandGroup(
            intakeSetFeedCmd(),
            // new WaitCommand(TIMEOUT).until(() -> Robot.intake.not()),  // possible change to WaitUntilCommand
            new WaitCommand(secondsAfterGamepieceDeparture),
            intakeSetHoldCmd(),
            intakeStopCmd()
        );
    }
}
