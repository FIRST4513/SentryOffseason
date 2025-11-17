package org.team340.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.elevator.ElevatorSubsys.ElevatorState;

public class ElevatorCmds {

    public static final double TIMEOUT = 10;

    public static void setupDefaultCommand() {
        //    Robot.elevator.setDefaultCommand(elevatorStopCmd());
    }

    /* ----- elevator Stop Command ----- */
    public static Command elevatorStopCmd() {
        return new InstantCommand(() -> Robot.elevator.stopMotors());
    }

    /* ----- elevator Set State Commands ----- */
    public static Command elevatorSetState(ElevatorState newState) {
        return new InstantCommand(() -> Robot.elevator.setNewState(newState));
    }

    /* ----- Intake Set State Command Shortcuts ----- */
    public static Command elevatorSetLevelOne() {
        return elevatorSetState(ElevatorState.LEVELONE);
    }

    public static SequentialCommandGroup levelone = new SequentialCommandGroup(
        elevatorSetManual(),
        elevatorSetLevelOne()
    );

    public static Command elevatorSetLevelTwo() {
        return elevatorSetState(ElevatorState.LEVELTWO);
    }

    public static SequentialCommandGroup leveltwo = new SequentialCommandGroup(
        elevatorSetManual(),
        elevatorSetLevelTwo()
    );

    public static Command elevatorSetLevelThree() {
        return elevatorSetState(ElevatorState.LEVELTHREE);
    }

    public static SequentialCommandGroup levelthree = new SequentialCommandGroup(
        elevatorSetManual(),
        elevatorSetLevelThree()
    );

    public static Command elevatorSetLevelFour() {
        return elevatorSetState(ElevatorState.LEVELFOUR);
    }

    public static SequentialCommandGroup levelfour = new SequentialCommandGroup(
        elevatorSetManual(),
        elevatorSetLevelFour()
    );

    public static Command elevatorSetBottom() {
        return elevatorSetState(ElevatorState.BOTTOM);
    }

    public static Command elevatorSetIntake() {
        return elevatorSetState(ElevatorState.INTAKE);
    }

    public static Command elevatorSetManual() {
        System.out.println("OWW OW OWWWWW");
        return elevatorSetState(ElevatorState.MANUAL);
    }

    public static Command elevatorSetHoldUp() {
        return elevatorSetState(ElevatorState.HOLDUP);
    }

    public static Command elevatorSetHoldDown() {
        return elevatorSetState(ElevatorState.HOLDDOWN);
    }

    public static Command elevatorSetStopped() {
        return elevatorSetState(ElevatorState.STOPPED);
    }
}
