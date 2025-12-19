package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team340.lib.math.geometry.ExtPose;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.command.AutoChooser;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Swerve;
import org.team340.robot.subsystems.elevator.ElevatorCmds;
import org.team340.robot.subsystems.intake.IntakeCmds;
import org.team340.robot.util.Field;
import org.team340.robot.util.Field.ReefLocation;

/**
 * The Autos class declares autonomous modes, and adds them
 * to the dashboard to be selected by the drive team.
 */
//@SuppressWarnings("unused")
public final class Autos {

    private static final TunableTable tunables = Tunables.getNested("autos");
    private static final TunableDouble deceleration = tunables.value("deceleration", 6.0);
    private static final TunableDouble tolerance = tunables.value("tolerance", 0.05);
    private static final TunableDouble avoidDecel = tunables.value("avoidDecel", 10.0);
    private static final TunableDouble avoidTol = tunables.value("avoidTol", 0.25);

    private final Swerve swerve;
    private final Routines routines;

    private final AutoChooser chooser;

    public Autos(Robot robot) {
        swerve = robot.swerve;
        routines = robot.routines;

        // Create the auto chooser
        chooser = new AutoChooser();

        // Add autonomous modes to the dashboard
        chooser.add("Example", example());
        chooser.add("test", scoreOne(false));
    }

    public boolean defaultSelected() {
        return chooser.defaultSelected().getAsBoolean();
    }

    private Command example() {
        var start = new ExtPose(7.590, 1.900, Rotation2d.k180deg);
        var middle = new ExtPose(6.090, 2.029, Rotation2d.k180deg);
        var end = new ExtPose(4.897, 2.9847, new Rotation2d(2.012));

        return sequence(
            swerve.resetPose(start::get),
            new ParallelCommandGroup(
                ElevatorCmds.levelfour,
                swerve.apfDrive(middle::get, deceleration::get, tolerance::get)
            ),
            swerve.apfDrive(end::get, deceleration::get, tolerance::get),
            new WaitCommand(0.4),
            IntakeCmds.intakeSetShootCmd(),
            new WaitCommand(0.2),
            IntakeCmds.intakeStopCmd(),
            new WaitCommand(0.1),
            ElevatorCmds.elevatorSetLevelOne(),
            swerve.stop(false)
        );
    }

    private Command scoreOne(boolean left) {
        return sequence(
            //ElevatorCmds.elevatorSetLevelFour(),
            deadline(waitSeconds(2.5), swerve.stop(false) /*, routines.stow()*/),
            parallel(
                sequence(
                    swerve.resetPose(new ExtPose(7.590, 1.900, Rotation2d.k180deg)),
                    score(left ? Field.ReefLocation.F : Field.ReefLocation.I, left),
                    avoid(left),
                    swerve.stop(false)
                ),
                sequence(
                    IntakeCmds.intakeSetShootCmd(),
                    new WaitCommand(0.1),
                    IntakeCmds.intakeStopCmd(),
                    ElevatorCmds.elevatorSetLevelOne()
                )
            )
        );
    }

    private Command score(ReefLocation reefLocation, boolean left) {
        //ElevatorCmds.elevatorSetLevelFour();
        return swerve.apfDrive(reefLocation, () -> swerve.returnFalse(), () -> swerve.returnTrue());
    }

    private Command avoid(boolean left) {
        return swerve.apfDrive(() -> Field.avoid.get(left), avoidDecel::get, avoidTol::get);
    }

    public String getSelectedAutoMaybe() {
        return chooser.getSelected().getName().toString();
    }
}
