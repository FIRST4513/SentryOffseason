package org.team340.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team340.lib.logging.LoggedRobot;
import org.team340.lib.logging.Profiler;
import org.team340.lib.util.DisableWatchdog;
import org.team340.robot.commands.Autos;
import org.team340.robot.commands.Routines;
import org.team340.robot.subsystems.Swerve;
import org.team340.robot.subsystems.climber.ClimberCmds;
import org.team340.robot.subsystems.climber.ClimberSubsys;
import org.team340.robot.subsystems.elevator.ElevatorCmds;
import org.team340.robot.subsystems.elevator.ElevatorSubsys;
import org.team340.robot.subsystems.elevator.ElevatorSubsys.ElevatorState;
import org.team340.robot.subsystems.finger.FingerCmds;
import org.team340.robot.subsystems.finger.FingerSubsys;
import org.team340.robot.subsystems.intake.IntakeCmds;
import org.team340.robot.subsystems.intake.IntakeSubsys;

@Logged
public final class Robot extends LoggedRobot {

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    public final Swerve swerve;
    public static IntakeSubsys intake;
    public static ClimberSubsys climber;
    public static FingerSubsys finger;
    public static ElevatorSubsys elevator;

    public final Routines routines;
    public final Autos autos;

    private final CommandXboxController driver;
    private final CommandXboxController coDriver;

    public Robot() {
        // Initialize subsystems
        swerve = new Swerve();
        intake = new IntakeSubsys();
        climber = new ClimberSubsys();
        finger = new FingerSubsys();
        elevator = new ElevatorSubsys();

        // Initialize compositions
        routines = new Routines(this);
        autos = new Autos(this);

        // Initialize controllers
        driver = new CommandXboxController(Constants.DRIVER);
        coDriver = new CommandXboxController(Constants.CO_DRIVER);

        // Set default commands
        swerve.setDefaultCommand(swerve.drive(this::driverX, this::driverY, this::driverAngular));

        // Driver bindings
        //driver.a().onTrue(none());
        //driver.povLeft().onTrue(swerve.tareRotation());

        // Co-driver bindings
        coDriver.a().onTrue(ElevatorCmds.levelone);

        coDriver.b().onTrue(ElevatorCmds.leveltwo);

        coDriver.x().onTrue(ElevatorCmds.levelthree);

        coDriver.y().onTrue(ElevatorCmds.levelfour);

        coDriver.button(10).onTrue(ElevatorCmds.elevatorSetState(ElevatorState.HIGHALGAE));

        coDriver.povUp().whileTrue(ElevatorCmds.elevatorSetManual());
        coDriver.povUp().onFalse(ElevatorCmds.elevatorSetStopped());

        coDriver.povRight().onTrue(ElevatorCmds.elevatorSetState(ElevatorState.HIGHALGAE));

        coDriver.rightBumper().whileTrue(IntakeCmds.intakeSetShootCmd());
        coDriver.rightBumper().onFalse(IntakeCmds.intakeSetStoppedCmd());

        coDriver.leftBumper().onTrue(IntakeCmds.intakeGroundUntilGamepieceCmd());

        coDriver.button(7).whileTrue(FingerCmds.fingerSetOn());
        coDriver.button(7).onFalse(FingerCmds.fingerSetStopped());
        coDriver.button(8).whileTrue(FingerCmds.fingerSetIn());
        coDriver.button(8).onFalse(FingerCmds.fingerSetStopped());

        driver.button(7).onTrue(swerve.tareRotation());

        driver.povLeft().onTrue(ClimberCmds.climberSetExtend());
        driver.povLeft().onFalse(ClimberCmds.climberStopCmd());
        driver.povRight().onTrue(ClimberCmds.climberSetStow());
        driver.povRight().onFalse(ClimberCmds.climberStopCmd());

        driver.a().onTrue(new InstantCommand(() -> System.out.println(autos.getSelectedAutoMaybe())));

        // Disable loop overrun warnings from the command
        // scheduler, since we already log loop timings
        DisableWatchdog.in(scheduler, "m_watchdog");

        // Configure the brownout threshold to match RIO 1
        RobotController.setBrownoutVoltage(6.3);

        // Enable real-time thread priority (whatever that means)
        enableRT(true);
    }

    /**
     * Returns the current match time in seconds.
     */
    public double matchTime() {
        return Math.max(0.0, DriverStation.getMatchTime());
    }

    @NotLogged
    public double driverX() {
        return driver.getLeftX();
    }

    @NotLogged
    public double driverY() {
        return driver.getLeftY();
    }

    @NotLogged
    public double driverAngular() {
        return driver.getLeftTriggerAxis() - driver.getRightTriggerAxis();
    }

    // @Override
    // public void teleopInit() {
    //     // TODO Auto-generated method stub
    //     resetCommandsAndButtons();
    // }

    // @Override
    // public void robotPeriodic() {
    //     Threads.setCurrentThreadPriority(true, 99);
    //     CommandScheduler.getInstance().run(); // Make sure scheduled commands get run
    //     Threads.setCurrentThreadPriority(true, 10); // Set the main thread back to normal priority
    // }

    @Override
    public void robotPeriodic() {
        Profiler.run("scheduler", scheduler::run);
    }

    public static void resetCommandsAndButtons() {
        CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
    }
}
