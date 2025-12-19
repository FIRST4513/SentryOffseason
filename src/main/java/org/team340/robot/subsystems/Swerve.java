package org.team340.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team340.lib.logging.LoggedRobot;
import org.team340.lib.math.Math2;
import org.team340.lib.math.PAPFController;
import org.team340.lib.math.PAPFController.LineObstacle;
import org.team340.lib.math.PAPFController.Obstacle;
import org.team340.lib.swerve.Perspective;
import org.team340.lib.swerve.SwerveAPI;
import org.team340.lib.swerve.SwerveState;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.SwerveEncoders;
import org.team340.lib.swerve.hardware.SwerveIMUs;
import org.team340.lib.swerve.hardware.SwerveMotors;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.Alliance;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants;
import org.team340.robot.Constants.RobotMap;
import org.team340.robot.util.Field;
import org.team340.robot.util.Field.ReefLocation;
import org.team340.robot.util.Vision;

/**
 * The robot's swerve drivetrain.
 */
@Logged
public final class Swerve extends GRRSubsystem {

    private static final double OFFSET = Units.inchesToMeters(12.5);

    private static final TunableTable tunables = Tunables.getNested("swerve");

    private static final TunableTable apfTunables = tunables.getNested("apf");
    private static final TunableDouble apfX = apfTunables.value("x", 1.14);
    private static final TunableDouble apfVel = apfTunables.value("velocity", 4.5);
    private static final TunableDouble apfLead = apfTunables.value("lead", 0.45);
    private static final TunableDouble apfLeadMult = apfTunables.value("leadMult", 0.15);
    private static final TunableDouble apfLeadAccel = apfTunables.value("leadAccel", 7.7);
    private static final TunableDouble apfLeadAccelL4 = apfTunables.value("leadAccelL4", 6.95);
    private static final TunableDouble apfScoreAccel = apfTunables.value("scoreAccel", 6.0);
    private static final TunableDouble apfScoreAccelL4 = apfTunables.value("scoreAccelL4", 3.5);
    private static final TunableDouble apfL4Ta = apfTunables.value("apfL4Ta", 4.0);
    private static final TunableDouble apfAngTolerance = apfTunables.value("angTolerance", 0.4);
    private static final TunableDouble apfSafeTolerance = apfTunables.value("safeTolerance", 0.2);
    private static final TunableDouble apfAttractStrength = apfTunables.value("attractStrength", -9.0);
    private static final TunableDouble apfAttractRange = apfTunables.value("attractRange", 2.5);

    private final SwerveModuleConfig frontLeft = new SwerveModuleConfig()
        .setName("frontLeft")
        .setLocation(OFFSET, OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.FL_MOVE, false))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.FL_TURN, true))
        .setEncoder(SwerveEncoders.cancoder(RobotMap.FL_ENCODER, -0.37890625, false));

    private final SwerveModuleConfig frontRight = new SwerveModuleConfig()
        .setName("frontRight")
        .setLocation(OFFSET, -OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.FR_MOVE, true))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.FR_TURN, true))
        .setEncoder(SwerveEncoders.cancoder(RobotMap.FR_ENCODER, 0.44775390625, false));

    private final SwerveModuleConfig backLeft = new SwerveModuleConfig()
        .setName("backLeft")
        .setLocation(-OFFSET, OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.BL_MOVE, false))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.BL_TURN, true))
        .setEncoder(SwerveEncoders.cancoder(RobotMap.BL_ENCODER, 0.048583984375, false));

    private final SwerveModuleConfig backRight = new SwerveModuleConfig()
        .setName("backRight")
        .setLocation(-OFFSET, -OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.BR_MOVE, true))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.BR_TURN, true))
        .setEncoder(SwerveEncoders.cancoder(RobotMap.BR_ENCODER, -0.411376953125, false));

    private final SwerveConfig config = new SwerveConfig()
        .setTimings(LoggedRobot.DEFAULT_PERIOD, 0.004, 0.02, 0.02)
        .setMovePID(0.25, 0.0, 0.0)
        .setMoveFF(0.0, 0.125)
        .setTurnPID(100.0, 0.0, 0.2)
        .setBrakeMode(true, true)
        .setLimits(5.0, 0.01, 18.0, 15.0, 45.0)
        .setDriverProfile(1.0, 1.5, 0.1, 1.0, 2.0, 0.05)
        .setPowerProperties(Constants.VOLTAGE, 100.0, 80.0, 60.0, 60.0)
        .setMechanicalProperties(243.0 / 38.0, 12.1, Units.inchesToMeters(4.0))
        .setOdometryStd(0.1, 0.1, 0.05)
        .setIMU(SwerveIMUs.pigeon2(RobotMap.CANANDGYRO))
        .setPhoenixFeatures(new CANBus(RobotMap.LOWER_CAN), true, true, true)
        .setModules(frontLeft, frontRight, backLeft, backRight);

    @NotLogged
    private final SwerveState state;

    private final SwerveAPI api;
    private final Vision vision;
    private boolean seesAprilTag = false;

    private final PAPFController apf;
    private final ProfiledPIDController angularPID;

    private final ReefAssistData reefAssist = new ReefAssistData();
    private Pose2d reefReference = Pose2d.kZero;

    public Swerve() {
        api = new SwerveAPI(config);
        vision = new Vision(Constants.CAMERAS);
        apf = new PAPFController(6.0, 0.25, 0.01, true, Field.obstacles);
        angularPID = new ProfiledPIDController(8.0, 0.0, 0.0, new Constraints(10.0, 26.0));
        angularPID.enableContinuousInput(-Math.PI, Math.PI);

        state = api.state;

        tunables.add("api", api);
        tunables.add("apf", apf);
        Tunables.add("swerve/angularPID", angularPID);
    }

    @Override
    public void periodic() {
        api.refresh();

        // Apply vision estimates to the pose estimator.
        var measurements = vision.getUnreadResults(state.poseHistory, state.odometryPose, state.velocity);
        //System.out.println(measurements);
        this.seesAprilTag = measurements.length > 0;
        api.addVisionMeasurements(measurements);

        // Calculate helpers
        Translation2d reefCenter = Field.reef.get();
        Translation2d reefTranslation = state.translation.minus(reefCenter);
        Rotation2d reefAngle = new Rotation2d(
            Math.floor(
                    reefCenter.minus(state.translation).getAngle().plus(new Rotation2d(Math2.SIXTH_PI)).getRadians()
                        / Math2.THIRD_PI
                )
                * Math2.THIRD_PI
        );
    }

    /**
     * Tares the rotation of the robot. Useful for
     * fixing an out of sync or drifting IMU.
     */
    public Command tareRotation() {
        return commandBuilder("Swerve.tareRotation()")
            .onInitialize(() -> {
                api.tareRotation(Perspective.OPERATOR);
                vision.reset();
            })
            .isFinished(true)
            .ignoringDisable(true);
    }

    /**
     * Resets the pose of the robot, inherently seeding field-relative movement.
     * @param pose A supplier that returns the new blue origin relative pose to apply to the pose estimator.
     */
    public Command resetPose(Supplier<Pose2d> pose) {
        return commandBuilder("Swerve.resetPose()")
            .onInitialize(() -> api.resetPose(pose.get()))
            .isFinished(true)
            .ignoringDisable(true);
    }

    /**
     * Drives the robot using driver input.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     */
    public Command drive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angular) {
        return commandBuilder("Swerve.drive()").onExecute(() ->
            api.applyDriverInput(
                x.getAsDouble(),
                y.getAsDouble(),
                angular.getAsDouble(),
                Perspective.OPERATOR,
                true,
                true
            )
        );
    }

    /**
     * Drives the robot to a target position using the P-APF, until the
     * robot is positioned within a specified tolerance of the target.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param maxDeceleration A supplier that returns the desired deceleration rate of the robot, in m/s/s.
     * @param endTolerance The tolerance in meters at which to end the command.
     */
    public Command apfDrive(Supplier<Pose2d> goal, DoubleSupplier maxDeceleration, DoubleSupplier endTolerance) {
        return apfDrive(goal, maxDeceleration)
            .until(() -> Math2.isNear(goal.get().getTranslation(), state.translation, endTolerance.getAsDouble()))
            .withName("Swerve.apfDrive()");
    }

    /**
     * Drives the robot to the reef autonomously.
     * @param location The reef location to drive to.
     * @param ready If the robot is ready to approach the scoring location.
     * @param l4 If the robot is scoring L4.
     */
    public Command apfDrive(ReefLocation location, BooleanSupplier ready, BooleanSupplier l4) {
        return apfDrive(
            () -> Alliance.isBlue() ? location.side : location.side.rotateBy(Rotation2d.k180deg),
            () -> location.left,
            ready,
            l4
        );
    }

    /**
     * Internal function, converts reef side to APF drive controller.
     * @param side A supplier that returns the side of the reef to target.
     * @param left A supplier that returns {@code true} if the robot should target
     *             the left reef pole, or {@code false} to target the right pole.
     * @param ready If the robot is ready to approach the scoring location.
     * @param l4 If the robot is scoring L4.
     */

    private Command apfDrive(
        Supplier<Rotation2d> side,
        BooleanSupplier left,
        BooleanSupplier ready,
        BooleanSupplier l4
    ) {
        Mutable<Pose2d> lastTarget = new Mutable<>(Pose2d.kZero);
        Mutable<Double> torqueAccel = new Mutable<>(config.torqueAccel);
        Mutable<Boolean> nowSafe = new Mutable<>(false);

        return commandBuilder("Swerve.apfDrive()")
            .onInitialize(() -> {
                angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond);

                lastTarget.value = Pose2d.kZero;
                torqueAccel.value = config.torqueAccel;
                nowSafe.value = false;
            })
            .onExecute(() -> {
                Pose2d goal = reefAssist.targetPipe = generateReefLocation(apfX.get(), side.get(), left.getAsBoolean());

                Translation2d error = goal.getTranslation().minus(state.translation);
                Rotation2d robotAngle = error.getAngle();
                reefAssist.error = robotAngle.minus(side.get()).getRadians();

                if (!goal.equals(lastTarget.value)) nowSafe.value = false;
                lastTarget.value = goal;

                if (!nowSafe.value) {
                    goal = generateReefLocation(
                        apfX.get() + apfLead.get() + (apfLeadMult.get() * (error.getNorm() - apfLead.get())),
                        side.get(),
                        left.getAsBoolean()
                    );

                    if (
                        ready.getAsBoolean()
                        && state.translation.getDistance(goal.getTranslation()) * (Math.abs(reefAssist.error) / Math.PI)
                        <= apfSafeTolerance.get()
                        && Math.abs(state.rotation.minus(goal.getRotation()).getRadians()) <= apfAngTolerance.get()
                    ) {
                        if (l4.getAsBoolean()) config.torqueAccel = apfL4Ta.get();
                        nowSafe.value = true;
                    }
                }

                double deceleration = !nowSafe.value
                    ? (l4.getAsBoolean() ? apfLeadAccelL4.get() : apfLeadAccel.get())
                    : (l4.getAsBoolean() ? apfScoreAccelL4.get() : apfScoreAccel.get());

                Obstacle attract = new LineObstacle(
                    generateReefLocation(10.0, side.get(), left.getAsBoolean()).getTranslation(),
                    reefAssist.targetPipe.getTranslation(),
                    true,
                    apfAttractStrength.get(),
                    apfAttractRange.get()
                );

                var speeds = apf.calculate(state.pose, goal.getTranslation(), apfVel.get(), deceleration, attract);

                speeds.omegaRadiansPerSecond = angularPID.calculate(
                    state.rotation.getRadians(),
                    goal.getRotation().getRadians()
                );

                api.applySpeeds(speeds, Perspective.BLUE, true, true);
            })
            .onEnd(() -> config.torqueAccel = torqueAccel.value);
    }

    /**
     * Drives the robot to a target position using the P-APF. Thi   s command does not end.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param maxDeceleration A supplier that returns the desired deceleration rate of the robot, in m/s/s.
     */
    public Command apfDrive(Supplier<Pose2d> goal, DoubleSupplier maxDeceleration) {
        return commandBuilder("Swerve.apfDrive()")
            .onInitialize(() -> angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond))
            .onExecute(() -> {
                Pose2d next = goal.get();
                var speeds = apf.calculate(
                    state.pose,
                    next.getTranslation(),
                    config.velocity,
                    maxDeceleration.getAsDouble()
                );

                speeds.omegaRadiansPerSecond = angularPID.calculate(
                    state.rotation.getRadians(),
                    next.getRotation().getRadians()
                );

                api.applySpeeds(speeds, Perspective.BLUE, true, true);
            });
    }

    /**
     * Drives the modules to stop the robot from moving.
     * @param lock If the wheels should be driven to an X formation to stop the robot from being pushed.
     */
    public Command stop(boolean lock) {
        return commandBuilder("Swerve.stop(" + lock + ")").onExecute(() -> api.applyStop(lock));
    }

    private Pose2d generateReefLocation(double xOffset, Rotation2d side, boolean left) {
        return new Pose2d(
            reefReference
                .getTranslation()
                .plus(new Translation2d(-xOffset, Field.pipeY * (left ? 1.0 : -1.0)).rotateBy(side)),
            side
        );
    }

    @Logged
    public final class ReefAssistData {

        private Pose2d targetPipe = Pose2d.kZero;
        private boolean running = false;
        private double error = 0.0;
        private double output = 0.0;
    }

    public boolean returnTrue() {
        return true;
    }

    public boolean returnFalse() {
        return false;
    }
}
