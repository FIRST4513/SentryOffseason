package org.team340.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorConfig {

    // IR Prox distance value for detection of a gamepiece

    // retract/eject speeds
    protected static final double LEVELONE = 2.5;
    protected static final double LEVELTWO = 10.25;
    protected static final double LEVELTHREE = 22.1;
    protected static final double LEVELFOUR = 43.25;
    protected static final double HIGHALGAE = 35;
    protected static final double BOTTOM = 0;
    protected static final double INTAKE = 11;
    protected static final double MANUAL = 0.1;
    protected static final double HOLDUP = 0.1;
    protected static final double HOLDDOWN = -0.1;
    protected static final double RESET = 0.03;

    /* Inverts */
    protected static final boolean climberMotorInvert = true;

    // increase to reduce jitter
    //protected static final int climberAllowableError = 0;

    /* elevator Motor Current Limiting */
    protected static final int climberContinuousCurrentLimit = 30; //TODO: find real number
    protected static final int climberPeakCurrentLimit = 30; //TODO: find real number
    protected static final int climberPeakCurrentDuration = 100; //TODO: find real number
    protected static final boolean climberEnableCurrentLimit = true; //TODO: find real number1

    /* Ramp Rate */
    protected static final double openLoopRamp = 0;
    protected static final double closedLoopRamp = 0;

    /* constants for height definitions */
    public static final double MAX_ROTATIONS = 45.5;
    /* configuration constants */
    private static final double mmCruiseVelocity = 200; // 12 rpm cruise
    private static final double mmAcceleration = 200; // ~0.5 seconds to max vel.
    private static final double mmJerk = 0; // ~0.2 seconds to max accel.

    private static final double nonload_kP = 0.0; // (P)roportional value
    private static final double nonload_kI = 0.0; // (I)ntegral Value
    private static final double nonload_kD = 0.1; // (D)erivative Value
    private static final double nonload_kV = 0.12; // Volts/100 (?)
    private static final double nonload_kS = 0.1; // (S)tiction Value:
    private static final double nonload_kG = 0.45;

    private static final boolean enableCurrentLimitting = true;
    private static final double suppCurrent = 40; // Max Amps allowed in Supply
    private static final NeutralModeValue neutralMode = NeutralModeValue.Brake;

    // --------------- Constuctor Setting Up Motor Config values -------------
    protected static TalonFXConfiguration getConfig() {
        /* Declare Configuration Object */
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Configure Motion Magic Values
        MotionMagicConfigs mm = config.MotionMagic;
        mm.MotionMagicCruiseVelocity = mmCruiseVelocity;
        mm.MotionMagicAcceleration = mmAcceleration;
        mm.MotionMagicJerk = mmJerk;

        // Configure PID Slot0 Values (PIDVS)
        Slot0Configs slot0Configs = config.Slot0;
        slot0Configs.kP = nonload_kP;
        slot0Configs.kI = nonload_kI;
        slot0Configs.kD = nonload_kD;
        slot0Configs.kV = nonload_kV;
        slot0Configs.kS = nonload_kS;
        slot0Configs.kG = nonload_kG;

        // Configure Current Limits
        CurrentLimitsConfigs currentLimits = config.CurrentLimits;
        currentLimits.SupplyCurrentLimitEnable = enableCurrentLimitting;
        currentLimits.SupplyCurrentLimit = suppCurrent;

        // Configure Soft Limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ROTATIONS;

        // Configure neutral mode
        config.MotorOutput.NeutralMode = neutralMode;

        // finally return an object that will represent the configs we would like to
        return config;
    }
}
