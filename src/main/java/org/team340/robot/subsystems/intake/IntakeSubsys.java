package org.team340.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team340.robot.Constants.AnalogPorts;
import org.team340.robot.Constants.RobotMap;

public class IntakeSubsys extends SubsystemBase {

    public SparkMax intakeTopMotor = new SparkMax(RobotMap.IntakeTopMotorID, MotorType.kBrushless);
    public SparkMax intakeBottomMotor = new SparkMax(RobotMap.IntakeBottomMotorID, MotorType.kBrushless);
    public AnalogInput gamepieceDetectSensor = new AnalogInput(AnalogPorts.intakeGamepieceSensor);

    public enum IntakeState {
        FEED,
        SHOOT,
        HOLD,
        STOPPED
    }

    public static IntakeState state = IntakeState.STOPPED;

    public IntakeSubsys() {
        stopMotors();
    }

    @Override
    public void periodic() {
        switch (state) {
            case FEED:
                setSpeeds(IntakeConfig.FEED);
                break;
            case SHOOT:
                setSpeeds(IntakeConfig.SHOOT);
                break;
            case HOLD:
                setSpeeds(IntakeConfig.HOLD);
                break;
            default:
                setSpeeds(0);
                break;
        }
    }

    public void setNewState(IntakeState newState) {
        state = newState;
    }

    public void stopMotors() {
        //setBrakeMode(true);
        intakeBottomMotor.stopMotor();
        intakeTopMotor.stopMotor();
        state = IntakeState.STOPPED;
    }

    public void setSpeeds(double speed) {
        intakeBottomMotor.set(speed);
        intakeTopMotor.set(speed);
    }

    public double getSensorVal() {
        return gamepieceDetectSensor.getAverageVoltage();
    }

    public boolean getGamepieceDetected() {
        if (getSensorVal() > IntakeConfig.gamepieceDetectDistance) {
            return true;
        }
        return false;
    }
}
