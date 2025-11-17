package org.team340.robot.subsystems.finger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FingerSubsys extends SubsystemBase {

    public enum FingerState {
        ON,
        IN,
        HOLD,
        STOPPED
    }

    private static FingerState state = FingerState.STOPPED;

    public SparkMax FingerMotor = new SparkMax(23, MotorType.kBrushless);

    public FingerSubsys() {
        stopMotors();
    }

    @Override
    public void periodic() {
        switch (state) {
            case ON:
                FingerMotor.set(.70);
                break;
            case IN:
                FingerMotor.set(-0.4);
                break;
            case HOLD:
                FingerMotor.set(-0.3);
                break;
            default:
                FingerMotor.set(0.0);
                break;
        }
    }

    public void setNewState(FingerState newState) {
        state = newState;
    }

    public void stopMotors() {
        FingerMotor.stopMotor();
        state = FingerState.STOPPED;
    }
}
