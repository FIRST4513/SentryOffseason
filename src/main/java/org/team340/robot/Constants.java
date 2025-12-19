package org.team340.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.team340.robot.util.Vision.CameraConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final double VOLTAGE = 12.0;

    // Controller ports
    public static final int DRIVER = 0;
    public static final int CO_DRIVER = 1;

    public static final CameraConfig[] CAMERAS = {
        new CameraConfig(
            "Apriltag Camera",
            new Translation3d(0.3048, -0.2286, 0.1524),
            new Rotation3d(0, 0.0872665, 0)
        ),
        new CameraConfig("TopApriltag Camera", new Translation3d(0.070, -0.070, 0.990), new Rotation3d(0, 0, 0))
    };

    /**
     * The RobotMap class defines CAN IDs, CAN bus names, DIO/PWM/PH/PCM channel
     * IDs, and other relevant identifiers for addressing robot hardware.
     */
    public static final class RobotMap {

        public static final String LOWER_CAN = "CANFD";

        public static final int FL_MOVE = 1;
        public static final int FL_TURN = 2;
        public static final int FR_MOVE = 3;
        public static final int FR_TURN = 4;
        public static final int BL_MOVE = 7;
        public static final int BL_TURN = 8;
        public static final int BR_MOVE = 5;
        public static final int BR_TURN = 6;

        public static final int FL_ENCODER = 9;
        public static final int FR_ENCODER = 10;
        public static final int BL_ENCODER = 12;
        public static final int BR_ENCODER = 11;

        public static final int CANANDGYRO = 13;

        public static final int ClimberMotorID = 20; // Can ID Kraken

        public static final int IntakeBottomMotorID = 21; //
        public static final int IntakeTopMotorID = 22;

        public static final int ElevatorMotorID = 15;
    }

    public final class AnalogPorts {

        public static final int intakeGamepieceSensor = 0;

        //max 4 ports
    }

    public final class PWMPorts {

        public static final int winchLockID = 9;
    }
}
