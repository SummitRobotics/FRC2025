package frc.robot.utilities.lists;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class Constants {
    // TODO - set all of these
    public static final String
        RIO_BUS_NAME = "rio";
        // CANIVORE_BUS_NAME = "canivore";
    public static class OI {
        public static int
            DRIVER_XBOX = 0,
            GUNNER_XBOX = 1,
            DRIVER_PS5 = DRIVER_XBOX,
            GUNNER_PS5 = GUNNER_XBOX,
            BUTTON_BOX = 2;
    }
    public static class Devices {
        public static final int
            TOF_ID_LEFT = 41,
            TOF_ID_RIGHT = 40;
    }
    public static class Elevator {
        public static final int
            ELEVATOR_ID_A = 13,
            ELEVATOR_ID_B = 14;
        public static final AngularVelocity
            // ELEVATOR_MM_VEL = RotationsPerSecond.of(5);
            ELEVATOR_MM_VEL = RotationsPerSecond.of(30);
        public static final AngularAcceleration
            // ELEVATOR_MM_ACCEL = RotationsPerSecondPerSecond.of(10);
            ELEVATOR_MM_ACCEL = RotationsPerSecondPerSecond.of(60);
        public static final double
            P = 1,
            I = 0,
            D = 0,
            kS = 0,
            kG = 0.8,
            kV = 0.15,
            kA = 0,
            MAX_ROTATIONS = 15.5,
            ROTATION_TOLERANCE = 1;
    }

    public static class Manipulator {
        public static final int
            BELT_LEFT_ID = 11,
            BELT_RIGHT_ID = 10,
            PIVOT_ID = 28,
            CORAL_SENSOR_ID = 50,
            CANCODER_ID = 5;
        public static final double
            MAX_ROTATIONS = 0.3,
            MIN_ROTATIONS = -0.246,
            CLEAR_OF_ELEVATOR_ROTATIONS = -0.145264,
            PIVOT_RATIO = 54,
            P = 16,
            I = 1,
            D = 0,
            kS = 0,
            kG = 0.27,
            kV = 0.6,
            kA = 0,
            ROTATION_TOLERANCE = 0.1,
            MAX_VELOCITY = 60,
            MAX_ACCEL = 120;
    }

    public static class Scrubber {
        public static final int
            SCRUBBER_ID = 8;
        public static final double
            P = 0.6,
            I = 0,
            D = 0,
            // kG = 0,
            // kS = 0,
            // kV = 0,
            GEAR_RATIO = 16.8,
            MAX_ROTATIONS = GEAR_RATIO / 4;
    }

    public static class Climb {
        public static final int
            CLIMB_ID = 0;
        public static final double
            P = 0,
            I = 0,
            D = 0,
            kG = 0,
            kS = 0,
            kV = 0,
            kA = 0,
            MAX_ROTATIONS = 0;
    }
}
