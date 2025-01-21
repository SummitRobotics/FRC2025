package frc.robot.utilities.lists;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class Constants {
    // TODO - set all of these
    public static final String
        RIO_BUS_NAME = "rio",
        CANIVORE_BUS_NAME = "canivore";
    public static class Elevator {
        public static final int
            ELEVATOR_ID_A = 0,
            ELEVATOR_ID_B = 0;
        public static final AngularVelocity
            ELEVATOR_MM_VEL = RotationsPerSecond.of(5);
        public static final AngularAcceleration
            ELEVATOR_MM_ACCEL = RotationsPerSecondPerSecond.of(10);
        public static final double
            P = 0,
            I = 0,
            D = 0,
            kS = 0,
            kG = 0,
            kV = 0,
            kA = 0,
            MAX_ROTATIONS = 0;
    }

    public static class Manipulator {
        public static final int
            BELT_LEFT_ID = 0,
            BELT_RIGHT_ID = 0,
            PIVOT_ID = 0,
            CANCODER_ID = 0;
        public static final double
            MAX_RADIANS = 0,
            MIN_RADIANS = 0,
            PIVOT_RATIO = 0,
            P = 0,
            I = 0,
            D = 0;
    }

    public static class Scrubber {
        public static final int
            SCRUBBER_ID = 0;
        public static final double
            P = 0,
            I = 0,
            D = 0,
            kG = 0,
            kS = 0,
            kV = 0,
            MAX_ROTATIONS = 0;
    }
}
