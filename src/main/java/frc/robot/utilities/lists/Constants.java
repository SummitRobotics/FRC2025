package frc.robot.utilities.lists;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class Constants {

    // Flag to enable/disable driver controller tuning
    public static boolean ENABLE_DRIVER_CONTROLLER_TUNING = false;

    public static boolean DEBUG_LOG_ENABLED = false;

    public static final String
        RIO_BUS_NAME = "rio";
        // CANIVORE_BUS_NAME = "canivore";

    public static class Physical {
        public static final double
            TOF_OFFSET_METERS = 0.109;
    }

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
            ELEVATOR_MM_VEL = RotationsPerSecond.of(55);
        public static final AngularAcceleration
            // ELEVATOR_MM_ACCEL = RotationsPerSecondPerSecond.of(10);
            ELEVATOR_MM_ACCEL = RotationsPerSecondPerSecond.of(75);
        public static final double
            P = 10,
            I = 0,
            D = 0.05,
            kS = 0,
            kG = 0.68,
            kV = 0.10,
            kA = 0,
            MAX_ROTATIONS = 15.5,
            ROTATION_TOLERANCE = 1;
    }

    public static class Manipulator {
        public static final int
            BELT_LEFT_ID = 22,
            BELT_RIGHT_ID = 21,
            PIVOT_ID = 28,
            CORAL_SENSOR_ID = 50,
            CANCODER_ID = 5;
        public static final double
            MAX_ROTATIONS = 0.3,
            MIN_ROTATIONS = -0.246,
            CLEAR_OF_ELEVATOR_ROTATIONS = -0.145264,
            PIVOT_RATIO = 54,
            P = 50,
            I = 1,
            D = 0,
            kS = 0,
            kG = 0.27,
            kV = 0.4,
            kA = 0.02,
            ROTATION_TOLERANCE = 0.1,
            MAX_VELOCITY = 60,
            MAX_ACCEL = 80;
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
            CLIMB_ID = 17,
            SENSOR_ID = 0;
        public static final double
            P = 20,
            I = 0.4,
            D = 0,
            kG = 0,
            kS = 0,
            kV = 1.2,
            kA = 0,
            MAX_ROTATIONS = 210,
            BACK_ROTATIONS = -50.5,
            MAX_VELOCITY_IN = 180,
            MAX_ACCEL_IN = 200,
            MAX_VELOCITY_OUT = 300,
            MAX_ACCEL_OUT = 200;
    }

    public static class LED {
        public static final int
            PWM_PORT = 0,
            LED_COUNT = 27;
    }
}
