package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;

/**
 * Chaos Theory Lib utility class recycled between years
 */
public class Functions {

    /**
     * Clamps a double between two values.
     *
     * @param in  the input value to clamp
     * @param max the maximum you want it to be
     * @param min the minimum for it to be
     * @return the clamped double
     */
    public static double clampDouble(double in, double max, double min) {
        if (in > max) {
            return max;
        } else {
            return Math.max(in, min);
        }
    }

    /**
     * Clamps the magnitude of a value while preserving its sign
     * @param value The value to clamp
     * @param maxMagnitude The maximum allowed magnitude
     * @return The value with its magnitude limited but sign preserved
     */
    public static double clampMagnitude(double value, double maxMagnitude) {
        // Preserves sign while clamping magnitude
        return Math.signum(value) * Math.min(Math.abs(value), maxMagnitude);
    }

    /**
     * returns input value with deadzone applied.
     *
     * @param deadRange the range in both directions to be dead
     * @param in        the input value to kill
     * @return the value to be used
     */
    public static double deadzone(double deadRange, double in) {
        if (Math.abs(in) < deadRange) {
            return 0;
        } else {
            return in;
        }
    }

    /**
     * returns true if the abs of A is bigger than B.
     *
     * @param inputA reference value
     * @param inputB comparing value
     * @return true if abs of reference is bigger
     */
    public static boolean absGreater(double inputA, double inputB) {
        return Math.abs(inputA) > Math.abs(inputB);
    }

    /**
     * saves an object to a file.
     *
     * @param <T>    the object type
     * @param object the object to save
     * @param path   the path, including the file name, to save
     * @apiNote WARNING, this can fail and not save the object!
     */
    public static <T> void saveObjectToFile(T object, String path) {
        try {
            FileOutputStream fileOut = new FileOutputStream(path);
            ObjectOutputStream objectOut = new ObjectOutputStream(fileOut);
            objectOut.writeObject(object);
            objectOut.close();
            fileOut.close();
        } catch (Exception ex) {
            ex.printStackTrace();
            throw (new RuntimeException("saving failed"));
        }
    }

    /**
     * reads an object out of a file.
     *
     * @param <T>  the object type
     * @param path the path where the object is
     * @return the object read from the file
     * @throws Exception throws an exception if the fie can not be read
     */
    @SuppressWarnings("unchecked")
    public static <T> T retrieveObjectFromFile(String path) throws Exception {
        FileInputStream fis = new FileInputStream(path);
        ObjectInputStream ois = new ObjectInputStream(fis);
        T result = (T) ois.readObject();
        ois.close();
        return result;
    }

    /**
     * creates a trajectory from a file name.
     *
     * @param fileName   the path to and name of the file
     * @return the Trajectory for the path file
     * @throws IOException thrown if the file was not found or readable
     */
    public static Trajectory trajectoryFromFile(String fileName) throws IOException {
        Path path = Filesystem.getDeployDirectory().toPath().resolve(fileName);
        return TrajectoryUtil.fromPathweaverJson(path);
    }

    /**
     * takes a list of numbers and gets the median value of them. 
     *
     * @param toBeAveraged Arraylist of numbers to get the median of
     * @return the median value
     */
    public static double medianWithoutExtraneous(ArrayList<Double> toBeAveraged) {
        Collections.sort(toBeAveraged);
        double mean = 0;
        for (double i : toBeAveraged) {
            mean += i;
        }
        mean /= 3;
        final double asfasd = mean;
        toBeAveraged.removeIf(e -> (e != 0 && Math.abs((asfasd - e) / e) > .2));
        return toBeAveraged.size() == 0 ? 0 : toBeAveraged.get((int) (toBeAveraged.size() / 2));
    }

    /**
     * Finds the point closest to the input value.
     * If 2 points are the same dist it returns the first in the points array.
     *
     * @param value the value to find the point closest to
     * @param points the points that the value should be quantised to
     * @return the point closest to the input value
     */
    public static double findClosestPoint(double value, double[] points) {
        double out = Double.NaN;
        double minError = Double.POSITIVE_INFINITY;
        for (double x : points) {
            double error = Math.abs(x - value);
            if (error < minError) {
                minError = error;
                out = x;
            }
        }
        return out;
    }

    public static boolean withinTolerance(double value, double target, double tolerance) {
        return Math.abs(value - target) < tolerance;
    }

    /** Adjust periodic status frames to minimize CAN utilization. */
    // TODO - Fix after deprecation of these methods
    // public static void setStatusFrames(CANSparkMax controller) {
      // controller.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 95);
      // controller.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
      // controller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
      // controller.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
      // controller.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65533);
      // controller.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65531);
      // controller.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65529);
    // }

    // The CANCoder angle is a discontinuous angle;
    // PID controllers don't like the sudden jump between 0 and 360.
    // Both angleConsumer and recalibrate convert to the version of
    // that angle within 180 of the current position.
    // Formula to eliminate jumps: (integer number of 360s to produce something
    // close to the current angle) * 360 + (smallest representation of target angle)
    public static double makeAngleContinuous(double reference, double target) {
        return Math.round((reference - target % (2 * Math.PI))
            / (2 * Math.PI)) * 2 * Math.PI + target % (2 * Math.PI);
    }

    // Returns the difference between two angles in the range -pi to pi.
    // @param angle1 the first angle in the range -pi to pi
    // @param angle2 the second angle in the range -pi to pi
    public static double angleDifference(double angle1, double angle2) {
        double diff = (angle2 - angle1 + Math.PI) % (2 * Math.PI);
        if (diff < 0) {
            diff += 2 * Math.PI;
        }
        return diff - Math.PI;
    }

    // Adds two angles in the range -pi to pi.
    // @param angle1 the first angle in the range -pi to pi
    // @param angle2 the second angle in the range -pi to pi
    public static double addAngles(double angle1, double angle2) {
        double sum = (angle1 + angle2 + Math.PI) % (2 * Math.PI);
        if (sum < 0) {
            sum += 2 * Math.PI;
        }
        return sum - Math.PI;
    }

    public static Pose2d mirrorPoseToRed(Pose2d pose) {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? pose : new Pose2d(
            17.5 - pose.getX(),
            8 - pose.getY(),
            pose.getRotation().plus(Rotation2d.fromDegrees(180))
        );
    }

    public static Translation2d mirrorPoseToRed(Translation2d translation) {
        Pose2d pose = mirrorPoseToRed(new Pose2d(translation.getX(), translation.getY(), Rotation2d.fromDegrees(0)));
        return new Translation2d(pose.getX(), pose.getY());
    }

    public static boolean poseInTolerance(Pose2d poseA, Pose2d poseB, double radius, double angleTolerance) {
        return Math.hypot(poseA.getX() - poseB.getX(), poseA.getY() - poseB.getY()) < radius
            && Math.abs(poseA.getRotation().minus(poseB.getRotation()).getDegrees()) < angleTolerance;
    }
}
