package frc.robot.devices;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;

/**
 * The TOFSensor class manages the serial communication with one or more daisy-chained
 * WaveShare TOF (Time of Flight) sensors. It reads raw bytes from the serial port,
 * organizes into frames, validates the frames using checksums and status codes, and publishes
 * status reports for each sensor. Each sensor report includes the sensor ID, distance,
 * signal strength, and other relevant information. This data is made available via
 * SmartDashboard for monitoring and debugging purposes.
 *
 * Expectations:
 *  * The TOF sensor is connected to the MXP port on the RoboRIO.
 *  * TOF sensors have been configured (via the Windows application):
 *     * To be in UART mode
 *     * To have unique IDs (0..255)
 *     * To have a baud rate of 921600
 *
 * see: https://www.waveshare.com/wiki/TOF_Laser_Range_Sensor
 */
public class SerialTOFSensor {
    private static final int DEFAULT_BAUD_RATE = 921600;
    private static final int FRAME_SIZE = 16;
    private static final byte HEADER_1 = 0x57; // Frame Header
    private static final byte HEADER_2 = 0x00; // Function Mark

    /**
     * Status enum covering known status values:
     * 0 -> MEASURE_VALID
     * 1 -> STD_DEV_TOO_LARGE
     * 2 -> SIGNAL_TOO_LOW
     * 4 -> PHASE_EXCEEDS_BOUNDARY
     * 5 -> HW_OR_VCSEL_FAULT
     * 7 -> PHASE_NOT_MATCHED
     * 8 -> ALGO_UNDERFLOW
     * 14 -> MEASURE_INVALID
     * -1 -> UNKNOWN
     */
    public static enum Status {
        MEASURE_VALID(0),
        STD_DEV_TOO_LARGE(1),
        SIGNAL_TOO_LOW(2),
        PHASE_EXCEEDS_BOUNDARY(4),
        HW_OR_VCSEL_FAULT(5),
        PHASE_NOT_MATCHED(7),
        ALGO_UNDERFLOW(8),
        MEASURE_INVALID(14),
        UNKNOWN(-1);

        private final int value;

        Status(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }

        public static Status fromInt(int val) {
            for (Status ds : Status.values()) {
                if (ds.value == val) {
                    return ds;
                }
            }
            return UNKNOWN;
        }
    }

    private final SerialPort serialPort;
    private final Map<Integer, SensorData> sensorDataMap;
    private final List<Byte> dataBuffer; // Acts like a queue for received bytes

    /**
     * Constructs a TOFSensor instance with the default baud rate (921600).
     */
    public SerialTOFSensor() {
        this(DEFAULT_BAUD_RATE);
    }

    public SerialTOFSensor(int baudRate) {
        serialPort = new SerialPort(baudRate, SerialPort.Port.kMXP);
        sensorDataMap = new HashMap<>();
        dataBuffer = new ArrayList<>();
    }

    public void tick() {
        // Read all available bytes from the serial port
        int available = serialPort.getBytesReceived();
        if (available > 0) {
            byte[] incoming = serialPort.read(available);
            // Append bytes to dataBuffer
            for (byte b : incoming) {
                dataBuffer.add(b);
            }
        }

        // Keep parsing data from the buffer until no more frames can be found. We are
        // only interested in the most recent report from a given ID.
        boolean parsing = true;
        while (parsing) {
            parsing = parseNextFrame();
        }
    }

    /**
     * Searches the dataBuffer for one or more valid frames. For each occurrence of
     * the header bytes (HEADER_1, HEADER_2), this method extracts FRAME_SIZE bytes,
     * parses them, and removes them from dataBuffer. Returns true if at least one
     * valid frame was found and processed, otherwise returns false.
     */
    private boolean parseNextFrame() {
        int bufferSize = dataBuffer.size();

        // Ensure there are enough bytes to parse a frame
        if (bufferSize < FRAME_SIZE) {
            return false;
        }

        // Search for the header in the buffer
        int bytesToRemove = 0;
        boolean parsedFrame = false;
        for (int i = 0; i <= bufferSize - FRAME_SIZE; i++) {
            if (dataBuffer.get(i) == HEADER_1 && dataBuffer.get(i + 1) == HEADER_2) {
                // Extract frame
                byte[] frame = new byte[FRAME_SIZE];
                for (int j = 0; j < FRAME_SIZE; j++) {
                    frame[j] = dataBuffer.get(i + j);
                }

                // Parse the frame
                parseSensorData(frame);

                // Remove the frame from the buffer
                bytesToRemove += FRAME_SIZE;
                parsedFrame = true;
                break;
            } else {
                // Skip this byte
                bytesToRemove++;
            }
        }

        // Remove bytes up to the last frame found
        for (int j = 0; j < bytesToRemove; j++) {
            dataBuffer.remove(0);
        }

        return parsedFrame;
    }

    private void parseSensorData(byte[] data) {
        /*
         * Protocol Description:
         * 1. Frame Header (fixed): data[0] = 0x57
         * 2. Function Mark (fixed): data[1] = 0x00
         * 3. Data (the content of the transmitted data)
         * 4. Sum Check (the last byte): sum of all previous bytes (header + function
         *    mark + data), and then take only the lowest byte.
         *
         * If the sum check fails or the status byte is invalid, we do not consume the
         * data.
         */

        // Ensure this is a 16-byte frame.
        if (data.length != FRAME_SIZE) {
            // Not a valid frame size; ignore.
            return;
        }

        // Verify header.
        if (data[0] != HEADER_1 || data[1] != HEADER_2) {
            // Not a valid frame header; ignore.
            return;
        }

        // Calculate sum of the first 15 bytes (0..14).
        int calcSum = 0;
        for (int i = 0; i < FRAME_SIZE - 1; i++) {
            calcSum += (data[i] & 0xFF);
        }
        calcSum &= 0xFF; // Keep only the lowest byte

        // Compare with the last byte (sum check).
        byte sumCheck = data[FRAME_SIZE - 1];
        if (calcSum != (sumCheck & 0xFF)) {
            // Sum check fails; ignore frame.
            return;
        }

        // Parse ID from some byte in the frame (here assumed at data[3])
        int id = data[3] & 0xFF;

        // Parse system time (little-endian: data[4..7])
        long systemTime = ((data[7] & 0xFFL) << 24) | ((data[6] & 0xFFL) << 16)
                | ((data[5] & 0xFFL) << 8) | (data[4] & 0xFFL);

        // Parse distance (little-endian: data[8..10])
        int distanceRaw = ((data[10] & 0xFF) << 16) | ((data[9] & 0xFF) << 8) | (data[8] & 0xFF);
        double distance = distanceRaw / 1000.0;

        // Distance status byte at data[11]; if non-zero indicates error
        int statusValue = data[11] & 0xFF;
        Status status = Status.fromInt(statusValue);

        // Parse signal strength (little-endian: data[12..13])
        int signalStrength = ((data[13] & 0xFF) << 8) | (data[12] & 0xFF);

        SensorData sensorData = new SensorData(systemTime, distance, status, signalStrength);
        sensorDataMap.put(id, sensorData);

        // Publish to SmartDashboard
        // publishToSmartDashboard(id, sensorData);
    }

    public SensorData getSensorData(int id) {
        return sensorDataMap.get(id);
    }

    private void publishToSmartDashboard(int id, SensorData data) {
        String prefix = "TOF_Sensor_" + id + "_";
        SmartDashboard.putNumber(prefix + "SystemTime", data.systemTime);
        SmartDashboard.putNumber(prefix + "Distance", data.distance);
        SmartDashboard.putString(prefix + "Status", data.status.toString());
        SmartDashboard.putNumber(prefix + "SignalStrength", data.signalStrength);
    }

    public static class SensorData {
        public final long systemTime;
        public final double distance;
        public final Status status;
        public final int signalStrength;

        public SensorData(long systemTime, double distance, Status status, int signalStrength) {
            this.systemTime = systemTime;
            this.distance = distance;
            this.status = status;
            this.signalStrength = signalStrength;
        }
    }
}