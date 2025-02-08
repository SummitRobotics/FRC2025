package frc.robot.sensors;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class WaveshareCanToFSensor implements Sendable {
    private final CAN tofDevice;
    private double distanceMeters;
    private int signalStrength;
    private int reserved;
    private int statusCode = ToFStatus.UNKNOWN.getCode(); // default to unknown status
    private double timestamp; // Millisecond timestamp from CANData

    // Enumeration for TOF sensor status
    public enum ToFStatus {
        VALID(0, "Measuring distance is valid"),
        STD_DEV_EXCEEDED(1, "Standard deviation is more than 15mm"),
        SIGNAL_LOW(2, "Signal strength is lower than 1Mcps"),
        PHASE_EXCEEDS(4, "Phase exceeds boundary"),
        HW_VCSEL_FAULT(5, "HW or VCSEL has fault"),
        PHASE_MISMATCH(7, "Phase is not matched"),
        ALGORITHM_UNDERFLOW(8, "Internal algorithm underflow"),
        DISTANCE_INVALID(14, "Measuring distance is invalid"),
        UNKNOWN(-1, "Unknown status");

        private final int code;
        private final String description;
        ToFStatus(int code, String description) {
            this.code = code;
            this.description = description;
        }
        public int getCode() {
            return code;
        }
        public String getDescription() {
            return description;
        }
        public static ToFStatus fromCode(int code) {
            for (ToFStatus status : values()) {
                if (status.getCode() == code) {
                    return status;
                }
            }
            return UNKNOWN;
        }
    }

    /**
     * Constructs the Waveshare TOF sensor object using the given CAN device ID.
     *
     * @param deviceId The CAN device ID for the TOF sensor.
     */
    public WaveshareCanToFSensor(int deviceId) {
        tofDevice = new CAN(deviceId);
        // Uncomment the following line to run the self-test process on creation.
        // runTestData();
    }

    /**
     * Reads the latest data packet from the sensor over CAN and updates the internal state.
     * (SmartDashboard publishing is removed; values are exposed via Sendable.)
     */
    public void update() {
        CANData data = new CANData();
        boolean received = tofDevice.readPacketLatest(0, data);
        if (received) {
            processCanData(data);
        }
        // If no data is received, internal state remains unchanged.
    }

    /**
     * Processes the provided CANData packet.
     * This method can be called from tests or simulated scenarios.
     */
    public void processCanData(CANData data) {
        // 24-bit unsigned distance in little-endian order: bytes 0 (LSB) to 2 (MSB)
        int rawDistance = (data.data[0] & 0xFF)
                        | ((data.data[1] & 0xFF) << 8)
                        | ((data.data[2] & 0xFF) << 16);
        // Status from byte 3 (unsigned 8-bit)
        statusCode = data.data[3] & 0xFF;
        ToFStatus status = ToFStatus.fromCode(statusCode);
        // Save the timestamp from the CANData packet.
        timestamp = data.timestamp;

        if (status == ToFStatus.VALID) {
            distanceMeters = rawDistance / 1000.0;
            // Signal strength from bytes 4-5 (little-endian)
            signalStrength = (data.data[4] & 0xFF) | ((data.data[5] & 0xFF) << 8);
            // Reserved field from bytes 6-7 (little-endian)
            reserved = (data.data[6] & 0xFF) | ((data.data[7] & 0xFF) << 8);
        }
        // Otherwise, the internal values remain unchanged (or could be reset if desired).
    }

    // Self-test method that runs a dummy CANData packet through processCanData.
    public void runTestData() {
        CANData data = new CANData();
        // Construct test packet:
        // Distance (24-bit, little-endian): 0xAD, 0x08, 0x00 => 0x0008AD (2221)
        data.data[0] = (byte) 0xAD;  // LSB of distance
        data.data[1] = (byte) 0x08;
        data.data[2] = (byte) 0x00;  // MSB of distance
        // Status (byte 3): 0x00 (VALID)
        data.data[3] = (byte) 0x00;
        // Signal strength (16-bit, little-endian): 0x03, 0x00 => 0x0003 (3)
        data.data[4] = (byte) 0x03;
        data.data[5] = (byte) 0x00;
        // Reserved (16-bit, little-endian): 0x34, 0x12 => 0x1234 (4660)
        data.data[6] = (byte) 0x34;
        data.data[7] = (byte) 0x12;

        // Set the timestamp (in milliseconds).
        data.timestamp = 123456;

        // Process the dummy CANData packet.
        processCanData(data);

        // Print test output.
        System.out.println("Self-Test Data Processed:");
        System.out.println("Distance (m): " + getDistanceInMeters());
        System.out.println("Signal Strength: " + getSignalStrength());
        System.out.println("Reserved: " + getReserved());
        System.out.println("Status: " + getStatus().getDescription());
        System.out.println("Timestamp (ms): " + getTimestamp());
    }

    public double getDistanceInMeters() {
        return distanceMeters;
    }

    public int getSignalStrength() {
        return signalStrength;
    }

    public int getReserved() {
        return reserved;
    }

    public ToFStatus getStatus() {
        return ToFStatus.fromCode(statusCode);
    }

    public double getTimestamp() {
        return timestamp;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("WaveshareToFSensor");
        builder.addDoubleProperty("Distance (m)", this::getDistanceInMeters, null);
        builder.addDoubleProperty("Signal Strength", () -> (double) getSignalStrength(), null);
        builder.addDoubleProperty("Reserved", () -> (double) getReserved(), null);
        builder.addStringProperty("Status", () -> getStatus().getDescription(), null);
        builder.addDoubleProperty("Timestamp (ms)", this::getTimestamp, null);
    }
}