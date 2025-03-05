package frc.robot.oi;

import edu.wpi.first.math.MathUtil;

/**
 * Provides different response curves for joystick inputs to improve driver control.
 */
public class JoystickResponseCurve {
    /**
     * Available joystick response curve types.
     */
    public enum CurveType {
        LINEAR,
        SQUARE,
        CUBIC,
        MILD,
        EXPONENTIAL,
        SINUSOIDAL,
        DUAL_RATE
    }

    private static final double DEFAULT_DEADBAND = 0.1;
    private static final double DEFAULT_DUAL_RATE_BREAK = 0.5;
    private static final double DEFAULT_SCALE = 2.0;

    private final CurveType curveType;
    private final double deadband;
    private final double dualRateBreak;
    private final double scale;

    /**
     * Creates a joystick response curve with only the curve type specified. Uses default
     * deadband of 0.1.
     *
     * @param curveType The type of response curve to use
     */
    public JoystickResponseCurve(CurveType curveType) {
        this(curveType, DEFAULT_DEADBAND);
    }

    /**
     * Creates a joystick response curve with specified curve type and deadband.
     *
     * @param curveType The type of response curve to use
     * @param deadband The deadband to apply to inputs
     */
    public JoystickResponseCurve(CurveType curveType, double deadband) {
        this(curveType, deadband, DEFAULT_DUAL_RATE_BREAK, DEFAULT_SCALE);
    }

    /**
     * Creates a joystick response curve with fully custom parameters.
     *
     * @param curveType The type of response curve to use
     * @param deadband The deadband to apply to inputs
     * @param dualRateBreak The breakpoint for DUAL_RATE curves (0.0-1.0)
     * @param scale The scale factor for EXPONENTIAL curve
     */
    public JoystickResponseCurve(
            CurveType curveType, double deadband, double dualRateBreak, double scale) {
        this.curveType = curveType;
        this.deadband = deadband;
        this.dualRateBreak = MathUtil.clamp(dualRateBreak, 0.0, 1.0);
        this.scale = scale;
    }

    /**
     * Applies the configured response curve to an input value.
     *
     * @param input The raw input value (typically -1.0 to 1.0)
     * @return The processed output value
     */
    public double apply(double input) {
        double value = MathUtil.applyDeadband(input, deadband);
        double value_abs = Math.abs(value);
        return switch (curveType) {
            // Desmos: f(x) = x
            case LINEAR -> value;
            // Desmos: f(x) = x^2
            case SQUARE -> Math.signum(value) * Math.pow(value_abs, 2.0);
            // Desmos: f(x) = x^3
            case CUBIC -> Math.signum(value) * Math.pow(value_abs, 3.0);
            // Desmos: f(x) = sign(x) * abs(x)^{1.5}
            case MILD -> Math.signum(value) * Math.pow(value_abs, 1.5);
            // Desmos (scale=2): f(x) = sign(x) * \frac{e^{abs(x) * 2} - 1}{e^2 - 1}
            case EXPONENTIAL -> applyExponential(value);
            // Desmos: f(x) = sign(x) * \sin(abs(x) * \pi/2)
            case SINUSOIDAL -> Math.sin(value * Math.PI / 2);
            // Desmos (threshold=0.5): f(x) = \{abs(x)<0.5: 0.5x, sign(x) * (0.25+(abs(x)-0.5)^2 * 1.5)\}
            case DUAL_RATE -> applyDualRate(value);
        };
    }

    /**
     * Helper method for applying exponential curve with configurable scale.
     *
     * @param value The input value
     * @return The processed value
     */
    private double applyExponential(double value) {
        if (value == 0.0) {
            return 0.0;
        }

        return Math.signum(value)
                * (Math.exp(Math.abs(value) * scale) - 1)
                / (Math.exp(scale) - 1);
    }

    /**
     * Helper method for applying dual rate curve with configurable break point.
     *
     * @param value The input value
     * @return The processed value
     */
    private double applyDualRate(double value) {
        if (Math.abs(value) < dualRateBreak) {
            return value * (1.0 / (1.0 + dualRateBreak)); // Slower in precision range
        } else {
            double normalizedValue =
                    (Math.abs(value) - dualRateBreak) / (1.0 - dualRateBreak);
            double highRangeFactor =
                    dualRateBreak + (1.0 - dualRateBreak) * (1.0 + normalizedValue);
            return Math.signum(value) * highRangeFactor;
        }
    }

    /**
     * Gets the current curve type.
     *
     * @return The curve type
     */
    public CurveType getCurveType() {
        return curveType;
    }

    /**
     * Gets the current deadband value.
     *
     * @return The deadband value
     */
    public double getDeadband() {
        return deadband;
    }
}