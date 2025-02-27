package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;

import java.util.HashMap;
import java.util.Map;

/**
 * Subsystem for controlling addressable LEDs.
 * Provides various animation patterns and status indications for the robot.
 */
public class LEDSubsystem extends SubsystemBase {

    // Hardware
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    // Animation
    private LEDAnimation currentAnimation;
    private LEDStatus currentStatus = LEDStatus.IDLE;

    // Game state tracking
    private boolean isEnabled = false;
    private boolean isAutonomous = false;
    private Alliance alliance = Alliance.Blue;

    // Test Mode
    private static final boolean RUN_EFFECTS_TEST = false;
    private long lastEffectChangeTime;
    private Effect currentEffect = Effect.SOLID;
    private static final long EFFECT_DURATION = 10000;

    // Standard colors
    private static final Color ALLIANCE_BLUE = Color.kBlue;
    private static final Color ALLIANCE_RED = Color.kRed;
    private static final Color IDLE_COLOR = Color.kGreen;
    private static final Color ERROR_COLOR = Color.kOrange;
    private static final Color WARNING_COLOR = Color.kYellow;
    private static final Color SUCCESS_COLOR = new Color(0, 1.0, 0);

    // Animation presets
    private final Map<LEDStatus, LEDAnimation> statusAnimations = new HashMap<>();

    /**
     * Represents different LED status modes.
     */
    public enum LEDStatus {
        IDLE,
        SCORING,
        INTAKING,
        SCRUBBING,
        SUCCESS,
        ERROR,
        WARNING,
        CLIMBING,
        ALLIANCE
    }

    /**
     * Enum representing different LED effects.
     */
    public enum Effect {
        SOLID,
        RAINBOW,
        BLINK,
        PINGPONG,
        COLOR_WIPE,
        BREATHING,
        FIRE,
        CHASE;

        /**
         * Gets the next effect in the cycle.
         *
         * @return The next effect.
         */
        public Effect next() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    /**
     * Creates a new LEDSubsystem.
     *
     * @param pwmPort The PWM port the LEDs are connected to.
     * @param ledCount The number of LEDs in the strip.
     */
    public LEDSubsystem(int pwmPort, int ledCount) {
        led = new AddressableLED(pwmPort);
        ledBuffer = new AddressableLEDBuffer(ledCount);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

        // Initialize status animations
        setupStatusAnimations();

        // Default to alliance or idle color
        setStatus(LEDStatus.IDLE);
    }

    /**
     * Sets up the animations for different robot statuses.
     */
    private void setupStatusAnimations() {
        // Initialize status animation map
        statusAnimations.put(LEDStatus.IDLE, new SolidColorAnimation(ledBuffer, IDLE_COLOR));
        statusAnimations.put(LEDStatus.ERROR, new BlinkAnimation(ledBuffer, ERROR_COLOR, 250)); // Fast blink
        statusAnimations.put(LEDStatus.WARNING, new BlinkAnimation(ledBuffer, WARNING_COLOR, 500)); // Medium blink
        statusAnimations.put(LEDStatus.SUCCESS, new SolidColorAnimation(ledBuffer, SUCCESS_COLOR));
        statusAnimations.put(LEDStatus.SCORING, new BreathingAnimation(ledBuffer, SUCCESS_COLOR, 1000));
        statusAnimations.put(LEDStatus.INTAKING, new ChaseAnimation(ledBuffer, IDLE_COLOR, 50));
        statusAnimations.put(LEDStatus.SCRUBBING, new PingPongAnimation(ledBuffer, IDLE_COLOR, 30));
        statusAnimations.put(LEDStatus.CLIMBING, new FireAnimation(ledBuffer, ERROR_COLOR, 30));
        // Alliance status will be set dynamically
    }

    /**
     * Updates alliance status with current alliance color.
     */
    private void updateAllianceAnimation() {
        Color allianceColor = getAllianceColor();
        statusAnimations.put(LEDStatus.ALLIANCE, new BreathingAnimation(ledBuffer, allianceColor, 2000));
    }

    /**
     * Gets the color for the current alliance.
     * 
     * @return The alliance color
     */
    private Color getAllianceColor() {
        return alliance == Alliance.Red ? ALLIANCE_RED : ALLIANCE_BLUE;
    }

    @Override
    public void periodic() {
        // Check for alliance and robot state changes
        updateRobotState();

        // Run the current animation
        if (currentAnimation != null) {
            currentAnimation.update();
            led.setData(ledBuffer);
        }

        // Run the effects test if enabled
        if (RUN_EFFECTS_TEST) {
            long currentTime = System.currentTimeMillis();
            if (currentTime - lastEffectChangeTime >= EFFECT_DURATION) {
                currentEffect = currentEffect.next();
                setEffect(currentEffect);
                lastEffectChangeTime = currentTime;
            }
        }
    }

    /**
     * Updates the robot state information from DriverStation.
     */
    private void updateRobotState() {
        boolean wasEnabled = isEnabled;
        boolean wasAutonomous = isAutonomous;
        Alliance prevAlliance = alliance;
        
        // Update state
        isEnabled = DriverStation.isEnabled();
        isAutonomous = DriverStation.isAutonomous();
        
        // Check if alliance changed
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }
        
        // If state changed, update LEDs if we're showing alliance colors
        if (wasEnabled != isEnabled || wasAutonomous != isAutonomous || prevAlliance != alliance) {
            updateAllianceAnimation();
            if (currentStatus == LEDStatus.ALLIANCE) {
                setStatus(LEDStatus.ALLIANCE);
            }
        }
    }

    /**
     * Sets the LED status, activating the appropriate animation.
     *
     * @param status The status to display
     */
    public void setStatus(LEDStatus status) {
        currentStatus = status;
        if (status == LEDStatus.ALLIANCE) {
            updateAllianceAnimation();
        }
        
        LEDAnimation animation = statusAnimations.get(status);
        if (animation != null) {
            currentAnimation = animation;
            animation.reset();
        } else {
            setSolidColor(IDLE_COLOR);
        }
    }

    /**
     * Sets the LED effect.
     *
     * @param effect The effect to set.
     */
    public void setEffect(Effect effect) {
        switch (effect) {
            case SOLID:
                setSolidColor(IDLE_COLOR);
                break;
            case RAINBOW:
                setRainbow();
                break;
            case BLINK:
                setBlinking(IDLE_COLOR, 500);
                break;
            case PINGPONG:
                setPingPong(IDLE_COLOR, 25);
                break;
            case COLOR_WIPE:
                setColorWipe(getAllianceColor(), 50);
                break;
            case BREATHING:
                setBreathing(getAllianceColor(), 1000);
                break;
            case FIRE:
                setFire(Color.kOrange, 30);
                break;
            case CHASE:
                setChase(IDLE_COLOR, 100);
                break;
        }
    }

    /**
     * Sets the LEDs to a solid color.
     *
     * @param color The color to set.
     */
    public void setSolidColor(Color color) {
        currentAnimation = new SolidColorAnimation(ledBuffer, color);
        currentAnimation.reset();
    }

    /**
     * Creates a command to set the LEDs to a solid color.
     *
     * @param color The color to set
     * @return A command that sets the color
     */
    public Command setSolidColorCommand(Color color) {
        return new InstantCommand(() -> setSolidColor(color), this);
    }

    /**
     * Sets the LEDs to a rainbow pattern.
     */
    public void setRainbow() {
        currentAnimation = new RainbowAnimation(ledBuffer, 128);
        currentAnimation.reset();
    }

    /**
     * Creates a command to set the LEDs to a rainbow pattern.
     *
     * @return A command that sets the rainbow pattern
     */
    public Command setRainbowCommand() {
        return new InstantCommand(this::setRainbow, this);
    }

    /**
     * Sets the LEDs to blink a certain color.
     *
     * @param color The color to blink.
     * @param blinkRate The blink rate in milliseconds.
     */
    public void setBlinking(Color color, int blinkRate) {
        currentAnimation = new BlinkAnimation(ledBuffer, color, blinkRate);
        currentAnimation.reset();
    }

    /**
     * Sets the LEDs to a ping pong animation.
     *
     * @param color The color to use.
     * @param stepsPerSecond The steps per second.
     */
    public void setPingPong(Color color, int stepsPerSecond) {
        currentAnimation = new PingPongAnimation(ledBuffer, color, stepsPerSecond);
        currentAnimation.reset();
    }

    /**
     * Sets the LEDs to a color wipe animation.
     *
     * @param color The color to wipe.
     * @param speed The speed of the wipe in milliseconds per step.
     */
    public void setColorWipe(Color color, int speed) {
        currentAnimation = new ColorWipeAnimation(ledBuffer, color, speed);
        currentAnimation.reset();
    }

    /**
     * Sets the LEDs to a breathing animation.
     *
     * @param color The base color for breathing.
     * @param period The period of one breath in milliseconds.
     */
    public void setBreathing(Color color, int period) {
        currentAnimation = new BreathingAnimation(ledBuffer, color, period);
        currentAnimation.reset();
    }

    /**
     * Sets the LEDs to a fire animation.
     *
     * @param baseColor The base color for the fire effect.
     * @param speed The speed of the animation in milliseconds per update.
     */
    public void setFire(Color baseColor, int speed) {
        currentAnimation = new FireAnimation(ledBuffer, baseColor, speed);
        currentAnimation.reset();
    }

    /**
     * Sets the LEDs to a chase animation.
     *
     * @param color The color to chase.
     * @param speed The speed of the chase in milliseconds per step.
     */
    public void setChase(Color color, int speed) {
        currentAnimation = new ChaseAnimation(ledBuffer, color, speed);
        currentAnimation.reset();
    }

    /**
     * Creates a command to set the status.
     *
     * @param status The status to set
     * @return A command that sets the status
     */
    public Command setStatusCommand(LEDStatus status) {
        return new InstantCommand(() -> setStatus(status), this);
    }

    /**
     * Stops the current animation and sets the LEDs to black.
     */
    public void stopAnimation() {
        setSolidColor(Color.kBlack);
    }

    /**
     * Base class for LED animations.
     */
    private abstract class LEDAnimation {
        protected AddressableLEDBuffer ledBuffer;

        /**
         * Creates a new LEDAnimation.
         *
         * @param ledBuffer The LED buffer to use.
         */
        public LEDAnimation(AddressableLEDBuffer ledBuffer) {
            this.ledBuffer = ledBuffer;
        }

        /**
         * Updates the animation.
         */
        public abstract void update();

        /**
         * Resets the animation to its initial state.
         */
        public void reset() {
            // Default implementation does nothing
        }
    }

    /**
     * Animation for solid color.
     */
    private class SolidColorAnimation extends LEDAnimation {
        private Color color;

        /**
         * Creates a new SolidColorAnimation.
         *
         * @param ledBuffer The LED buffer to use.
         * @param color The color to display.
         */
        public SolidColorAnimation(AddressableLEDBuffer ledBuffer, Color color) {
            super(ledBuffer);
            this.color = color;
        }

        @Override
        public void update() {
            LEDPattern.solid(color).applyTo(ledBuffer);
        }

        @Override
        public void reset() {
            update();
        }
    }

    /**
     * Animation for rainbow pattern.
     */
    private class RainbowAnimation extends LEDAnimation {
        private int saturation;
        private int offset = 0;

        /**
         * Creates a new RainbowAnimation.
         *
         * @param ledBuffer The LED buffer to use.
         * @param saturation The saturation of the rainbow.
         */
        public RainbowAnimation(AddressableLEDBuffer ledBuffer, int saturation) {
            super(ledBuffer);
            this.saturation = saturation;
        }

        @Override
        public void update() {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                int hue = (offset + (i * 180 / ledBuffer.getLength())) % 180;
                ledBuffer.setHSV(i, hue, saturation, 255);
            }
            offset = (offset + 1) % 180;
        }
    }

    /**
     * Blink animation for the LEDs.
     */
    private class BlinkAnimation extends LEDAnimation {
        private Color color;
        private int blinkRate;
        private long lastUpdateTime;
        private boolean on = true;

        /**
         * Creates a new BlinkAnimation.
         *
         * @param ledBuffer The LED buffer to use.
         * @param color The color to blink.
         * @param blinkRate The blink rate in milliseconds.
         */
        public BlinkAnimation(AddressableLEDBuffer ledBuffer, Color color, int blinkRate) {
            super(ledBuffer);
            this.color = color;
            this.blinkRate = blinkRate;
            this.lastUpdateTime = System.currentTimeMillis();
        }

        @Override
        public void update() {
            long currentTime = System.currentTimeMillis();
            if (currentTime - lastUpdateTime >= blinkRate) {
                on = !on;
                Color displayColor = on ? color : Color.kBlack;
                LEDPattern.solid(displayColor).applyTo(ledBuffer);
                lastUpdateTime = currentTime;
            }
        }

        @Override
        public void reset() {
            on = true;
            lastUpdateTime = System.currentTimeMillis();
            LEDPattern.solid(color).applyTo(ledBuffer);
        }
    }

    /**
     * Ping Pong animation for the LEDs.
     */
    private class PingPongAnimation extends LEDAnimation {
        private Color color;
        private int position;
        private boolean direction; // true for forward, false for backward
        private long lastUpdateTime;
        private long updateInterval;

        /**
         * Creates a new PingPongAnimation.
         *
         * @param ledBuffer The LED buffer to use.
         * @param color The color to use.
         * @param stepsPerSecond The steps per second.
         */
        public PingPongAnimation(AddressableLEDBuffer ledBuffer, Color color, int stepsPerSecond) {
            super(ledBuffer);
            this.color = color;
            this.position = 0;
            this.direction = true;
            this.lastUpdateTime = System.currentTimeMillis();
            this.updateInterval = 1000 / stepsPerSecond; // Convert steps per second to milliseconds per step
        }

        @Override
        public void update() {
            long currentTime = System.currentTimeMillis();
            if (currentTime - lastUpdateTime >= updateInterval) {
                // Clear the buffer and set the current position
                LEDPattern.solid(Color.kBlack).applyTo(ledBuffer);
                ledBuffer.setLED(position, color);

                // Update the position and direction
                position += direction ? 1 : -1;
                if (position >= ledBuffer.getLength() || position < 0) {
                    direction = !direction;
                    position = Math.max(0, Math.min(position, ledBuffer.getLength() - 1));
                }

                lastUpdateTime = currentTime;
            }
        }

        @Override
        public void reset() {
            position = 0;
            direction = true;
            lastUpdateTime = System.currentTimeMillis();
            LEDPattern.solid(Color.kBlack).applyTo(ledBuffer);
            ledBuffer.setLED(position, color);
        }
    }

    /**
     * Color wipe animation that fills LEDs one by one.
     */
    private class ColorWipeAnimation extends LEDAnimation {
        private Color color;
        private int position;
        private long lastUpdateTime;
        private long updateInterval;
        private boolean reverse = false;

        /**
         * Creates a new ColorWipeAnimation.
         *
         * @param ledBuffer The LED buffer to use.
         * @param color The color to wipe with.
         * @param speed The speed in milliseconds per step.
         */
        public ColorWipeAnimation(AddressableLEDBuffer ledBuffer, Color color, int speed) {
            super(ledBuffer);
            this.color = color;
            this.position = 0;
            this.lastUpdateTime = System.currentTimeMillis();
            this.updateInterval = speed;
        }

        @Override
        public void update() {
            long currentTime = System.currentTimeMillis();
            if (currentTime - lastUpdateTime >= updateInterval) {
                if (!reverse) {
                    if (position < ledBuffer.getLength()) {
                        ledBuffer.setLED(position, color);
                        position++;
                    } else {
                        reverse = true;
                        position = ledBuffer.getLength() - 1;
                    }
                } else {
                    if (position >= 0) {
                        ledBuffer.setLED(position, Color.kBlack);
                        position--;
                    } else {
                        reverse = false;
                        position = 0;
                    }
                }
                lastUpdateTime = currentTime;
            }
        }

        @Override
        public void reset() {
            position = 0;
            reverse = false;
            lastUpdateTime = System.currentTimeMillis();
            LEDPattern.solid(Color.kBlack).applyTo(ledBuffer);
        }
    }

    /**
     * Breathing animation that fades in and out.
     */
    private class BreathingAnimation extends LEDAnimation {
        private Color color;
        private long period;
        private long startTime;

        /**
         * Creates a new BreathingAnimation.
         *
         * @param ledBuffer The LED buffer to use.
         * @param color The base color for breathing.
         * @param period The period of one breath in milliseconds.
         */
        public BreathingAnimation(AddressableLEDBuffer ledBuffer, Color color, long period) {
            super(ledBuffer);
            this.color = color;
            this.period = period;
            this.startTime = System.currentTimeMillis();
        }

        @Override
        public void update() {
            long currentTime = System.currentTimeMillis();
            double phase = ((currentTime - startTime) % period) / (double) period;
            
            // Sine wave for smooth breathing effect
            double brightness = 0.5 * (1.0 + Math.sin(2 * Math.PI * phase));
            
            Color adjustedColor = new Color(
                color.red * brightness, 
                color.green * brightness, 
                color.blue * brightness
            );
            
            LEDPattern.solid(adjustedColor).applyTo(ledBuffer);
        }

        @Override
        public void reset() {
            startTime = System.currentTimeMillis();
        }
    }

    /**
     * Fire animation effect.
     */
    private class FireAnimation extends LEDAnimation {
        private Color baseColor;
        private long updateInterval;
        private long lastUpdateTime;
        private double[] heat;

        /**
         * Creates a new FireAnimation.
         *
         * @param ledBuffer The LED buffer to use.
         * @param baseColor The base color for the fire effect.
         * @param speed The speed in milliseconds per update.
         */
        public FireAnimation(AddressableLEDBuffer ledBuffer, Color baseColor, int speed) {
            super(ledBuffer);
            this.baseColor = baseColor;
            this.updateInterval = speed;
            this.lastUpdateTime = System.currentTimeMillis();
            this.heat = new double[ledBuffer.getLength()];
            
            // Initialize heat array
            for (int i = 0; i < heat.length; i++) {
                heat[i] = 0;
            }
        }

        @Override
        public void update() {
            long currentTime = System.currentTimeMillis();
            if (currentTime - lastUpdateTime >= updateInterval) {
                // Simulate fire cooling
                for (int i = 0; i < heat.length; i++) {
                    heat[i] = Math.max(0, heat[i] - Math.random() * 0.1);
                }
                
                // Heat from bottom
                for (int i = 0; i < 3; i++) {
                    if (i < heat.length) {
                        heat[i] = Math.min(1.0, heat[i] + Math.random() * 0.4);
                    }
                }
                
                // Heat propagation
                for (int i = heat.length - 1; i >= 3; i--) {
                    heat[i] = (heat[i - 1] + heat[i - 2] + heat[i - 3]) / 3.0;
                }
                
                // Set colors based on heat
                for (int i = 0; i < heat.length; i++) {
                    double h = heat[i];
                    
                    // Scale colors based on heat
                    Color pixelColor = new Color(
                        baseColor.red * h,
                        baseColor.green * Math.max(0, h - 0.5) * 2,
                        baseColor.blue * Math.max(0, h - 0.75) * 4
                    );
                    
                    ledBuffer.setLED(i, pixelColor);
                }
                
                lastUpdateTime = currentTime;
            }
        }

        @Override
        public void reset() {
            for (int i = 0; i < heat.length; i++) {
                heat[i] = 0;
            }
            lastUpdateTime = System.currentTimeMillis();
        }
    }

    /**
     * Chase animation for the LEDs.
     */
    private class ChaseAnimation extends LEDAnimation {
        private Color color;
        private int position;
        private int chaseLength;
        private long lastUpdateTime;
        private long updateInterval;

        /**
         * Creates a new ChaseAnimation.
         *
         * @param ledBuffer The LED buffer to use.
         * @param color The color to chase.
         * @param speed The speed in milliseconds per step.
         */
        public ChaseAnimation(AddressableLEDBuffer ledBuffer, Color color, int speed) {
            super(ledBuffer);
            this.color = color;
            this.position = 0;
            this.lastUpdateTime = System.currentTimeMillis();
            this.updateInterval = speed;
            this.chaseLength = Math.max(3, ledBuffer.getLength() / 8);
        }

        @Override
        public void update() {
            long currentTime = System.currentTimeMillis();
            if (currentTime - lastUpdateTime >= updateInterval) {
                LEDPattern.solid(Color.kBlack).applyTo(ledBuffer);
                
                for (int i = 0; i < chaseLength; i++) {
                    int pixelPos = (position + i) % ledBuffer.getLength();
                    
                    // Fade intensity based on position in chase
                    double intensity = 1.0 - ((double)i / chaseLength);
                    Color pixelColor = new Color(
                        color.red * intensity,
                        color.green * intensity,
                        color.blue * intensity
                    );
                    
                    ledBuffer.setLED(pixelPos, pixelColor);
                }
                
                // Move position for next update
                position = (position + 1) % ledBuffer.getLength();
                lastUpdateTime = currentTime;
            }
        }

        @Override
        public void reset() {
            position = 0;
            lastUpdateTime = System.currentTimeMillis();
            LEDPattern.solid(Color.kBlack).applyTo(ledBuffer);
        }
    }
}