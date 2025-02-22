package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;


public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private LEDAnimation currentAnimation;

    // For testing purposes, we will cycle through the effects
    private static final boolean runEffectsTest = false;
    private long lastEffectChangeTime;
    private Effect currentEffect = Effect.SOLID;
    private static final long EFFECT_DURATION = 10000;

    public LEDSubsystem(int pwmPort, int ledCount) {
        led = new AddressableLED(pwmPort);
        ledBuffer = new AddressableLEDBuffer(ledCount);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

        // Default to Green
        setSolidColor(Color.kGreen);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (currentAnimation != null) {
            currentAnimation.update();
            led.setData(ledBuffer);
        }

        // Run the effects test if enabled
        if (runEffectsTest) {
            long currentTime = System.currentTimeMillis();
            if (currentTime - lastEffectChangeTime >= EFFECT_DURATION) {
                currentEffect = currentEffect.next();
                setEffect(currentEffect);
                lastEffectChangeTime = currentTime;
            }
        }
    }

    // Enumeration for the different effects
    private enum Effect {
        SOLID,
        RAINBOW,
        BLINK,
        PINGPONG;

        // Get the next effect in the cycle
        public Effect next() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    private void setEffect(Effect effect) {
        switch (effect) {
            case SOLID:
                setSolidColor(Color.kGreen);
                break;
            case RAINBOW:
                setRainbow();
                break;
            case BLINK:
                setBlinking(Color.kGreen, 500);
                break;
            case PINGPONG:
                setPingPong(Color.kGreen, 25);
                break;
        }
    }

    public void setSolidColor(Color color) {
        currentAnimation = null;
        LEDPattern.solid(color).applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    public void setRainbow() {
        currentAnimation = null;
        LEDPattern.rainbow(255, 128).applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    public void setBlinking(Color color, int blinkRate) {
        currentAnimation = new BlinkAnimation(ledBuffer, color, blinkRate);
    }

    public void setPingPong(Color color, int stepsPerSecond) {
        currentAnimation = new PingPongAnimation(ledBuffer, color, stepsPerSecond);
    }

    public void stopAnimation() {
        setSolidColor(Color.kBlack);
    }

    // Base Animation Class
    private abstract class LEDAnimation {
        protected AddressableLEDBuffer ledBuffer;

        public LEDAnimation(AddressableLEDBuffer ledBuffer) {
            this.ledBuffer = ledBuffer;
        }

        public abstract void update();
    }

    // Blink Animation Class
    private class BlinkAnimation extends LEDAnimation {
        private Color color;
        private int blinkRate;
        private long lastUpdateTime;

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
                boolean on = (currentTime / blinkRate) % 2 == 0;
                Color color = on ? this.color : Color.kBlack;
                LEDPattern.solid(color).applyTo(ledBuffer);
                lastUpdateTime = currentTime;
            }
        }
    }

    // Ping Pong Animation Class
    private class PingPongAnimation extends LEDAnimation {
        private Color color;
        private int position;
        private boolean direction; // true for forward, false for backward
        private long lastUpdateTime;
        private long updateInterval;

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
    }
}