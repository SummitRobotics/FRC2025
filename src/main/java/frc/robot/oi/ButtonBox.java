package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Represents the Arduino Leonardo-driven button box. */
public class ButtonBox extends GenericHID {

    public enum Button {

        STOW_LOWER_PRESET(1, "Stow Lower"),
        STOW_UPPER_PRESET(2, "Stow Upper"),
        RECEIVE_PRESET(3, "Receive"),
        L1_PRESET(4, "L1"),
        L2_PRESET(5, "L2"),
        L3_PRESET(7, "L3"),
        L4_PRESET(8, "L4"),
        L4_BACKWARDS(9, "L4B"),
        MO_PRESET(10, "MO"), // Missile switch 1
        GO_PRESET(6, "Go");
        public int index;
        public String name;

        Button(int index, String name) {
            this.index = index;
            this.name = name;
        }
    }

    private int ledState;
    private static int kLedMask = 0b111111111;

    public ButtonBox(int port) {
        super(port);
    }

    /**
     * Set the LED status for a given button. All Button LED's are updated in
     * SendMessage and persist until this method is updated.
     *
     * @param button The button to set LED status.
     * @param on     True to turn LED on, otherwise off.
     */
    public void LED(Button button, boolean on) {
        int ledIndex = button.index - 1;
        if (on) {
            this.ledState |= 1 << ledIndex;
        } else {
            this.ledState &= ~(1 << ledIndex);
        }
    }

    /**
     * Set all LEDs to either ON or OFF.
     *
     * @param on True to turn LEDs on, otherwise off.
     */
    public void allLED(boolean on) {
        this.ledState = on ? kLedMask : 0;
    }

    /**
     * Sends the queued message to the ButtonBox arduino to decode.
     */
    public void sendMessage() {
        // TODO: Only updating LEDs. See Buttonbox/README.md for format of message
        this.setOutputs(this.ledState & kLedMask);
    }

    /** Generates trigger for binding purposes. */
    public Trigger getTrigger(Button button) {
        return new Trigger(() -> getRawButton(button.index));
    }

    /*
     * Flash the lights on the button box.
     */
    public void flashAllButtonBoxLights() {
      allLED(true);
      new java.util.Timer().schedule(
        new java.util.TimerTask() {
            @Override
            public void run() {
              allLED(false);
            }
        },
    500);
    }
}
