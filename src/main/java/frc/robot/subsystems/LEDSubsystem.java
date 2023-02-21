package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem to control the robot's LEDs, by determining what number should be encoded to DIO pins and
 * sent to the Arduino we used for controlling the patterns and colors
 */
public class LEDSubsystem extends SubsystemBase {
    private DigitalOutput codeChannel1, codeChannel2, codeChannel3, codeChannel4, codeChannel5;

    private LEDStatusMode currentStatusMode;
    private LEDStatusMode currentDefaultStatusMode;

    private boolean disableLEDs;

    private Timer timer;

    private LEDSubsystem() {
        codeChannel1 = new DigitalOutput(Constants.LEDs.CHANNEL_1_PIN);    // DIO outputs
        codeChannel2 = new DigitalOutput(Constants.LEDs.CHANNEL_2_PIN);
        codeChannel3 = new DigitalOutput(Constants.LEDs.CHANNEL_3_PIN);
        codeChannel4 = new DigitalOutput(Constants.LEDs.CHANNEL_4_PIN);
        codeChannel5 = new DigitalOutput(Constants.LEDs.CHANNEL_5_PIN);

        timer = new Timer();

        currentStatusMode = LEDStatusMode.DISABLED;

        // SmartDashboard.putNumber("LED CODE", 0); // For manually inputting code to encode to DIO pins
    }
    private static LEDSubsystem instance;       // Static that stores the instance of class
    public static LEDSubsystem getInstance() {  // Method to allow calling this class and getting the single instance from anywhere, creating the instance if the first time.
        if (instance == null) {
            instance = new LEDSubsystem();
        }
        return instance;
    }
    
    public static enum LEDStatusMode {
        OFF(0),
        CONE(1),
        CUBE(2),
        DISABLED(3),
        DISABLED_RED_PULSE(4),
        DISABLED_BLUE_PULSE(5);


        private final int code;

        private LEDStatusMode(int code) {
            this.code = code;
        }

        public int getPositionTicks() {
            return code;
        }
    }

    @Override
    public void periodic() {
        int code = 0;

        if(!disableLEDs) {
                if (currentStatusMode == null) {
                    if (currentDefaultStatusMode == LEDStatusMode.DISABLED) {   // If disabled, finds gets the alliance color from the driver station and pulses that. Only pulses color if connected to station or FMS, else pulses default disabled color (Firefl status mode)
                        if (DriverStation.getAlliance() == Alliance.Red) {
                            currentStatusMode = LEDStatusMode.DISABLED_RED_PULSE;
                        } else if (DriverStation.getAlliance() == Alliance.Blue) {
                            currentStatusMode = LEDStatusMode.DISABLED_BLUE_PULSE;
                        } else {
                            currentStatusMode = LEDStatusMode.DISABLED; // Reaches here if DriverStation.getAlliance returns Invalid, which just means it can't determine our alliance and we do cool default effect
                        }
                    } else {
                        currentStatusMode = currentDefaultStatusMode;
                    }
                }

                code = currentStatusMode.code;
        } else {
            code = 0;
        }
        
        // code = (int) SmartDashboard.getNumber("LED CODE", 0); // For manually inputting code to encode to DIO pins

        // SmartDashboard.putBoolean("channel1", (code & 1) > 0);
        // SmartDashboard.putBoolean("channel2", (code & 2) > 0);
        // SmartDashboard.putBoolean("channel3", (code & 4) > 0);
        // SmartDashboard.putBoolean("channel4", (code & 8) > 0);
        // SmartDashboard.putBoolean("channel5", (code & 16) > 0);

        // Code for encoding the code to binary on the digitalOutput pins
        codeChannel1.set((code & 1) > 0);   // 2^0
        codeChannel2.set((code & 2) > 0);   // 2^1
        codeChannel3.set((code & 4) > 0);   // 2^2
        codeChannel4.set((code & 8) > 0);   // 2^3
        codeChannel5.set((code & 16) > 0);  // 2^4

        clearStatusMode(); // Clears status mode after every loop to make sure high priority status modes 
        // don't stick around forever and everything trying to use it has to be activley setting the status mode
    }

    public void setLEDStatusMode(LEDStatusMode statusMode) {
        if (!disableLEDs) {
            currentStatusMode = statusMode;
        }
    }

    public void clearStatusMode() {
        currentStatusMode = LEDStatusMode.OFF;
    }

    // Disables LEDs (turns them off)
    public void disable() {
        disableLEDs = true;
    }

    // Enables LEDs (turns them on)
    public void enable() {
        disableLEDs = false;
    }
}