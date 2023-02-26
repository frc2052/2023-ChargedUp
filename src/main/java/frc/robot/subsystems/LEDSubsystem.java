package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Dashboard;
import frc.robot.io.Dashboard.Autos;

/**
 * Subsystem to control the robot's LEDs, by determining what number should be encoded to DIO pins and
 * sent to the Arduino we used for controlling the patterns and colors
 */
public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem INSTANCE;
    
    private DigitalOutput codeChannel1, codeChannel2, codeChannel3, codeChannel4, codeChannel5, codeChannel6, codeChannel7, codeChannel8;

    private LEDStatusMode currentStatusMode = LEDStatusMode.OFF;

    private boolean disableLEDs;
    private boolean robotDisabled;

    private LEDSubsystem() {
        // DIO outputs
        codeChannel1 = new DigitalOutput(Constants.LEDs.CHANNEL_1_PIN);
        codeChannel2 = new DigitalOutput(Constants.LEDs.CHANNEL_2_PIN);
        codeChannel3 = new DigitalOutput(Constants.LEDs.CHANNEL_3_PIN);
        codeChannel4 = new DigitalOutput(Constants.LEDs.CHANNEL_4_PIN);
        codeChannel5 = new DigitalOutput(Constants.LEDs.CHANNEL_5_PIN);
        codeChannel6 = new DigitalOutput(Constants.LEDs.CHANNEL_6_PIN);
        codeChannel7 = new DigitalOutput(Constants.LEDs.CHANNEL_7_PIN);
        codeChannel8 = new DigitalOutput(Constants.LEDs.CHANNEL_8_PIN);
        robotDisabled = true;

        // SmartDashboard.putNumber("LED CODE", 0); // For manually inputing code to encode to DIO pins
    }

    public static LEDSubsystem getInstance() {  // Method to allow calling this class and getting the single instance from anywhere, creating the instance if the first time.
        if (INSTANCE == null) {
            INSTANCE = new LEDSubsystem();
        }
        return INSTANCE;
    }

    @Override
    public void periodic() {
        int code = 0;
        if(!disableLEDs) {
                if (robotDisabled) {   // If disabled, finds gets the alliance color from the driver station and pulses that. Only pulses color if connected to station or FMS, else pulses default disabled color (Firefl status mode)

                    Autos selected = Dashboard.getInstance().getAuto();
                    if (selected == Autos.NO_AUTO){
                        currentStatusMode = LEDStatusMode.NO_AUTO;
                    }
                    else if (DriverStation.getAlliance() == Alliance.Red) {
                        currentStatusMode = LEDStatusMode.DISABLED_RED_PULSE;
                    } else if (DriverStation.getAlliance() == Alliance.Blue) {
                        currentStatusMode = LEDStatusMode.DISABLED_BLUE_PULSE;
                    } else {
                        currentStatusMode = LEDStatusMode.OFF; // Reaches here if DriverStation.getAlliance returns Invalid, which just means it can't determine our alliance and we do cool default effect
                    }
                }

                code = currentStatusMode.code;

        } else {
            //LEDs are disabled
            code = 0;
        }

        // Code for encoding the code to binary on the digitalOutput pins
        //System.out.println("Sending LED Code" + code + "************************");
        codeChannel1.set((code & 1) > 0);   // 2^0
        codeChannel2.set((code & 2) > 0);   // 2^1
        codeChannel3.set((code & 4) > 0);   // 2^2
        codeChannel4.set((code & 8) > 0);   // 2^3
        codeChannel5.set((code & 16) > 0);  // 2^4
        codeChannel6.set((code & 32) > 0);
        codeChannel7.set((code & 64) > 0); 
        codeChannel8.set((code & 128) > 0);       
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
    public void disableLEDs() {
        disableLEDs = true;
    }

    // Enables LEDs (turns them on)
    public void enableLEDs() {
        disableLEDs = false;
    }

    public void robotDisabled(){
        robotDisabled = true;
    }

    public void robotEnabled(){
        robotDisabled = false;
    }

    public static enum LEDStatusMode {
        OFF(0),
        CONE(1),
        CUBE(2),
        DISABLED_RED_PULSE(3),
        DISABLED_BLUE_PULSE(4),
        NO_AUTO(5);

        private final int code;

        private LEDStatusMode(int code) {
            this.code = code;
        }

        public int getPositionTicks() {
            return code;
        }
    }
}