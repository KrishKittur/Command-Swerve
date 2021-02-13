package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

// Analog encoder class
public class AnalogEncoder {

    // Global variables
    private final AnalogInput input;
    private final double offset;

    // Constructor
    public AnalogEncoder(int channel, double offset) {
        input = new AnalogInput(channel);
        this.offset = offset;
    }

    // Method to read the encoder
    public double get() {
        double angle = ((input.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI) - offset;
        if (angle < 0.0) {
            angle = 2.0 * Math.PI - Math.abs(angle);
        }
        if (angle > Math.PI) {
            angle = angle - 2.0 * Math.PI;
        }
        return -angle;
    }

    // Method to get the inputs raw value
    public double getRaw() {
        return input.getVoltage() / RobotController.getVoltage5V();
    }

    // Method to get the inputs raw angle
    public double getRawAngle() {
        return getRaw() * 2.0 * Math.PI;
    }
    
}
