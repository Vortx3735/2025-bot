package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.jni.ControlJNI;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import edu.wpi.first.math.controller.PIDController;

public class Elevator extends SubsystemBase {
    private final TalonFX elevatorMotor;
    private final CANcoder elevatorEncoder;

    // Kraken motors placeholder (adjust based on functionality)
    private final Object krakenMotors;

    // PID Constants
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Encoder limits for the elevator
    private static final double MAX_HEIGHT = 100.0; // Maximum encoder value for height
    private static final double MIN_HEIGHT = 0.0;   // Minimum encoder value for height

    public Elevator(int motorCANID, int encoderCANID, Object krakenMotorsInstance) {
        // Initialize motor and encoder
        elevatorMotor = new TalonFX(motorCANID);
        elevatorEncoder = new CANcoder(encoderCANID);

        // Configure PID settings on the motor
        // Assign Kraken motors instance
        this.krakenMotors = krakenMotorsInstance;

        // Initialize encoder settings
    }

    /**
     * Manually control the elevator motor with a percentage output.
     *
     * @param speed The speed to set the motor (-1.0 to 1.0).
     */
    public void manualControl(double speed) {
        // Prevent movement beyond soft limits
        double currentHeight = getElevatorHeight();
        if ((speed > 0 && currentHeight >= MAX_HEIGHT) || (speed < 0 && currentHeight <= MIN_HEIGHT)) {
            elevatorMotor.set(0.0);
        } else {
            elevatorMotor.set(speed);
        }
    }

    /**
     * Move the elevator to a target height using position control.
     *
     * @param targetHeight The target height in encoder units.
     */
    public void moveToHeight(double targetHeight) {
        // Clamp target height within limits
        targetHeight = Math.max(MIN_HEIGHT, Math.min(MAX_HEIGHT, targetHeight));

        // Set motor to position control mode
       
    }
    

    /**
     * Get the current elevator height from the encoder.
     *
     * @return The current height in encoder units.
     */
    public double getElevatorHeight() {
        return elevatorEncoder.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // Called periodically by the scheduler, can be used for logging or updates
        System.out.println("Elevator Height: " + getElevatorHeight());
    }
}
