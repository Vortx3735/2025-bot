package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    private final TalonFX climbMotor1;
    private final TalonFX climbMotor2;

    private static final double CLIMB_MAX_POSITION = 10000; // Placeholder encoder ticks
    private static final double PITCH_THRESHOLD = 30.0; // Degrees, placeholder

    public ClimbSubsystem(int motorPort1, int motorPort2) {
        climbMotor1 = new TalonFX(motorPort1);
        climbMotor2 = new TalonFX(motorPort2);
        
        // Configure second motor to follow the first
        climbMotor2.setControl(new Follower(climbMotor1.getDeviceID(), false));
        
        setBrakeMode();
    }

    public void setSpeed(double speed) {
        if (shouldStop(speed)) {
            stopMotor();
            return;
        }
        climbMotor1.setControl(new DutyCycleOut(speed));
    }

    private boolean shouldStop(double speed) {
        double position = getPosition();
        if (speed > 0) { // Extending
            return position >= CLIMB_MAX_POSITION;
        }
        return false; // Add lower limit check if needed
    }

    public void stopMotor() {
        climbMotor1.stopMotor();
    }

    public void setBrakeMode() {
        climbMotor1.setNeutralMode(NeutralModeValue.Brake);
        climbMotor2.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setCoastMode() {
        climbMotor1.setNeutralMode(NeutralModeValue.Coast);
        climbMotor2.setNeutralMode(NeutralModeValue.Coast);
    }

    public double getMotorSpeed() {
        return climbMotor1.get() * climbMotor1.getDeviceID(); // Returns percent output
    }

    public double getPosition() {
        return climbMotor1.getPosition().getValue(); // Encoder position
    }

    public double getVelocity() {
        return climbMotor1.getVelocity().getValue(); // Encoder velocity
    }
}