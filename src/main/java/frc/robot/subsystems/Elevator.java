package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.jni.ControlJNI;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import edu.wpi.first.math.controller.PIDController;


public class Elevator extends SubsystemBase {
    private static TalonFX elevatorMotor1 = new TalonFX(1);
  private static TalonFX elevatorMotor2 = new TalonFX(2);
  private static CANcoder elevatorEncoder1 = new CANcoder(1);
  private static CANcoder elevatorEncoder2 = new CANcoder(2);
  private static PIDController elevatorPID = new PIDController(0.1, 0.1, 0.1);
    // in init function
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // create a Motion Magic request, voltage output
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // set target position to 100 rotations
    m_talonFX.setControl(m_request.withPosition(100));
    m_talonFX.getConfigurator().apply(talonFXConfigs);
    public class Elevator {
    
        public enum ElevatorLevel {
            LEVEL1(0.0),
            LEVEL2(1.0),
            LEVEL3(2.0),
            LEVEL4(3.0);
    
            private final double position;
    
            ElevatorLevel(double position) {
                this.position = position;
            }
    
            public double getPosition() {
                return position;
            }
        }
    
        public void moveToElevatorLevel(ElevatorLevel level) {
            double position = level.getPosition();
            elevatorMotor1.getEncoder().setPosition(position);
            elevatorMotor2.getEncoder().setPosition(position);
        }
    
    }
    elevator.moveToElevatorLevel(ElevatorLevel.LEVEL1);
    elevator.moveToElevatorLevel(ElevatorLevel.LEVEL2);
    elevator.moveToElevatorLevel(ElevatorLevel.LEVEL3);
    elevator.moveToElevatorLevel(ElevatorLevel.LEVEL4);

    public void setElevatorSpeed(double speed) {
        elevatorMotor1.set(speed);
        elevatorMotor2.set(speed);
    }
    public void moveElevatorUp() {
        elevatorMotor1.set(1);
        elevatorMotor2.set(1);
    }
    public void moveElevatorDown() {
        elevatorMotor1.set(-1);
        elevatorMotor2.set(-1);
    }
    public void moveElrvatorToPosition(double position) {
        elevatorMotor1.set(ControlMode.Position, position);
        elevatorMotor2.set(ControlMode.Position, position);
    }

    public void stopElevator() {
        elevatorMotor1.set(0);
        elevatorMotor2.set(0);
    }

    public double getElevatorPosition() {
        return elevatorMotor1.getEncoderPosition();
    }
}
    
   