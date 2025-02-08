package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public static TalonFX leftElevatorMotor;
  public static TalonFX rightElevatorMotor;
  private static CANcoder elevatorEncoder;
  private static PIDController elevatorPIDController = new PIDController(0.1, 0.1, 0.1);

  public Elevator(int encoderID, int leftMotorID, int rightMotorID) {
    elevatorEncoder = new CANcoder(encoderID);
    leftElevatorMotor = new TalonFX(leftMotorID);
    rightElevatorMotor = new TalonFX(rightMotorID);

    configureCANcoder();
    configureTalonFX();
    logPositions();

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
    motionMagicConfigs.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    leftElevatorMotor.getConfigurator().apply(talonFXConfigs);
    // create a Motion Magic request, voltage output
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // set target position to 100 rotations
    leftElevatorMotor.setControl(m_request.withPosition(100));
    leftElevatorMotor.getConfigurator().apply(talonFXConfigs);
  }

  public double getSelectedSensorPosition() {
    double position1 = leftElevatorMotor.getPosition().getValueAsDouble();
    double position2 = rightElevatorMotor.getPosition().getValueAsDouble();
    // Assuming you want to return the average position of both motors
    return (position1 + position2) / 2.0;
  }

  // This method configures the CANcoder sensor with specific settings for the absolute sensor
  // discontinuity point and magnet offset, and applies these settings to the elevatorEncoder
  private void configureCANcoder() {
    try {
      CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
      cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
          0.0; // Set to an appropriate double value
      cc_cfg.MagnetSensor.MagnetOffset = 0.4;
      elevatorEncoder.getConfigurator().apply(cc_cfg);
    } catch (Exception e) {
      System.err.println("Failed to configure CANcoder: " + e.getMessage());
    }
  }

  private void configureTalonFX() {
    try {
      TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
      fx_cfg.Feedback.FeedbackRemoteSensorID = elevatorEncoder.getDeviceID();
      fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      fx_cfg.Feedback.SensorToMechanismRatio = 1.0;
      fx_cfg.Feedback.RotorToSensorRatio = 12.8;
      leftElevatorMotor.getConfigurator().apply(fx_cfg);
      rightElevatorMotor.getConfigurator().apply(fx_cfg);
    } catch (Exception e) {
      System.err.println("Failed to configure TalonFX: " + e.getMessage());
    }
  }

  private void logPositions() {
    try {
      leftElevatorMotor.getPosition().getValueAsDouble();
      rightElevatorMotor.getPosition().getValueAsDouble();
      elevatorEncoder.getAbsolutePosition();
      System.out.println("FX Position: " + leftElevatorMotor.getPosition().getValueAsDouble());
      System.out.println("CANcoder Position: " + elevatorEncoder.getPosition());
    } catch (Exception e) {
      System.err.println("Failed to log positions: " + e.getMessage());
    }
  }

  // Define soft limits
  private static final double LOWERLIMIT = 0.0;
  private static final double UPPERLIMIT = 5.0;

  public enum ElevatorLevel {
    LEVEL1(1.0),
    LEVEL2(2.0),
    LEVEL3(3.0),
    LEVEL4(4.0);

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
    moveElevatorToPosition(position);
  }

  // This method sets both leftElevatorMotor and rightElevatorMotor to move to the specified
  // position
  // using a motion control algorithm (MotionMagicVoltage). This ensures that both motors move
  // in a coordinated manner to reach the desired position
  public void moveElevatorToPosition(double position) {
    leftElevatorMotor.setControl(new MotionMagicVoltage(0).withPosition(position));
    rightElevatorMotor.setControl(new MotionMagicVoltage(0).withPosition(position));
  }

  public void setElevatorSpeed(double speed) {
    leftElevatorMotor.set(speed);
    rightElevatorMotor.set(speed);
  }

  public void moveElevatorUp() {
    leftElevatorMotor.set(1);
    rightElevatorMotor.set(1);
  }

  public void moveElevatorDown() {
    leftElevatorMotor.set(-1);
    rightElevatorMotor.set(-1);
  }

  public void moveToPostitionLEVEL1() {
    moveElevatorToPosition(ElevatorLevel.LEVEL1.getPosition());
  }

  public void moveToPostitionLEVEL2() {
    moveElevatorToPosition(ElevatorLevel.LEVEL2.getPosition());
  }

  public void moveToPostitionLEVEL3() {
    moveElevatorToPosition(ElevatorLevel.LEVEL3.getPosition());
  }

  public void moveToPostitionLEVEL4() {
    moveElevatorToPosition(ElevatorLevel.LEVEL4.getPosition());
  }

  public void stopElevator() {
    leftElevatorMotor.set(0);
    rightElevatorMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double position = elevatorEncoder.getAbsolutePosition().getValueAsDouble();
    // Checks if the elevator is is pass limits
    if (position <= LOWERLIMIT || position >= UPPERLIMIT) {
      stopElevator();
      System.out.println("Elevator position out of bounds: " + position);
    } else {
      System.out.println("Elevator position: " + position);
    }
  }
}
