package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import java.util.function.BooleanSupplier;

public class Elevator extends SubsystemBase {
  public static TalonFX leftElevatorMotor;
  public static TalonFX rightElevatorMotor;
  private static CANcoder elevatorEncoder;

  public double position;
  public double krakenPosition;
  public double elevatorSpeed;

  // PID values
  private double kP, kI, kD, kV, kA, kG;

  // Motion Magic values
  private double cruiseVelocity, acceleration, jerk;

  // create a Motion Magic request, voltage output
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  // Define soft limits
  public static double LOWER_LIMIT = 0.5;
  public static double UPPER_LIMIT = 5;

  // public static final double KRAKEN_LOWER_LIMIT = 0;
  // public static final double KRAKEN_UPPER_LIMIT = 5;

  /**
   * @param encoderID CAN ID of the CANcoder.
   * @param leftMotorID CAN ID of the left elevator motor.
   * @param rightMotorID CAN ID of the right elevator motor.
   */
  public Elevator(int encoderID, int leftMotorID, int rightMotorID) {
    elevatorEncoder = new CANcoder(encoderID);
    leftElevatorMotor = new TalonFX(leftMotorID);
    rightElevatorMotor = new TalonFX(rightMotorID);

    configureCANcoder();
    configureTalonFX();
    logPositions();

    elevatorSpeed = 0.2;
  }

  /** Returns the average of the two elevator motor positions */
  public double getSelectedSensorPosition() {
    double position1 = leftElevatorMotor.getPosition().getValueAsDouble();
    double position2 = rightElevatorMotor.getPosition().getValueAsDouble();
    return (position1 + position2) / 2.0;
  }

  /**
   * This method configures the CANcoder sensor with specific settings for the absolute sensor
   * discontinuity point and magnet offset, and applies these settings to the ElevatorEncoder
   */
  private void configureCANcoder() {
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        0.0; // Set to an appropriate double value
    cc_cfg.MagnetSensor.MagnetOffset = 0.4;
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    elevatorEncoder.getConfigurator().apply(cc_cfg);
  }

  public void configureTalonFX() {
    TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
    fx_cfg.Feedback.FeedbackRemoteSensorID = elevatorEncoder.getDeviceID();
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    fx_cfg.Feedback.SensorToMechanismRatio = 1.0;
    fx_cfg.Feedback.RotorToSensorRatio = 15;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Set PID & Feedforward gains

    fx_cfg.Slot0.kP = 0.05;
    fx_cfg.Slot0.kI = 0.0;
    fx_cfg.Slot0.kD = 0.1;
    fx_cfg.Slot0.kV = 0.12;
    fx_cfg.Slot0.kA = 0.01;
    fx_cfg.Slot0.kG = 0.01;

    // Motion Magic settings
    fx_cfg.MotionMagic.MotionMagicCruiseVelocity = 80;
    fx_cfg.MotionMagic.MotionMagicAcceleration = 160;
    fx_cfg.MotionMagic.MotionMagicJerk = 1600;

    leftElevatorMotor.getConfigurator().apply(fx_cfg);
    rightElevatorMotor.getConfigurator().apply(fx_cfg);
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

  public void moveToElevatorLevel(double position) {
    moveElevatorToPosition(position);
  }

  /**
   * Moves the elevator to the specified position using Motion Magic control
   *
   * @param targetPosition The target position to move the elevator to
   */
  public void moveElevatorToPosition(double targetPosition) {
    // Prevent moving past soft limits

    // Motion Magic    
    leftElevatorMotor.setControl(m_request.withPosition(targetPosition));
    rightElevatorMotor.setControl(m_request.withPosition(targetPosition));

    // Manual PID
    // if (Math.abs(targetPosition-position)<0.05){
    //   stopElevator();
    // }
    // else if (targetPosition>position){
    //   moveElevatorUpSlow();
    // }
    // else if (targetPosition<position){
    //   moveElevatorDownSlow();
    // }
    // else{
    //   stopElevator();
    // }
  }

  public void moveElevatorUp() {
    if (position <= UPPER_LIMIT) {
      leftElevatorMotor.set(elevatorSpeed);
      rightElevatorMotor.set(elevatorSpeed);
    } else {
      stopElevator();
    }
  }
  public void moveElevatorUpSlow() {
    if (position <= UPPER_LIMIT) {
      leftElevatorMotor.set(0.1);
      rightElevatorMotor.set(0.1);
    } else {
      stopElevator();
    }
  }

  public void moveElevatorDown() {
    if (position >= LOWER_LIMIT) {
      leftElevatorMotor.set(-elevatorSpeed);
      rightElevatorMotor.set(-elevatorSpeed);
    } else {
      stopElevator();
    }
  }
  public void moveElevatorDownSlow() {
    if (position >= LOWER_LIMIT) {
      leftElevatorMotor.set(-0.1);
      rightElevatorMotor.set(-0.1);
    } else {
      stopElevator();
    }
  }

  public void setElevatorSpeed(double speed) {
    leftElevatorMotor.set(speed);
    rightElevatorMotor.set(speed);
  }

  public void moveToPostitionLEVEL1() {
    moveElevatorToPosition(ElevatorConstants.LEVEL_1);
  }

  public void moveToPostitionLEVEL2() {
    moveElevatorToPosition(ElevatorConstants.LEVEL_2);
  }

  public void moveToPostitionLEVEL3() {
    moveElevatorToPosition(ElevatorConstants.LEVEL_3);
  }

  public void moveToPostitionLEVEL4() {
    moveElevatorToPosition(ElevatorConstants.LEVEL_4);
  }

  public void stopElevator() {
    leftElevatorMotor.set(0);
    rightElevatorMotor.set(0);
  }

  public void setBrakeMode() {
    leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void updateTalonFxConfigs() {
    // Apply new PID values if changed
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.Slot0.kP = kP;
    talonFXConfigs.Slot0.kI = kI;
    talonFXConfigs.Slot0.kD = kD;
    // talonFXConfigs.Slot0.kS = kS;
    talonFXConfigs.Slot0.kV = kV;
    talonFXConfigs.Slot0.kA = kA;
    talonFXConfigs.Slot0.kG = kG;

    // Apply new Motion Magic settings if changed
    talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
    talonFXConfigs.MotionMagic.MotionMagicAcceleration = acceleration;
    talonFXConfigs.MotionMagic.MotionMagicJerk = jerk;

    leftElevatorMotor.getConfigurator().apply(talonFXConfigs);
    rightElevatorMotor.getConfigurator().apply(talonFXConfigs);
  }

  public BooleanSupplier isPastLowerLimit() {
    if (leftElevatorMotor.getPosition().getValueAsDouble() <= LOWER_LIMIT) {
      return () -> true;
    }
    return () -> false;
  }

  public BooleanSupplier isPastUpperLimit() {
    if (leftElevatorMotor.getPosition().getValueAsDouble() >= UPPER_LIMIT) {
      return () -> true;
    }
    return () -> false;
  }

  public void publishInitialValues() {
    SmartDashboard.putNumber("elevator/kP", kP);
    SmartDashboard.putNumber("elevator/kI", kI);
    SmartDashboard.putNumber("elevator/kD", kD);
    // SmartDashboard.putNumber("elevator/kS", kS);
    SmartDashboard.putNumber("elevator/kV", kV);
    SmartDashboard.putNumber("elevator/kA", kA);
    SmartDashboard.putNumber("elevator/kG", kG);
    SmartDashboard.putNumber("elevator/cruiseVelocity", cruiseVelocity);
    SmartDashboard.putNumber("elevator/acceleration", acceleration);
    SmartDashboard.putNumber("elevator/jerk", jerk);
    SmartDashboard.putNumber("elevator/Elevator Speed", elevatorSpeed);
    SmartDashboard.putNumber("UpperLimit", UPPER_LIMIT);
    SmartDashboard.putNumber("LowerLimit", LOWER_LIMIT);
  }

  @Override
  public void periodic() {
    // Cancoder Values
    position = -elevatorEncoder.getPositionSinceBoot().getValueAsDouble();
    // position = (leftElevatorMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("elevator/Elevator Position", position);
    UPPER_LIMIT = SmartDashboard.getNumber("UpperLimit", UPPER_LIMIT);
    LOWER_LIMIT = SmartDashboard.getNumber("LowerLimit", LOWER_LIMIT);

    // Kraken Values
    krakenPosition = leftElevatorMotor.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("elevator/Kraken Position", krakenPosition);

    // Retrieve updated PID values from SmartDashboard
    kP = SmartDashboard.getNumber("elevator/kP", kP);
    kI = SmartDashboard.getNumber("elevator/kI", kI);
    kD = SmartDashboard.getNumber("elevator/kD", kD);
    // kS = SmartDashboard.getNumber("elevator/kS", kS);
    kV = SmartDashboard.getNumber("elevator/kV", kV);
    kA = SmartDashboard.getNumber("elevator/kA", kA);
    kG = SmartDashboard.getNumber("elevator/kG", kG);

    cruiseVelocity = SmartDashboard.getNumber("elevator/cruiseVelocity", cruiseVelocity);
    acceleration = SmartDashboard.getNumber("elevator/acceleration", acceleration);
    jerk = SmartDashboard.getNumber("elevator/jerk", jerk);

    // Add Slider to dynamically change Elevator Speed
    double newSpeed = SmartDashboard.getNumber("elevator/Elevator Speed", elevatorSpeed);
    if (newSpeed != elevatorSpeed) {
      elevatorSpeed = newSpeed;
    }

    // Publish actual elevator speed
    SmartDashboard.putNumber(
        "elevator/Left Motor Velocity", leftElevatorMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "elevator/Right Motor Velocity", rightElevatorMotor.getVelocity().getValueAsDouble());

    // Publish Elevator Voltage
    SmartDashboard.putNumber(
        "elevator/Left Motor Voltage", leftElevatorMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        "elevator/Right Motor Voltage", rightElevatorMotor.getMotorVoltage().getValueAsDouble());

    // Publish Motor Temperatures
    // SmartDashboard.putNumber(
    //     "elevator/Left Motor Temp", leftElevatorMotor.getDeviceTemp().getValueAsDouble());
    // SmartDashboard.putNumber(
    //     "elevator/Right Motor Temp", rightElevatorMotor.getDeviceTemp().getValueAsDouble());
  }
}
