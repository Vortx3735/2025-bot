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

public class Elevator extends SubsystemBase {
  public static TalonFX leftElevatorMotor;
  public static TalonFX rightElevatorMotor;
  private static CANcoder elevatorEncoder;

  public double position;
  public double elevatorSpeed;

  // create a Motion Magic request, voltage output
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  // Define soft limits
  public static double LOWER_LIMIT = 0;
  public static double UPPER_LIMIT = 5;

  public static double startingPosition;

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
  
    startingPosition = leftElevatorMotor.getPosition().getValueAsDouble();
    configureCANcoder();
    configureTalonFX();

    elevatorSpeed = 0.07;
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
    // fx_cfg.Feedback.FeedbackRemoteSensorID = elevatorEncoder.getDeviceID();
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    fx_cfg.Feedback.SensorToMechanismRatio = 15.0;
    fx_cfg.Feedback.RotorToSensorRatio = 1;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Set PID & Feedforward gains

    fx_cfg.Slot0.kP = 35;
    fx_cfg.Slot0.kI = 0;
    fx_cfg.Slot0.kD = 0.0;
    fx_cfg.Slot0.kV = 0.8;
    fx_cfg.Slot0.kA = 0.075;
    fx_cfg.Slot0.kG = 0.368;

    // Motion Magic settings
    fx_cfg.MotionMagic.MotionMagicCruiseVelocity = 15;
    fx_cfg.MotionMagic.MotionMagicAcceleration = 30;
    fx_cfg.MotionMagic.MotionMagicJerk = 300;

    leftElevatorMotor.getConfigurator().apply(fx_cfg);
  }

  /**
   * Moves the elevator to the specified position using Motion Magic control
   *
   * @param targetPosition The target position to move the elevator to
   */
  public boolean moveElevatorToPosition(double targetPos) {
    // Prevent moving past soft limits

    // Motion Magic
    double error = Math.abs(targetPos - position);
    if(error<0.02){
      stopElevator();
      return true;
    }
    leftElevatorMotor.setControl(m_request.withPosition(targetPos));
    return false;
  }

  public void moveElevatorToPositionVoid(double targetPos) {
    // Prevent moving past soft limits
    leftElevatorMotor.setControl(m_request.withPosition(targetPos));
  }

  public void moveElevatorToHP(){
    moveElevatorToPositionVoid(1.015);
  }

  public boolean moveElevatorToL2(){
    return moveElevatorToPosition(1.453125);
  }

  public void moveElevatorToL3(){
    moveElevatorToPosition(2.7412);
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

  public void stopElevator() {
    leftElevatorMotor.set(0);
    rightElevatorMotor.set(0);
  }

  public void setBrakeMode() {
    leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void publishInitialValues() {
    SmartDashboard.putNumber("elevator/Elevator Speed", elevatorSpeed);
    SmartDashboard.putNumber("UpperLimit", UPPER_LIMIT);
    SmartDashboard.putNumber("LowerLimit", LOWER_LIMIT);
  }

  @Override
  public void periodic() {
    position = leftElevatorMotor.getPosition().getValueAsDouble() - startingPosition;

    UPPER_LIMIT = SmartDashboard.getNumber("UpperLimit", UPPER_LIMIT);
    LOWER_LIMIT = SmartDashboard.getNumber("LowerLimit", LOWER_LIMIT);

    // Values
    SmartDashboard.putNumber("elevator/Elevator Position", position);
    SmartDashboard.putNumber("elevator/Kraken Left pos", leftElevatorMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("elevator/Kraken Right Pos", rightElevatorMotor.getPosition().getValueAsDouble());

    // Add Slider to dynamically change Elevator Speed
    double newSpeed = SmartDashboard.getNumber("elevator/Elevator Speed", elevatorSpeed);
    if (newSpeed != elevatorSpeed) {
      elevatorSpeed = newSpeed;
    }
  }
}
