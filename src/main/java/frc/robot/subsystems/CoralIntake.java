package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.SensorConstants;

public class CoralIntake extends SubsystemBase {

  public static SparkMax leftCoralMotor;
  public static SparkMax rightCoralMotor;
  public static SparkMax coralWrist;

  private final CANcoder wristEncoder;
  private double position;

  private PIDController wristPID;
  private double ki, kp, kd;

  private SimpleMotorFeedforward wristFF;
  private double ka, kg, ks, kv;

  private double intakeSpeed;

  public DigitalInput leftCoralBeamBreak = new DigitalInput(SensorConstants.CORAL_LEFT_BEAM_BREAK);
  public DigitalInput rightCoralBeamBreak =
      new DigitalInput(SensorConstants.CORAL_RIGHT_BEAM_BREAK);

  /**
   * @param leftMotorId The CAN ID of the left intake motor.
   * @param rightMotorId The CAN ID of the right intake motor.
   * @param wristId The CAN ID of the wrist motor.
   * @param wristEncoderId The CAN ID of the wrist encoder.
   */
  public CoralIntake(int leftMotorId, int rightMotorId, int wristId, int wristEncoderId) {
    // Motor configurations
    SparkMaxConfig coralMotorConfig = new SparkMaxConfig();
    SparkMaxConfig coralWristConfig = new SparkMaxConfig();

    // Initialize intake motors
    leftCoralMotor = new SparkMax(leftMotorId, MotorType.kBrushless);
    rightCoralMotor = new SparkMax(rightMotorId, MotorType.kBrushless);

    // Set motor configurations
    coralMotorConfig.inverted(true).idleMode(IdleMode.kBrake);
    leftCoralMotor.configure(
        coralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightCoralMotor.configure(
        coralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize wrist motor and encoder
    coralWrist = new SparkMax(wristId, MotorType.kBrushless);
    wristEncoder = new CANcoder(wristEncoderId);

    // Configure wrist motor settings
    coralWristConfig.inverted(true).idleMode(IdleMode.kBrake);
    coralWristConfig.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    coralWristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);
    coralWrist.configure(
        coralWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize wrist feedforward control
    ka = 0.1;
    kg = 0.1;
    ks = 0.1;
    kv = 0.1;
    wristFF = new SimpleMotorFeedforward(ks, kg, kv, ka);

    // Initialize PID control for wrist mechanism
    kp = 0.01;
    ki = 0.0;
    kd = 0.0;
    wristPID = new PIDController(kp, ki, kd);
  }

  public void intake() {
    // move motor
    leftCoralMotor.set(intakeSpeed);
    rightCoralMotor.set(intakeSpeed);
  }

  public void outtake(){
    leftCoralMotor.set(-intakeSpeed);
    rightCoralMotor.set(-intakeSpeed);
  }

  public void stopIntake() {
    // stop motor
    leftCoralMotor.set(0);
    rightCoralMotor.set(0);
  }

  public void moveWrist(double speed) {
    if (position > CoralConstants.WRIST_LOWER_LIMIT
        && position < CoralConstants.WRIST_UPPER_LIMIT) {
      coralWrist.set(speed);
    } else {
      stopWrist(); // Stop motor if out of bounds
    }
  }

  public void stopWrist() {
    // stop wrist
    coralWrist.set(0);
  }

  /**
   * @Param targetRadians The target position in radians to move the wrist to.
   */
  public void moveWristToPosition(double targetRadians) {
    if (targetRadians < CoralConstants.WRIST_LOWER_LIMIT
        || targetRadians > CoralConstants.WRIST_UPPER_LIMIT) {
      stopWrist();
      return;
    }

    coralWrist.set(
        wristPID.calculate(position * 2 * Math.PI, targetRadians * 2 * Math.PI)
            + wristFF.calculate(targetRadians * 2 * Math.PI, kv));
  }

  public void holdWrist() {
    coralWrist.set(
        wristPID.calculate(position * 2 * Math.PI, (int) position * 2 * Math.PI)
            + wristFF.calculate(position * 2 * Math.PI, kv));
  }

  public double getWristPosition() {
    // get wrist position
    return wristEncoder.getPosition().getValueAsDouble();
  }

  public void resetWristPosition() {
    // reset wrist position
    coralWrist.getEncoder().setPosition(0);
  }

  public boolean isCoralDetected() {
    // check if coral is detected type shi
    return !leftCoralBeamBreak.get() || !rightCoralBeamBreak.get();
  }

  public void publishInitialValues() {
    SmartDashboard.putNumber("CoralIntake/Wrist P", kp);
    SmartDashboard.putNumber("CoralIntake/Wrist I", ki);
    SmartDashboard.putNumber("CoralIntake/Wrist D", kd);

    SmartDashboard.putNumber("CoralIntake/Wrist A", ka);
    SmartDashboard.putNumber("CoralIntake/Wrist G", kg);
    SmartDashboard.putNumber("CoralIntake/Wrist S", ks);
    SmartDashboard.putNumber("CoralIntake/Wrist V", kv);

    SmartDashboard.putNumber("CoralIntake/intakeSpeed", intakeSpeed);
  }

  @Override
  public void periodic() {
    // Update wrist position
    position = getWristPosition();

    // Get PID values from the dashboard (or use default values)
    double newP = SmartDashboard.getNumber("CoralIntake/Wrist P", kp);
    double newI = SmartDashboard.getNumber("CoralIntake/Wrist I", ki);
    double newD = SmartDashboard.getNumber("CoralIntake/Wrist D", kd);

    double newA = SmartDashboard.getNumber("CoralIntake/Wrist A", ka);
    double newG = SmartDashboard.getNumber("CoralIntake/Wrist G", kg);
    double newS = SmartDashboard.getNumber("CoralIntake/Wrist S", ks);
    double newV = SmartDashboard.getNumber("CoralIntake/Wrist V", kv);

    // If the values changed, update the PID controller
    if (newP != kp || newI != ki || newD != kd) {
      kp = newP;
      ki = newI;
      kd = newD;
      wristPID.setP(kp);
      wristPID.setI(ki);
      wristPID.setD(kd);
    }

    // Update FF controller
    if (newA != ka || newG != kg || newS != ks || newV != kv) {
      ka = newA;
      kg = newG;
      ks = newS;
      kv = newV;
      wristFF = new SimpleMotorFeedforward(ka, kg, ks, kv);
    }

    // Update intake speed
    intakeSpeed = SmartDashboard.getNumber("CoralIntake/intakeSpeed", intakeSpeed);

    // Publish actual intake speed
    SmartDashboard.putNumber("CoralIntake/Left Motor Speed", leftCoralMotor.getAppliedOutput());
    SmartDashboard.putNumber("CoralIntake/Right Motor Speed", rightCoralMotor.getAppliedOutput());

    // Publish Wrist Position
    SmartDashboard.putNumber("CoralIntake/Wrist Position", position);
  }
}
