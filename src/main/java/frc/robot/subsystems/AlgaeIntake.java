package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {

  private PIDController wristPID;
  private double ki, kp, kd;

  private ArmFeedforward wristFF;
  private double ka, kg, ks, kv;

  static SparkMax leftAlgaeMotor;
  static SparkMax rightAlgaeMotor;
  static SparkMax algaeWrist1;

  private final CANcoder wristEncoder;

  private double position;
  private double intakeSpeed = 0.25;

  /**
   * @param leftMotorID The CAN ID of the left intake motor.
   * @param rightMotorID The CAN ID of the right intake motor.
   * @param wristID The CAN ID of the wrist motor.
   * @param wristEncoderID The CAN ID of the wrist encoder.
   */
  public AlgaeIntake(int leftMotorID, int rightMotorID, int wristID, int wristEncoderID) {
    // Configure motor settings
    SparkMaxConfig algaeMotorConfig = new SparkMaxConfig();
    SparkMaxConfig algaeWristConfig = new SparkMaxConfig();

    // Initialize intake motors
    leftAlgaeMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
    rightAlgaeMotor = new SparkMax(rightMotorID, MotorType.kBrushless);

    // Set motor configurations
    algaeMotorConfig.inverted(true).idleMode(IdleMode.kBrake);
    leftAlgaeMotor.configure(
        algaeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightAlgaeMotor.configure(
        algaeMotorConfig.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize wrist motor and encoder
    algaeWrist1 = new SparkMax(wristID, MotorType.kBrushless);
    wristEncoder = new CANcoder(wristEncoderID);

    // Configure wrist motor settings
    algaeWristConfig.inverted(true).idleMode(IdleMode.kBrake);
    algaeWristConfig.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    algaeWristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);

    algaeWrist1.configure(
        algaeWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize wrist feedforward control
    ka = 0.0;
    kg = 0.0;
    ks = 0.0;
    kv = 0.0;
    wristFF = new ArmFeedforward(ks, kg, kv, ka);

    // Initialize PID control for wrist mechanism
    kp = 0.01;
    ki = 0.0;
    kd = 0.0;
    wristPID = new PIDController(kp, ki, kd);
  }

  public void intake() {
    leftAlgaeMotor.set(intakeSpeed);
    rightAlgaeMotor.set(intakeSpeed);
  }

  public void outtake() {
    leftAlgaeMotor.set(-intakeSpeed);
    rightAlgaeMotor.set(-intakeSpeed);
  }

  public void stopIntake() {
    // stop motor
    leftAlgaeMotor.set(0);
    rightAlgaeMotor.set(0);
  }

  public void moveWrist(double speed) {
    // move wrist
    algaeWrist1.set(speed);
  }

  public void moveWristToPosition(double p) {
    algaeWrist1.set(
        wristPID.calculate(position * 2 * Math.PI, p * 2 * Math.PI)
            + wristFF.calculate(p * 2 * Math.PI, kv));
  }

  public void stopWrist() {
    // stop wrist
    algaeWrist1.set(0);
  }

  public double getWristPosition() {
    // get wrist position
    return wristEncoder.getPosition().getValueAsDouble();
  }

  public void resetWristPosition() {
    // reset wrist position
    algaeWrist1.getEncoder().setPosition(0);
  }

  public void hold() {
    algaeWrist1.set(
        wristPID.calculate(position * 2 * Math.PI, (int) position * 2 * Math.PI)
            + wristFF.calculate(position * 2 * Math.PI, kv));
  }

  public void publishInitialValues() {
    // Publish initial values to the dashboard
    SmartDashboard.putNumber("AlgaeIntake/Wrist P", kp);
    SmartDashboard.putNumber("AlgaeIntake/Wrist I", ki);
    SmartDashboard.putNumber("AlgaeIntake/Wrist D", kd);

    SmartDashboard.putNumber("AlgaeIntake/Wrist A", ka);
    SmartDashboard.putNumber("AlgaeIntake/Wrist G", kg);
    SmartDashboard.putNumber("AlgaeIntake/Wrist S", ks);
    SmartDashboard.putNumber("AlgaeIntake/Wrist V", kv);

    SmartDashboard.putNumber("AlgaeIntake/intakeSpeed", intakeSpeed);
  }

  @Override
  public void periodic() {
    // Update wrist position
    position = getWristPosition();

    // Get PID values from the dashboard (or use default values)
    double newP = SmartDashboard.getNumber("AlgaeIntake/Wrist P", kp);
    double newI = SmartDashboard.getNumber("AlgaeIntake/Wrist I", ki);
    double newD = SmartDashboard.getNumber("AlgaeIntake/Wrist D", kd);

    double newA = SmartDashboard.getNumber("AlgaeIntake/Wrist A", ka);
    double newG = SmartDashboard.getNumber("AlgaeIntake/Wrist G", kg);
    double newS = SmartDashboard.getNumber("AlgaeIntake/Wrist S", ks);
    double newV = SmartDashboard.getNumber("AlgaeIntake/Wrist V", kv);

    // If the values changed, update the PID controller
    if (newP != kp || newI != ki || newD != kd) {
      wristPID.setP(newP);
      wristPID.setI(newI);
      wristPID.setD(newD);
    }

    // Update FF controller
    if (newA != ka || newG != kg || newS != ks || newV != kv) {
      ka = newA;
      kg = newG;
      ks = newS;
      kv = newV;
      wristFF = new ArmFeedforward(ks, kg, kv, ka);
    }

    intakeSpeed = SmartDashboard.getNumber("AlgaeIntake/intakeSpeed", intakeSpeed);

    // Publish actual intake speed
    SmartDashboard.putNumber("AlgaeIntake/Left Motor Speed", leftAlgaeMotor.getAppliedOutput());
    SmartDashboard.putNumber("AlgaeIntake/Right Motor Speed", rightAlgaeMotor.getAppliedOutput());

    // Publish Wrist Position
    SmartDashboard.putNumber("AlgaeIntake/Wrist Position", position);
  }
}
