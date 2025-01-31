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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {

  private PIDController wristPID;
  private double ki, kp, kd;

  private ArmFeedforward wristFF;
  private double ka, kg, ks, kv;
  // Right side algae instake
  static SparkMax algaeInMotor1;
  static SparkMax algaeWrist1;

  // Left side algae intake
  static SparkMax algaeInMotor2;
  static SparkMax algaeWrist2;

  private final CANcoder wristEncoder;

  private double position;

  public AlgaeIntake(int motor1id, int wrist1id, int motor2id, int wristid) {
    // Intake constructor
    SparkMaxConfig algaeInMotorConfig = new SparkMaxConfig();
    SparkMaxConfig algaeWristConfig = new SparkMaxConfig();

    algaeInMotor1 = new SparkMax(motor1id, MotorType.kBrushless);
    algaeInMotorConfig.inverted(true).idleMode(IdleMode.kBrake);

    // set up PID
    algaeWrist1 = new SparkMax(wrist1id, MotorType.kBrushless);
    algaeWristConfig.inverted(true).idleMode(IdleMode.kBrake);
    algaeWristConfig.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    algaeWristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);
    algaeInMotor1.configure(
        algaeInMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeInMotor2.configure(
        algaeInMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeWrist1.configure(
        algaeWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeWrist2.configure(
        algaeWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wristEncoder = new CANcoder(wristid);

    ka = 0.0;
    kg = 0.0;
    ks = 0.0;
    kv = 0.0;
    wristFF = new ArmFeedforward(ks, kg, kv, ka);

    kp = 0.01;
    ki = 0.0;
    kd = 0.0;
    wristPID = new PIDController(kp, ki, kd);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    position = wristEncoder.getAbsolutePosition().getValueAsDouble();
    // send data to dashboard or something but idk how to do that
    // SmartDashboard.putData("Wrist 1 velocity",algaeWrist1.getEncoder().getVelocity());

  }

  public void move(double speed) {
    // move motor
    algaeInMotor1.set(speed);
    algaeInMotor2.set(speed);
  }

  public void stopIntake() {
    // stop motor
    algaeInMotor1.set(0);
    algaeInMotor2.set(0);
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
    return algaeWrist1.getEncoder().getPosition();
  }

  public void resetWristPosition() {
    // reset wrist position
    algaeWrist1.getEncoder().setPosition(0);
  }

  public void hold() {
    int setpoint = (int) position;
    algaeWrist1.set(
        wristPID.calculate(position * 2 * Math.PI, setpoint * 2 * Math.PI)
            + wristFF.calculate(setpoint * 2 * Math.PI, kv));
  }
}
