package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {

  // Right side algae instake
  static SparkMax algaeInMotor1;
  static SparkMax algaeWrist1;

  // Left side algae intake
  static SparkMax algaeInMotor2;
  static SparkMax algaeWrist2;

  public AlgaeIntake(int motor1id, int wrist1id, int motor2id, int wrist2id) {
    // Intake constructor
    SparkMaxConfig algaeInMotorConfig = new SparkMaxConfig();
    SparkMaxConfig algaeWristConfig = new SparkMaxConfig();

    algaeInMotor1 = new SparkMax(motor1id, MotorType.kBrushless);
    algaeInMotor2 = new SparkMax(motor2id, MotorType.kBrushless);
    algaeInMotorConfig.inverted(true).idleMode(IdleMode.kBrake);

    // set up PID
    algaeWrist1 = new SparkMax(wrist1id, MotorType.kBrushless);
    algaeWrist2 = new SparkMax(wrist2id, MotorType.kBrushless);
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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

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
    algaeWrist2.set(speed);
  }

  public void stopWrist() {
    // stop wrist
    algaeWrist1.set(0);
    algaeWrist2.set(0);
  }

  public void moveWristToPosition(double position) {
    // move wrist to position
    algaeWrist1.getEncoder().setPosition(position);
    algaeWrist2.getEncoder().setPosition(position);
  }

  public double getWristPosition() {
    // get wrist position
    return algaeWrist1.getEncoder().getPosition();
  }

  public void resetWristPosition() {
    // reset wrist position
    algaeWrist1.getEncoder().setPosition(0);
    algaeWrist2.getEncoder().setPosition(0);
  }
}
