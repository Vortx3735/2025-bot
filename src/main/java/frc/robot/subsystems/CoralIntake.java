package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {

  // Right side coral instake
  static SparkMax coralInMotor1;
  static SparkMax coralWrist1;

  // Left side coral intake
  static SparkMax coralInMotor2;
  static SparkMax coralWrist2;

  public CoralIntake(int motor1id, int wrist1id, int motor2id, int wrist2id) {
    // Intake constructor
    SparkMaxConfig coralInMotorConfig = new SparkMaxConfig();
    SparkMaxConfig coralWristConfig = new SparkMaxConfig();

    coralInMotor1 = new SparkMax(motor1id, MotorType.kBrushless);
    coralInMotor2 = new SparkMax(motor2id, MotorType.kBrushless);
    coralInMotorConfig.inverted(true).idleMode(IdleMode.kBrake);

    // set up PID
    coralWrist1 = new SparkMax(wrist1id, MotorType.kBrushless);
    coralWrist2 = new SparkMax(wrist2id, MotorType.kBrushless);
    coralWristConfig.inverted(true).idleMode(IdleMode.kBrake);
    coralWristConfig.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    coralWristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);

    coralInMotor1.configure(
        coralInMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralInMotor2.configure(
        coralInMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralWrist1.configure(
        coralWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralWrist2.configure(
        coralWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // send data to dashboard or something but idk how to do that
    // SmartDashboard.putData("Wrist 1 velocity",coralWrist1.getEncoder().getVelocity());

  }

  public void move(double speed) {
    // move motor
    coralInMotor1.set(speed);
    coralInMotor2.set(speed);
  }

  public void stopIntake() {
    // stop motor
    coralInMotor1.set(0);
    coralInMotor2.set(0);
  }

  public void moveWrist(double speed) {
    // move wrist
    coralWrist1.set(speed);
    coralWrist2.set(speed);
  }

  public void stopWrist() {
    // stop wrist
    coralWrist1.set(0);
    coralWrist2.set(0);
  }

  public double getWristPosition() {
    // get wrist position
    return coralWrist1.getEncoder().getPosition();
  }

  public void resetWristPosition() {
    // reset wrist position
    coralWrist1.getEncoder().setPosition(0);
    coralWrist2.getEncoder().setPosition(0);
  }
}
