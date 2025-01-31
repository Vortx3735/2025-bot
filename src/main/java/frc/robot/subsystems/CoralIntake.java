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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {

  public static SparkMax coralInMotor1;
  public static SparkMax coralInMotor2;
  public static SparkMax coralWrist;

  private final CANcoder wristEncoder;
  private double position;

  private PIDController wristPID;
  private double ki, kp, kd;

  private ArmFeedforward wristFF;
  private double ka, kg, ks, kv;

  public DigitalInput beamBreakCoral1 = new DigitalInput(3); // Update with the correct DIO port
  public DigitalInput beamBreakCoral2 = new DigitalInput(4); // Update with the correct DIO port

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable coralTable = inst.getTable("CoralState");
  private final DoublePublisher coralInMotor1Vel =
      coralTable.getDoubleTopic("CoralInMotorVel").publish();
  private final DoublePublisher coralInMother2Vel =
      coralTable.getDoubleTopic("CoralInMotor2Vel").publish();

  public CoralIntake(int motor1id, int Wristid, int motor2id) {
    // Intake constructor
    SparkMaxConfig coralInMotorConfig = new SparkMaxConfig();
    SparkMaxConfig coralWristConfig = new SparkMaxConfig();

    coralInMotor1 = new SparkMax(motor1id, MotorType.kBrushless);
    coralInMotor2 = new SparkMax(motor2id, MotorType.kBrushless);
    coralInMotorConfig.inverted(true).idleMode(IdleMode.kBrake);

    // set up PID
    coralWrist = new SparkMax(Wristid, MotorType.kBrushless);
    coralWristConfig.inverted(true).idleMode(IdleMode.kBrake);
    coralWristConfig.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    coralWristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);

    coralInMotor1.configure(
        coralInMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralInMotor2.configure(
        coralInMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralWrist.configure(
        coralWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wristEncoder = new CANcoder(19);

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

    // send data to dashboard or something but idk how to do that
    // SmartDashboard.putData("Wrist 1 velocity",coralWrist.getEncoder().getVelocity());
    position = wristEncoder.getAbsolutePosition().getValueAsDouble();
    coralInMotor1Vel.set(coralInMotor1.getEncoder().getVelocity());
    coralInMother2Vel.set(coralInMotor2.getEncoder().getVelocity());
  }

  public void move(double speed) {
    // move motor
    coralInMotor1.set(speed);
    coralInMotor2.set(speed);
    System.out.println("test");
  }

  public void stopIntake() {
    // stop motor
    coralInMotor1.set(0);
    coralInMotor2.set(0);
  }

  public void moveWrist(double speed) {
    // move wrist
    coralWrist.set(speed);
  }

  public void stopWrist() {
    // stop wrist
    coralWrist.set(0);
  }

  public void moveWristToPosition(double p) {
    coralWrist.set(
        wristPID.calculate(position * 2 * Math.PI, p * 2 * Math.PI)
            + wristFF.calculate(p * 2 * Math.PI, kv));
  }

  public double getWristPosition() {
    // get wrist position
    return coralWrist.getEncoder().getPosition();
  }

  public void resetWristPosition() {
    // reset wrist position
    coralWrist.getEncoder().setPosition(0);
  }

  public boolean isCoralDetected() {
    // check if coral is detected type shi
    return !beamBreakCoral1.get() || !beamBreakCoral2.get();
  }
}
