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
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {

  public static SparkMax leftCoralMotor;
  public static SparkMax rightCoralMotor;
  public static SparkMax coralWrist;

  private final CANcoder wristEncoder;
  private double position;

  private PIDController wristPID;
  private double ki, kp, kd;

  private ArmFeedforward wristFF;
  private double ka, kg, ks, kv;

  public DigitalInput leftCoralBeamBreak = new DigitalInput(Constants.Sensors.CORAL_LEFT_BEAM_BREAK); 
  public DigitalInput rightCoralBeamBreak = new DigitalInput(Constants.Sensors.CORAL_RIGHT_BEAM_BREAK); 

  // NetworkTable Initialization
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable coralTable = inst.getTable("CoralState");
  private final DoublePublisher leftCoralMotorVel =
      coralTable.getDoubleTopic("CoralInMotorVel").publish();
  private final DoublePublisher coralInMother2Vel =
      coralTable.getDoubleTopic("rightCoralMotorVel").publish();

  public CoralIntake(int leftMotorId, int Wristid, int rightMotorId) {
    // Intake constructor
    SparkMaxConfig coralInMotorConfig = new SparkMaxConfig();
    SparkMaxConfig coralWristConfig = new SparkMaxConfig();

    leftCoralMotor = new SparkMax(leftMotorId, MotorType.kBrushless);
    rightCoralMotor = new SparkMax(rightMotorId, MotorType.kBrushless);

    coralInMotorConfig.inverted(true).idleMode(IdleMode.kBrake);

    // set up PID for Coral Wrist
    coralWrist = new SparkMax(Wristid, MotorType.kBrushless);
    coralWristConfig.inverted(true).idleMode(IdleMode.kBrake);
    coralWristConfig.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    coralWristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);

    leftCoralMotor.configure(
        coralInMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightCoralMotor.configure(
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
    leftCoralMotorVel.set(leftCoralMotor.getEncoder().getVelocity());
    coralInMother2Vel.set(rightCoralMotor.getEncoder().getVelocity());
  }

  public void move(double speed) {
    // move motor
    leftCoralMotor.set(speed);
    rightCoralMotor.set(speed);
  }

  public void stopIntake() {
    // stop motor
    leftCoralMotor.set(0);
    rightCoralMotor.set(0);
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
    return !leftCoralBeamBreak.get() || !rightCoralBeamBreak.get();
  }
}
