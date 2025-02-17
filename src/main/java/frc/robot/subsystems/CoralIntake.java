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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SensorConstants;
import frc.robot.commands.defaultcommands.DefaultCoralIntakeCommand;
import java.util.function.BooleanSupplier;

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

  public double wristDownDefault = 0.09;
  public double wristUpDefault = -0.17;
  public double errorDefault = 0.03;

  public double wristSpeedDown;
  public double wristSpeedUp;
  public double error;
  private double intakeSpeed = 0.25;

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
    SparkMaxConfig coralRightMotorConfig = new SparkMaxConfig();
    SparkMaxConfig coralWristConfig = new SparkMaxConfig();

    // Initialize intake motors
    leftCoralMotor = new SparkMax(leftMotorId, MotorType.kBrushless);
    rightCoralMotor = new SparkMax(rightMotorId, MotorType.kBrushless);

    // Set motor configurations
    coralMotorConfig.inverted(true).idleMode(IdleMode.kBrake);
    leftCoralMotor.configure(
        coralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightCoralMotor.configure(
        coralMotorConfig.inverted(false),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Initialize wrist motor and encoder
    coralWrist = new SparkMax(wristId, MotorType.kBrushless);
    wristEncoder = new CANcoder(wristEncoderId);

    // Configure wrist motor settings
    coralWristConfig.inverted(true).idleMode(IdleMode.kBrake);
    // coralWristConfig.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    coralWristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);
    coralWrist.configure(
        coralWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wristSpeedDown = wristDownDefault;
    wristSpeedUp = wristUpDefault;
    error = errorDefault;
  }

  public void intake() {
    // move motor
    if (leftCoralDetected() || rightCoralDetected()) {
      // leftCoralMotor.set(0);
      // rightCoralMotor.set(0);
      CommandScheduler.getInstance()
          .schedule(
              new SequentialCommandGroup(
                  new WaitCommand(0.2), new DefaultCoralIntakeCommand(this)));
    } else {
      leftCoralMotor.set(intakeSpeed);
      rightCoralMotor.set(intakeSpeed);
    }
  }

  // returns true assuming beam break is broken
  public BooleanSupplier getCoralIntakeBeam() {
    if (rightCoralBeamBreak.get() == false) {
      return () -> true;
    }
    return () -> false;
  }

  public void outtake() {
    leftCoralMotor.set(-intakeSpeed);
    rightCoralMotor.set(-intakeSpeed);
  }

  public void stopIntake() {
    // stop motor
    leftCoralMotor.set(0);
    rightCoralMotor.set(0);
  }

  public void moveWristUp() {
    moveWrist(wristSpeedUp);
  }

  public void moveWristDown() {
    moveWrist(wristSpeedDown);
  }

  public void moveWrist(double speed) {
    coralWrist.set(speed);
  }

  public void stopWrist() {
    // stop wrist
    coralWrist.set(0);
  }

  /**
   * @Param targetPos The target position to move the wrist to.
   */
  public void moveWristToPosition(double targetPos) {
    // if (targetRadians < CoralConstants.WRIST_LOWER_LIMIT
    //     || targetRadians > CoralConstants.WRIST_UPPER_LIMIT) {
    //   stopWrist();
    //   return;
    // }
    if (Math.abs(targetPos - position) < error) {
      coralWrist.stopMotor();
    } else if (position < targetPos) {
      coralWrist.set(wristSpeedUp);
    } else if (position > targetPos) {
      coralWrist.set(wristSpeedDown);
    } else {
      coralWrist.stopMotor();
    }
    // wristPID.calculate(position * 2 * Math.PI, targetPos * 2 * Math.PI)
    //     + wristFF.calculate(targetPos * 2 * Math.PI, kv));
  }

  public boolean moveWristToPositionBool(double targetPos) {
    // if (targetRadians < CoralConstants.WRIST_LOWER_LIMIT
    //     || targetRadians > CoralConstants.WRIST_UPPER_LIMIT) {
    //   stopWrist();
    //   return;
    // }
    if (Math.abs(targetPos - position) < error) {
      coralWrist.stopMotor();
      return true;
    } else if (position < targetPos) {
      coralWrist.set(wristSpeedUp);
      return false;
    } else if (position > targetPos) {
      coralWrist.set(wristSpeedDown);
      return false;
    } else {
      coralWrist.stopMotor();
      return true;
    }
    // wristPID.calculate(position * 2 * Math.PI, targetPos * 2 * Math.PI)
    //     + wristFF.calculate(targetPos * 2 * Math.PI, kv));
  }

  public void hold() {
    coralWrist.set(
        wristPID.calculate(position * 2 * Math.PI, (int) position * 2 * Math.PI)
            + wristFF.calculate(position * 2 * Math.PI, kv));
  }

  public double getWristPosition() {
    // get wrist position
    return wristEncoder.getPositionSinceBoot().getValueAsDouble();
  }

  // public void resetWristPosition() {
  //   // reset wrist position
  //   coralWrist.getEncoder().setPosition(0);
  // }

  public boolean leftCoralDetected() {
    // check if coral is detected type shi
    return !leftCoralBeamBreak.get();
  }

  public boolean rightCoralDetected() {
    // check if coral is detected type shi
    return !rightCoralBeamBreak.get();
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

    SmartDashboard.putNumber("CoralIntake/Wrist Up Speed", wristSpeedUp);
    SmartDashboard.putNumber("CoralIntake/Wrist Down Speed", wristSpeedDown);
    SmartDashboard.putNumber("CoralIntake/Wrist Error", error);
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

    wristSpeedDown = SmartDashboard.getNumber("CoralIntake/Wrist Down Speed", wristDownDefault);
    wristSpeedUp = SmartDashboard.getNumber("CoralIntake/Wrist Up Speed", wristUpDefault);
    error = SmartDashboard.getNumber("CoralIntake/Wrist Error", errorDefault);
    SmartDashboard.putBoolean("CoralIntake/Left Beam Break", leftCoralBeamBreak.get());
    SmartDashboard.putBoolean("CoralIntake/Right Beam Break", rightCoralBeamBreak.get());

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
