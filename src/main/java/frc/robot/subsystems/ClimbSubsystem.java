package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private final TalonFX climbMotor1;
  private final TalonFX climbMotor2;
  public static final double CLIMB_MIN_POSITION = 0; // Placeholder encoder ticks
  public static final double CLIMB_MAX_POSITION = 10000; // Placeholder encoder ticks

  public double speed;

  /**
   * @param motorPort1 The CAN ID of the first climb motor.
   * @param motorPort2 The CAN ID of the second climb motor.
   */
  public ClimbSubsystem(int motorPort1, int motorPort2) {
    climbMotor1 = new TalonFX(motorPort1);
    climbMotor2 = new TalonFX(motorPort2);

    climbMotor1.setNeutralMode(NeutralModeValue.Brake);
    climbMotor2.setNeutralMode(NeutralModeValue.Brake);
  }

  public void move() {
    if ((climbMotor1.getPosition().getValueAsDouble() >= CLIMB_MAX_POSITION)
        && (climbMotor1.getPosition().getValueAsDouble() <= CLIMB_MIN_POSITION)) {
      stopMotor();
    } else {
      climbMotor1.set(speed);
      climbMotor2.set(speed);
    }
  }

  public void stopMotor() {
    climbMotor1.set(0);
    climbMotor2.set(0);
  }

  public double getMotorSpeed() {
    return climbMotor1.get();
  }

  public double getClimbPosition() {
    return climbMotor1.getPosition().getValueAsDouble();
  }

  public void publishInitialValues() {
    SmartDashboard.putNumber("climb/speed", speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update motor speed
    speed = SmartDashboard.getNumber("climb/speed", speed);
  }
}
