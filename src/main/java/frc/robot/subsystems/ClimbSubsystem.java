package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
public class ClimbSubsystem extends SubsystemBase {
  private final SparkMax climbMotor;
  private final RelativeEncoder encoder;

  public ClimbSubsystem(int motorPort) {
    climbMotor = new SparkMax(1, MotorType.kBrushless);
    encoder = climbMotor.getEncoder();
  }

  public void grab() {
    climbMotor.set(0.2); //  for grabbing
  }

  public void lift() {
    climbMotor.set(0.5); //  for lifting
  }

  public void release() {
    climbMotor.set(-0.2); //  for releasing
  }

  public void hold() {
    climbMotor.set(0.1); //  to hold position
  }

  public void stop() {
    climbMotor.set(0); // Stop the motor
  }

  public double getMotorSpeed() {
    return climbMotor.get();
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void periodic() {
    // Update telemetry to Elastic
    SmartDashboard.putNumber("Climb Motor Speed", getMotorSpeed());
    SmartDashboard.putNumber("Climb Position", getPosition());
    SmartDashboard.putNumber("Climb Velocity", getVelocity());
  }
  
}

