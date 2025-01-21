package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Intake extends SubsystemBase {

  //Right side coral instake
  static CANSparkMax coralInMotor1;
  static CANSparkMax coralWrist1;
  static RelativeEncoder coralWrist1Encoder;
  
  //Left side coral intake
  static CANSparkMax coralInMotor2;
  static CANSparkMax coralWrist2;
  static RelativeEncoder coralWrist2Encoder;


  public Intake(int motor1id, int wrist1id, int motor2id, int wrist2id) {
    // Intake constructor
    coralInMotor1 = new CANSparkMax(motor1id, MotorType.kBrushless);
    coralInMotor1.restoreFactoryDefaults();
    coralInMotor1.setIdleMode(IdleMode.kBrake);
    coralInMotor1.setInverted(false);

    coralWrist1 = new CANSparkMax(wrist1id, MotorType.kBrushless);
    coralWrist1.restoreFactoryDefaults();
    coralWrist1.setIdleMode(IdleMode.kBrake);
    coralWrist1.setInverted(false);
    coralWrist1Encoder = new RelativeEncoder(coralWrist1);

    coralInMotor2 = new CANSparkMax(motor1id, MotorType.kBrushless);
    coralInMotor2.restoreFactoryDefaults();
    coralInMotor2.setIdleMode(IdleMode.kBrake);
    coralInMotor2.setInverted(false);

    coralWrist2 = new CANSparkMax(wrist1id, MotorType.kBrushless);
    coralWrist2.restoreFactoryDefaults();
    coralWrist2.setIdleMode(IdleMode.kBrake);
    coralWrist2.setInverted(false);
    coralWrist1Encoder = new RelativeEncoder(coralWrist2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //send data to dashboard or something but idk how to do that

  }

  public void move(double speed){
    //move motor
    coralInMotor1.set(speed);
    coralInMotor2.set(speed);
  }

  public void stopIntake(){
    //stop motor
    coralInMotor1.set(0);
    coralInMotor2.set(0);
  }

  public void moveWrist(double speed){
    //move wrist
    coralWrist1.set(speed);
    coralWrist2.set(speed);
  }

  public void stopWrist(){
    //stop wrist
    coralWrist1.set(0);
    coralWrist2.set(0);
  }

  public void moveWristToPosition(double position){
    //move wrist to position
    coralWrist1.getEncoder().setPosition(position);
    coralWrist2.getEncoder().setPosition(position);
  }

  public double getWristPosition(){
    //get wrist position
    return coralWrist1.getEncoder().getPosition();
  }

  public void resetWristPosition(){
    //reset wrist position
    coralWrist1.getEncoder().setPosition(0);
    coralWrist2.getEncoder().setPosition(0);
  }

}
