package frc.robot.commands;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Climb;

public class ClimbSubsystem extends SubsystemBase {
    private final MotorController climbMotor;

    pubilic ClimbSubsystem(MotorController climbMotor){
        this.climbMotor = climbMotor;
    }


}
    //EXTEND CLIMBER!//
    public void extend() {
        climbMotor.set(1.0); // SET ALLOWS YOU TO ADJUST SPPED IT EXTENDS !//
    }

    //RETRACT CLIMBER!//
    public void retract() {
        climbMotor.set(-1.0); // SET ALLOWS YOU TO ADJUST SPPED IT RETRACTS !//
    }

    public void stop() {
        climbMotor.set(0); // STOPS CLIMBER
    }