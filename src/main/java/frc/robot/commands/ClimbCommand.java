package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {
    public enum Operation {
        LIFT, RELEASE, GRAB, HOLD
    }

    private final ClimbSubsystem climbSubsystem;
    private final Operation operation;
    private final double speed;

    public ClimbCommand(ClimbSubsystem climbSubsystem, Operation operation) {
        this.climbSubsystem = climbSubsystem;
        this.operation = operation;
        this.speed = operation == Operation.LIFT ? 0.5 : -0.5; // Adjust speeds as needed
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climbSubsystem.setCoastMode();
    }

    @Override
    public void execute() {
        climbSubsystem.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stopMotor();
        climbSubsystem.setBrakeMode();
    }

    @Override
    public boolean isFinished() {
        return (operation == Operation.LIFT) && 
               (climbSubsystem.getPosition() >= ClimbSubsystem.CLIMB_MAX_POSITION);
    }
}