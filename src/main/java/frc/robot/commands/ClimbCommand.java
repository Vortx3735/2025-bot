package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends CommandBase {
    private final ClimbSubsystem climbSubsystem;
    private final Operation operation;

    public enum Operation {
        GRAB,
        LIFT,
        RELEASE,
        HOLD
    }

    public ClimbCommand(ClimbSubsystem subsystem, Operation operation) {
        this.climbSubsystem = subsystem;
        this.operation = operation;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        switch (operation) {
            case GRAB:
                climbSubsystem.grab();
                break;
            case LIFT:
                climbSubsystem.lift();
                break;
            case RELEASE:
                climbSubsystem.release();
                break;
            case HOLD:
                climbSubsystem.hold();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Adjust logic if needed for ending
    }
}
