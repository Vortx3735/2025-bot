package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Elevator;

public class ElevatorCom {
    private final Elevator elevator;

    public ElevatorCom(Elevator elevator) {
        this.elevator = elevator;
    }

    /**
     * Command for manual control of the elevator using a joystick axis.
     *
     * @param speedSupplier A supplier that provides the joystick input for speed (-1.0 to 1.0).
     * @return A command that runs the elevator manually.
     */
    public Command manualControl(java.util.function.DoubleSupplier speedSupplier) {
        return new RunCommand(
            () -> elevator.manualControl(speedSupplier.getAsDouble()),
            elevator
        );
    }

    /**
     * Command to move the elevator to a specific height.
     *
     * @param targetHeight The target height in encoder units.
     * @return A command that moves the elevator to the target height.
     */
    public Command moveToHeight(double targetHeight) {
        return new RunCommand(
            () -> elevator.moveToHeight(targetHeight),
            elevator
        ).until(() -> Math.abs(elevator.getElevatorHeight() - targetHeight) < 10); // Stop when close enough
    }
}
