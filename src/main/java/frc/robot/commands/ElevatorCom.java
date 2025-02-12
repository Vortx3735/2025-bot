package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorCom extends Command {

  private final Elevator m_elevator;
  private boolean isUp;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorCom(Elevator subsystem, boolean isUp) {
    m_elevator = subsystem;
    this.isUp = isUp;
    addRequirements(m_elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isUp) {
      m_elevator.setElevatorSpeed(m_elevator.elevatorSpeed);
    } else {
      m_elevator.setElevatorSpeed(-m_elevator.elevatorSpeed * 0.7);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_elevator.position >= m_elevator.UPPER_LIMIT || m_elevator.position <= m_elevator.LOWER_LIMIT){
      return true;
    }
    return false;
  }
}
