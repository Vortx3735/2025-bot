package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;

public class MoveToL2 extends Command {
  private final Elevator m_elevator;
  private final CoralIntake m_coralIntake;
  private final AlgaeIntake m_algaeIntake;
  private boolean isFinishedBool = false;
  
  /**
   * @param elevator The elevator subsystem used by this command.
   * @param algaeIntake The algae intake subsystem used by this command.
   * @param coralIntake The coral intake subsystem used by this command.
   * @param elevatorPos The target position for the elevator(0 to 5).
   * @param coralWristPos The target angle for the coral intake wrist(0 to -1).
   * @param algaeWristPos The target angle for the algae intake wrist(0 to 1).
   */
  public MoveToL2(
      Elevator elevator,
      AlgaeIntake algaeIntake,
      CoralIntake coralIntake
      ) {
    m_elevator = elevator;
    m_coralIntake = coralIntake;
    m_algaeIntake = algaeIntake;
    addRequirements(m_elevator);
    addRequirements(m_coralIntake);
    addRequirements(m_algaeIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if( m_elevator.moveElevatorToL2() && m_coralIntake.moveWristToL2()){
      isFinishedBool = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinishedBool;
  }
}
