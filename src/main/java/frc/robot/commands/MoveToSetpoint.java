package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;

public class MoveToSetpoint extends Command {
  private final Elevator m_elevator;
  private final CoralIntake m_coralIntake;
  private final AlgaeIntake m_algaeIntake;
  private final double targetElevatorPos;
  private final double targetCoralPos;
  private final double targetAlgaePos;
  private final boolean spinintake;

  /**
   * @param elevator The elevator subsystem used by this command.
   * @param algaeIntake The algae intake subsystem used by this command.
   * @param coralIntake The coral intake subsystem used by this command.
   * @param elevatorPos The target position for the elevator(0 to 5).
   * @param coralWristPos The target angle for the coral intake wrist(0 to -1).
   * @param algaeWristPos The target angle for the algae intake wrist(0 to 1).
   */
  public MoveToSetpoint(
      Elevator elevator,
      AlgaeIntake algaeIntake,
      CoralIntake coralIntake,
      double elevatorPos,
      double coralWristPos,
      double algaeWristPos,
      boolean spinIntake) {
    m_elevator = elevator;
    m_coralIntake = coralIntake;
    m_algaeIntake = algaeIntake;
    targetElevatorPos = elevatorPos;
    targetCoralPos = coralWristPos;
    targetAlgaePos = algaeWristPos;
    spinintake = spinIntake;
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
    m_elevator.moveElevatorToPosition(targetElevatorPos);
    m_coralIntake.moveWristToPosition(targetCoralPos);
    m_algaeIntake.moveWristToPosition(targetAlgaePos);
    if (spinintake){
      m_coralIntake.intake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
