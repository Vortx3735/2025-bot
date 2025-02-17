package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

public class MoveAlgaeWrist extends Command {
  private final AlgaeIntake m_AlgaeIntake;
  private final double position;
  private boolean isFinishedBool;

  /**
   * Creates a new DefaultAlgaeIntakeCommand
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveAlgaeWrist(AlgaeIntake subsystem, double pos) {
    m_AlgaeIntake = subsystem;
    position = pos;
    addRequirements(m_AlgaeIntake);
    isFinishedBool = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_AlgaeIntake.hold();
    isFinishedBool = m_AlgaeIntake.moveWristToPositionBool(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return isFinishedBool;
  }
}
