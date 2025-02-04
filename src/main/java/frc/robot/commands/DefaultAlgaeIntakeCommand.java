package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

public class DefaultAlgaeIntakeCommand extends Command {
  private final AlgaeIntake m_AlgaeIntake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultAlgaeIntakeCommand(AlgaeIntake subsystem) {
    this.m_AlgaeIntake = subsystem;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_AlgaeIntake.isAlgaeDetected()) {
      m_AlgaeIntake.moveWristToPosition(0); // Move to elevator angle (default to L4)
      m_AlgaeIntake.stopIntake();
      ; // Stop intake motor
    } else {
      m_AlgaeIntake.moveWristToPosition(-0.5); // Move to Human Player position
      m_AlgaeIntake.move(1.0); // Run intake motor
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_AlgaeIntake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
