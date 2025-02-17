package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

public class DefaultCoralIntakeCommand extends Command {
  private final CoralIntake m_CoralIntake;

  /**
   * Default Coral Intake Command
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultCoralIntakeCommand(CoralIntake subsystem) {
    m_CoralIntake = subsystem;
    addRequirements(m_CoralIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // // m_CoralIntake.hold();

    // if (m_CoralIntake.isCoralDetected()) {
    //   m_CoralIntake.moveWristToPosition(0); // Move to elevator angle (default to L4)
    //   m_CoralIntake.stopIntake();
    //   ; // Stop intake motor
    // } else {
    //   m_CoralIntake.moveWristToPosition(-0.5); // Move to Human Player position
    //   m_CoralIntake.intake(); // Run intake motor
    // }
    m_CoralIntake.stopIntake();
    m_CoralIntake.stopWrist();
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
