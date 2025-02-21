package frc.robot.commands;
//starting wrist setpoint: -0.22
//L2 L3 wrist setpoint: 0.49

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

public class MoveCoralWrist extends Command {
  private final CoralIntake m_CoralIntake;
  private final double position;
  private boolean isFinishedBool;

  /**
   * Creates a new DefaultAlgaeIntakeCommand
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveCoralWrist(CoralIntake subsystem, double pos) {
    m_CoralIntake = subsystem;
    position = pos;
    addRequirements(m_CoralIntake);
    boolean isFinishedBool = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_AlgaeIntake.hold();
    isFinishedBool = m_CoralIntake.moveWristToPosition(position);
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
