package frc.robot;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.*;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.*;
import frc.robot.RobotContainer;

public class AutoRoutines {
  private final AutoFactory m_factory;
  private final Optional<Trajectory<SwerveSample>> testReef = Choreo.loadTrajectory("TestReef");

  public AutoRoutines(AutoFactory factory) {
    m_factory = factory;
  }

  public AutoRoutine testAuto1() {
    final AutoRoutine routine = m_factory.newRoutine("Test Auto 1");
    final AutoTrajectory testTraj = routine.trajectory("TestForward");

    routine.active().onTrue(testTraj.resetOdometry().andThen(testTraj.cmd()));
    return routine;
  }

  public AutoRoutine testAuto3() {
    final AutoRoutine routine = m_factory.newRoutine("Test Auto 3");
    final AutoTrajectory testTraj = routine.trajectory("TestReef");

    routine.active().onTrue(
      Commands.sequence(
        testTraj.resetOdometry(),
        testTraj.cmd(),
        new RunCommand(() -> RobotContainer.coralIntake.moveWristToL2(), RobotContainer.coralIntake))
        
      );
    return routine;
  }
  // public AutoRoutine testAuto2() {
  // final AutoRoutine routine = m_factory.newRoutine("Test Auto 2");
  // // final AutoTrajectory testTraj = routine.trajectory("TestReef");

  // routine.active().onTrue(testReef.resetOdometry().andThen(testTraj.cmd()));
  // routine.active().onTrue(
  // new ParallelCommandGroup(
  // testTraj.resetOdometry(),
  // new FollowTrajectoryCommand(testTraj, ),
  // new RunCommand(() -> coralIntake.moveWristToL2(), coralIntake),
  // new RunCommand(() -> elevator.moveElevatorToL2(), elevator),
  // new RunCommand(() -> algaeIntake.stowWrist(), algaeIntake)
  // )
  // );
  // return routine;
  // return routine;
  // }

  public AutoRoutine mAutoRoutine() {
    final AutoRoutine routine = m_factory.newRoutine("Maine Auton");
    final AutoTrajectory testTraj = routine.trajectory("MainAuton");

    routine.active().onTrue(testTraj.resetOdometry().andThen(testTraj.cmd()));
    return routine;
  }
}
