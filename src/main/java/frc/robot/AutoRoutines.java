package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
  private final AutoFactory m_factory;

  public AutoRoutines(AutoFactory factory) {
    m_factory = factory;
  }

  public AutoRoutine testAuto1() {
    final AutoRoutine routine = m_factory.newRoutine("Test Auto 1");
    final AutoTrajectory testTraj = routine.trajectory("LeftHP_ReefFront");

    routine.active().onTrue(testTraj.resetOdometry().andThen(testTraj.cmd()));
    return routine;
  }
}
 
