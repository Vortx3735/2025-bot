import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.CoralIntake;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class UnitTest {
  static final double DELTA = 1e-2; // acceptable deviation range

  SparkMax motor;
  SparkMaxSim motor_Sim;
  CoralIntake m_CoralIntake;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    m_CoralIntake = new CoralIntake(21, 22, 23);
    motor = CoralIntake.coralInMotor1;
    motor_Sim = new SparkMaxSim(motor, DCMotor.getNeo550(1));
  }

  @Test
  void intakeTest() {
    m_CoralIntake.move(-1);
    assertEquals(-1, motor.get(), DELTA);
  }
}
