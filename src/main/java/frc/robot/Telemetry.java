package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {
  DoublePublisher xPub;
  DoublePublisher yPub;
  DoublePublisher anglePub;
  DoublePublisher frontRightCancoderPub;
  DoublePublisher frontLeftCancoderPub;
  DoublePublisher backRightCancoderPub;
  DoublePublisher backLeftCancoderPub;

  BooleanPublisher exampleSensorPub;

  boolean exampleSensor = false;
  private final double MaxSpeed;

  /**
   * Construct a telemetry object, with the specified max speed of the robot
   *
   * @param maxSpeed Maximum speed in meters per second
   */
  public Telemetry(double maxSpeed) {
    MaxSpeed = maxSpeed;
    SignalLogger.start();
  }

  /* What to publish over networktables for telemetry */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot swerve drive state */
  private final NetworkTable driveStateTable = inst.getTable("DriveState");
  private final StructPublisher<Pose2d> drivePose =
      driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
  private final StructPublisher<ChassisSpeeds> driveSpeeds =
      driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> driveModuleStates =
      driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModuleState> driveModuleTargets =
      driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
  private final StructArrayPublisher<SwerveModulePosition> driveModulePositions =
      driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
  private final DoublePublisher driveTimestamp =
      driveStateTable.getDoubleTopic("Timestamp").publish();
  private final DoublePublisher driveOdometryFrequency =
      driveStateTable.getDoubleTopic("OdometryFrequency").publish();

  /* Robot pose for field positioning */
  private final NetworkTable table = inst.getTable("Pose");
  private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
  private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

  /* Mechanisms to represent the swerve module states */
  private final Mechanism2d[] m_moduleMechanisms =
      new Mechanism2d[] {
        new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1),
      };
  /* A direction and length changing ligament for speed representation */
  private final MechanismLigament2d[] m_moduleSpeeds =
      new MechanismLigament2d[] {
        m_moduleMechanisms[0]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(new MechanismLigament2d("Speed", 0.5, 0)),
      };
  /* A direction changing and length constant ligament for module direction */
  private final MechanismLigament2d[] m_moduleDirections =
      new MechanismLigament2d[] {
        m_moduleMechanisms[0]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      };

  private final double[] m_poseArray = new double[3];
  private final double[] m_moduleStatesArray = new double[8];
  private final double[] m_moduleTargetsArray = new double[8];

  /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
  public void telemeterize(SwerveDriveState state) {
    /* Telemeterize the swerve drive state */
    drivePose.set(state.Pose);
    driveSpeeds.set(state.Speeds);
    driveModuleStates.set(state.ModuleStates);
    driveModuleTargets.set(state.ModuleTargets);
    driveModulePositions.set(state.ModulePositions);
    driveTimestamp.set(state.Timestamp);
    driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

    /* Also write to log file */
    m_poseArray[0] = state.Pose.getX();
    m_poseArray[1] = state.Pose.getY();
    m_poseArray[2] = state.Pose.getRotation().getDegrees();
    for (int i = 0; i < 4; ++i) {
      m_moduleStatesArray[i * 2 + 0] = state.ModuleStates[i].angle.getRadians();
      m_moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
      m_moduleTargetsArray[i * 2 + 0] = state.ModuleTargets[i].angle.getRadians();
      m_moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
    }

    SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);
    SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
    SignalLogger.writeDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray);
    SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

    /* Telemeterize the pose to a Field2d */
    fieldTypePub.set("Field2d");
    fieldPub.set(m_poseArray);

    /* Telemeterize the module states to a Mechanism2d */
    for (int i = 0; i < 4; ++i) {
      m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
      m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
      m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

      SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
    }
  }

  public void configureNetworkTables() {
    // Get the default instance of NetworkTables that was created automatically
    // when the robot program starts
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // Get the table within that instance that contains the data. There can
    // be as many tables as you like and exist to make it easier to organize
    // your data. In this case, it's a table called datatable.
    NetworkTable table = inst.getTable("positionTable");
    // Start publishing topics within that table that correspond to the X and Y values
    // for some operation in your program.
    // The topic names are actually "/datatable/x" and "/datatable/y".
    xPub = table.getDoubleTopic("x").publish();
    yPub = table.getDoubleTopic("y").publish();
    anglePub = table.getDoubleTopic("robotAngle").publish();
    frontRightCancoderPub = table.getDoubleTopic("frontRightCancoder").publish();
    frontLeftCancoderPub = table.getDoubleTopic("frontLeftCancoder").publish();
    backRightCancoderPub = table.getDoubleTopic("backRightCancoderPub").publish();
    backLeftCancoderPub = table.getDoubleTopic("backLeftCancoderPub").publish();
    exampleSensorPub = table.getBooleanTopic("exampleSensor").publish();
  }

  public void updateNetworkTables(double[] robotPos, double[] encoderPos) {
    xPub.set(robotPos[0]);
    yPub.set(robotPos[1]);
    anglePub.set(robotPos[2]);
    frontRightCancoderPub.set(encoderPos[0]);
    frontLeftCancoderPub.set(encoderPos[1]);
    backRightCancoderPub.set(encoderPos[2]);
    backLeftCancoderPub.set(encoderPos[3]);
    exampleSensorPub.set(exampleSensor);
    exampleSensor = !exampleSensor;
  }
}
