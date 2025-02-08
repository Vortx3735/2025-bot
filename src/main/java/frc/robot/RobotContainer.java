// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.DefaultAlgaeIntakeCommand;
import frc.robot.commands.DefaultCoralIntakeCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.util.TunerConstants;
import frc.robot.util.VorTXControllerXbox;

public class RobotContainer {
  private final ClimbSubsystem climbSubsystem;
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed, MaxAngularRate);

  private final VorTXControllerXbox driver = new VorTXControllerXbox(0);
  private final VorTXControllerXbox operator = new VorTXControllerXbox(1);

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /* Path follower */
  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser = new AutoChooser();

  public final CoralIntake coralIntake;
  public final AlgaeIntake algaeIntake;
  public final Elevator elevator;

  public RobotContainer() {
    climbSubsystem =
        new ClimbSubsystem(
            Constants.Climber.CLIMBER_LEFTMOTOR_ID, Constants.Climber.CLIMBER_RIGHTMOTOR_ID);
    autoFactory = drivetrain.createAutoFactory();
    autoRoutines = new AutoRoutines(autoFactory);

    autoChooser.addRoutine("Test Auto 1", autoRoutines::testAuto1);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
    configureNetworkTables();

    coralIntake =
        new CoralIntake(
            Constants.Coral_Intake.CORAL_LEFTINTAKEMOTOR_ID,
            Constants.Coral_Intake.CORAL_WRISTPIVOT_MOTOR_ID,
            Constants.Coral_Intake.CORAL_RIGHTINTAKEMOTOR_ID); // set to arbitrary numbers for now
    algaeIntake = new AlgaeIntake(34, 35, 36, 37);
    elevator =
        new Elevator(
            Constants.elevator.ELEVATOR_ENCODER_ID,
            Constants.elevator.ELEVATOR_LEFTMOTOR_ID,
            Constants.elevator.ELEVATOR_RIGHTMOTOR_ID);

    coralIntake.setDefaultCommand(new DefaultCoralIntakeCommand(coralIntake));
    algaeIntake.setDefaultCommand(new DefaultAlgaeIntakeCommand(algaeIntake));
  }

  private void configureNetworkTables() {
    logger.initSwerveTable(drivetrain.getState());
  }

  public void updateNetworkTables() {
    drivetrain.registerTelemetry(logger::telemeterize);
    // Climber Telemetary
    logger.updateClimbTelemetry(climbSubsystem);
  }

  private void configureBindings() {
    // climber keybinds use D-pad btw
    operator.povUp().whileTrue(new RunCommand(() -> climbSubsystem.setSpeed(0.5), climbSubsystem));
    operator
        .povDown()
        .whileTrue(new RunCommand(() -> climbSubsystem.setSpeed(-0.5), climbSubsystem));
    // up down right and left are for the climbing mechanism's keybinds
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                driver.rightBumper().getAsBoolean() == true
                    ? // if right bumper is pressed then reduce speed of robot
                    drive // coefficients can be changed to driver preferences
                        .withVelocityX(
                            -driver.getLeftY()
                                * drivetrain.getMaxSpeed()
                                / 4) // divide drive speed by 4
                        .withVelocityY(
                            -driver.getLeftX()
                                * drivetrain.getMaxSpeed()
                                / 4) // divide drive speed by 4
                        .withRotationalRate(
                            -driver.getRightX()
                                * drivetrain.getMaxRotation()
                                / 3) // divide turn sppeed by 3
                    : drive
                        .withVelocityX(
                            -driver.getLeftY()
                                * drivetrain
                                    .getMaxSpeed()) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            -driver.getLeftX()
                                * drivetrain.getMaxSpeed()) // Drive left with negative X (left)
                        .withRotationalRate(
                            -driver.getRightX()
                                * drivetrain
                                    .getMaxRotation()) // Drive counterclockwise with negative X
            // (left)
            ));

    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driver
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // coral intake
    operator.rightBumper().whileTrue(new CoralIntakeCommand(coralIntake));

    // algae intake
    operator.rightTrigger().whileTrue(new AlgaeIntakeCommand(algaeIntake));

    // coral outtake
    operator.leftBumper().whileTrue(new RunCommand(() -> coralIntake.move(-1), coralIntake));

    // algae outtake
    operator.leftTrigger().whileTrue(new RunCommand(() -> algaeIntake.move(-1), algaeIntake));

    // elevator down
    operator.povDown().whileTrue(new RunCommand(() -> elevator.moveElevatorDown(), elevator));

    // elevator up
    operator.povUp().whileTrue(new RunCommand(() -> elevator.moveElevatorUp(), elevator));
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
