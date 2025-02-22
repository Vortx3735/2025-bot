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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.defaultcommands.*;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.util.TunerConstants;
import frc.robot.util.VorTXControllerXbox;

public class RobotContainer {
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

  public final Telemetry logger = new Telemetry(MaxSpeed, MaxAngularRate);

  private final VorTXControllerXbox driver = new VorTXControllerXbox(0);
  private final VorTXControllerXbox operator = new VorTXControllerXbox(1);

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final CoralIntake coralIntake =
      new CoralIntake(
          Constants.CoralConstants.CORAL_LEFTINTAKEMOTOR_ID,
          Constants.CoralConstants.CORAL_RIGHTINTAKEMOTOR_ID,
          Constants.CoralConstants.CORAL_WRISTPIVOT_MOTOR_ID,
          Constants.CoralConstants.CORAL_WRISTPIVOT_ENCODER_ID);

  private final AlgaeIntake algaeIntake =
      new AlgaeIntake(
          Constants.AlgaeConstants.LEFTINTAKE_MOTOR_ID,
          Constants.AlgaeConstants.RIGHTINTAKE_MOTOR_ID,
          Constants.AlgaeConstants.WRISTPIVOT_MOTOR_ID,
          Constants.AlgaeConstants.WRISTPIVOT_ENCODER_ID);

  private final Elevator elevator =
      new Elevator(
          Constants.ElevatorConstants.ELEVATOR_ENCODER_ID,
          Constants.ElevatorConstants.ELEVATOR_LEFTMOTOR_ID,
          Constants.ElevatorConstants.ELEVATOR_RIGHTMOTOR_ID);

  private final ClimbSubsystem climbSubsystem =
      new ClimbSubsystem(
          Constants.ClimberConstants.CLIMBER_LEFTMOTOR_ID,
          Constants.ClimberConstants.CLIMBER_RIGHTMOTOR_ID);

  /* Path follower */
  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser = new AutoChooser();

  private Trigger coralDetected;
  private Trigger coralNotDetected;

  public RobotContainer() {
    configureTriggers();
    configureBindings();
    configureNetworkTables();
    // Auton
    autoFactory = drivetrain.createAutoFactory();
    autoRoutines = new AutoRoutines(autoFactory);

    autoChooser.addRoutine("Test Auto 1", autoRoutines::testAuto1);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    elevator.publishInitialValues();
    coralIntake.publishInitialValues();
    algaeIntake.publishInitialValues();
    climbSubsystem.publishInitialValues();

    // default commands
    coralIntake.setDefaultCommand(new DefaultCoralIntakeCommand(coralIntake));
    algaeIntake.setDefaultCommand(new DefaultAlgaeIntakeCommand(algaeIntake));
    climbSubsystem.setDefaultCommand(new DefaultClimbCommand(climbSubsystem));
    elevator.setDefaultCommand(new DefaultElevatorCommand(elevator));

    elevator.configureTalonFX();
  }

  private void configureNetworkTables() {
    logger.initSwerveTable(drivetrain.getState());
  }

  public void updateNetworkTables() {
    drivetrain.registerTelemetry(logger::telemeterize);

    // Climber Telemetry
    logger.updateClimbTelemetry(climbSubsystem);
  }

  private void configureBindings() {
    // climber keybinds use D-pad btw
    // up down right and left are for the climbing mechanism's keybinds
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                driver.rb.getAsBoolean() == true
                    ? // if right bumper is pressed then reduce speed of robot
                    drive // coefficients can be changed to driver preferences
                        .withVelocityX(
                            -driver.getLeftY()
                                * drivetrain.getMaxSpeed()
                                / 10) // divide drive speed by 4
                        .withVelocityY(
                            -driver.getLeftX()
                                * drivetrain.getMaxSpeed()
                                / 10) // divide drive speed by 4
                        .withRotationalRate(
                            -driver.getRightX()
                                * drivetrain.getMaxRotation()
                                / 8) // divide turn sppeed by 3
                    : driver.lb.getAsBoolean() == true
                        ? drive
                            .withVelocityX(
                                -driver.getLeftY()
                                    * drivetrain.getMaxSpeed()
                                    / 7) // Drive forward with negative Y
                            // (forward)
                            .withVelocityY(
                                -driver.getLeftX()
                                    * drivetrain.getMaxSpeed()
                                    / 7) // Drive left with negative X (left)
                            .withRotationalRate(
                                -driver.getRightX() * drivetrain.getMaxRotation() / 4)
                        : drive
                            .withVelocityX(
                                -driver.getLeftY()
                                    * drivetrain.getMaxSpeed()) // Drive forward with negative Y
                            // (forward)
                            .withVelocityY(
                                -driver.getLeftX()
                                    * drivetrain.getMaxSpeed()) // Drive left with negative X (left)
                            .withRotationalRate(
                                -driver.getRightX()
                                    * drivetrain.getMaxRotation()) // Drive counterclockwise with
            // negative X
            // (left)
            ));

    driver.aButton.whileTrue(drivetrain.applyRequest(() -> brake));
    driver.bButton.whileTrue(
        drivetrain.applyRequest(
            () ->
                point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driver.view.and(driver.yButton).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driver.view.and(driver.xButton).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driver.menu.and(driver.yButton).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driver.menu.and(driver.xButton).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on menu button
    driver.menu.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // climber up
    driver.rt.whileTrue(new RunCommand(() -> climbSubsystem.move(0.1), climbSubsystem));
    // climber down
    driver.lt.whileTrue(new RunCommand(() -> climbSubsystem.move(-0.2), climbSubsystem));

    operator.povLeft.whileTrue(new RunCommand(() -> coralIntake.moveWristUp(), coralIntake));
    operator.povRight.whileTrue(new RunCommand(() -> coralIntake.moveWristDown(), coralIntake));

    operator.aButton.whileTrue(
        Commands.parallel(
            new RunCommand(() -> coralIntake.moveWristToHP(), coralIntake),
            new RunCommand(() -> elevator.moveElevatorToHP(), elevator)));
    operator.xButton.whileTrue(new RunCommand(() -> elevator.moveElevatorToHP(), elevator));
    operator.yButton.whileTrue(
        new RunCommand(() -> coralIntake.moveWristToPosition(-0.51), coralIntake)); // L2/L3
    // coral intake
    operator
        .lt
        .and(coralNotDetected)
        .whileTrue(new RunCommand(() -> coralIntake.intake(), coralIntake));
    // coral outtake
    operator.rt.whileTrue(
        new SequentialCommandGroup(
            new RunCommand(() -> coralIntake.outtake(), coralIntake),
            new RunCommand(() -> algaeIntake.intake(), algaeIntake)));
    // algae outtake
    operator.rb.whileTrue(new RunCommand(() -> algaeIntake.outtake(), algaeIntake));
    // algae intake
    operator.lb.whileTrue(new RunCommand(() -> algaeIntake.intake(), algaeIntake));
    // elevator up
    operator.povUp.whileTrue(new RunCommand(() -> elevator.moveElevatorUp(), elevator));
    // elevator down
    operator.povDown.whileTrue(new RunCommand(() -> elevator.moveElevatorDown(), elevator));
  }

  private void configureTriggers() {
    coralDetected = new Trigger(() -> coralIntake.getCoralDetectedBoolean());
    coralNotDetected = coralDetected.negate();

    coralDetected.onTrue(
        new WaitCommand(0.2).andThen(new RunCommand(() -> coralIntake.stopIntake(), coralIntake)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
