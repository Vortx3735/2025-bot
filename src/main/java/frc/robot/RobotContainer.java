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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DefaultAlgaeIntakeCommand;
import frc.robot.commands.DefaultCoralIntakeCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.util.TunerConstants;
import frc.robot.util.VorTXControllerXbox;
import edu.wpi.first.wpilibj2.command.RunCommand;

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

  private final Telemetry logger = new Telemetry(MaxSpeed, MaxAngularRate);

  private final VorTXControllerXbox joystick = new VorTXControllerXbox(0);
  private final VorTXControllerXbox joystick2 = new VorTXControllerXbox(1);

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /* Path follower */
  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser = new AutoChooser();

  public final CoralIntake coralIntake;
  public final AlgaeIntake algaeIntake;

  public RobotContainer() {
    autoFactory = drivetrain.createAutoFactory();
    autoRoutines = new AutoRoutines(autoFactory);

    autoChooser.addRoutine("Test Auto 1", autoRoutines::testAuto1);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
    configureNetworkTables();

    coralIntake = new CoralIntake(31, 32, 33); // set to arbitrary numbers for now
    algaeIntake = new AlgaeIntake(34, 35, 36, 37);
    coralIntake.setDefaultCommand(new DefaultCoralIntakeCommand(coralIntake));
    algaeIntake.setDefaultCommand(new DefaultAlgaeIntakeCommand(algaeIntake));
  }

  private void configureNetworkTables() {
    logger.initSwerveTable(drivetrain.getState());
  }

  public void updateNetworkTables() {
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                joystick.rightBumper().getAsBoolean() == true
                    ? // if right bumper is pressed then reduce speed of robot
                    drive // coefficients can be changed to driver preferences
                        .withVelocityX(
                            -joystick.getLeftY()
                                * drivetrain.getMaxSpeed()
                                / 4) // divide drive speed by 4
                        .withVelocityY(
                            -joystick.getLeftX()
                                * drivetrain.getMaxSpeed()
                                / 4) // divide drive speed by 4
                        .withRotationalRate(
                            -joystick.getRightX()
                                * drivetrain.getMaxRotation()
                                / 3) // divide turn sppeed by 3
                    : drive
                        .withVelocityX(
                            -joystick.getLeftY()
                                * drivetrain
                                    .getMaxSpeed()) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            -joystick.getLeftX()
                                * drivetrain.getMaxSpeed()) // Drive left with negative X (left)
                        .withRotationalRate(
                            -joystick.getRightX()
                                * drivetrain
                                    .getMaxRotation()) // Drive counterclockwise with negative X
            // (left)
            ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    drivetrain.registerTelemetry(logger::telemeterize);
    
        //intake and outtake
    joystick2.a().whileTrue(new RunCommand(() -> coralIntake.move(-1), coralIntake));
    joystick2.b().whileTrue(new RunCommand(() -> coralIntake.move(1), coralIntake));
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
