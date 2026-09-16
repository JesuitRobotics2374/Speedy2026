// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.*;

import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.drivetrain.TunerConstants;

public class Core {

    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.MAX_SPEED;

    public double MaxSpeedTurbo = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * Constants.MAX_SPEED_TURBO;

    public boolean isTurbo = false;

    public double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * Constants.MAX_ANGULAR_RATE;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new
    // SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new
    // SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandNiDsXboxController driveController = new CommandNiDsXboxController(0);

    public final DriveSubsystem drivetrain = TunerConstants.createDrivetrain();

    public Core() {

        registerAutoCommands();
        // autoChooser = AutoBuilder.buildAutoChooser();
        configureBindings();
        configureShuffleBoard();

        // drivetrain.setRobotPose(new Pose2d(7.5, 1.5, new Rotation2d(180 * (Math.PI /
        // 180))));
    }

    // A setpoint is a "macro" state. Find its definition in utils folder.
    // public void moveToSetpoint(Setpoint setpoint) {
    //     queuedRetractAction = setpoint.getRetractAction(); // Store what we just did for when we retract
    //     elevatorSubsystem.elevatorGoToDouble(setpoint.getElevator());
    //     armSubsystem.armGoTo(setpoint.getArm());
    //     armSubsystem.wristGoTo(setpoint.getWrist());
    // }
    public void registerAutoCommands() {
        // NamedCommands.registerCommand("OuttakeCommand", new
        // Outtake(outtakeSubsystem));
        // NamedCommands.registerCommand("Test Pathfind", new PathfindBasic(drivetrain,
        // Constants.TEST_PATHFIND_TARGET));

        // PathfindingCommand.warmupCommand().schedule();
    }

    public void configureShuffleBoard() {
        // Initialize dashboard values (they will be refreshed in corePeriodic)
        // Add the field Sendable so the field view shows on the dashboard
        SmartDashboard.putData("Field", drivetrain.getField());
        SmartDashboard.putNumber("Robot Y", drivetrain.getRobotY());
        SmartDashboard.putNumber("Robot X", drivetrain.getRobotX());
        SmartDashboard.putBoolean("FAST MODE", isTurbo);
    }

    private void configureBindings() {

        // STICK MOVEMENT
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        // .withVelocityX(-driveController.getLeftY() * Constants.MAX_SPEED *
                        // getAxisMovementScale())
                        // .withVelocityY(-driveController.getLeftX() * Constants.MAX_SPEED *
                        // getAxisMovementScale())
                        .withVelocityX(-driveController.getLeftY() * (isTurbo ? MaxSpeedTurbo : MaxSpeed)
                                * getAxisMovementScale() //* elevatorSlowSpeed()
                                )
                        .withVelocityY(-driveController.getLeftX() * (isTurbo ? MaxSpeedTurbo : MaxSpeed)
                                * getAxisMovementScale() //* elevatorSlowSpeed()
                                )
                        .withRotationalRate(-driveController.getRightX() * MaxAngularRate * getAxisMovementScale())));

        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // RESET POSE
    }

    public DriveSubsystem getDrivetrain() {
        return drivetrain;
    }

    public double getAxisMovementScale() {
        return (1 - (driveController.getRightTriggerAxis() * 0.85));
    }


    int clock = 0;

    public void corePeriodic() {
        SmartDashboard.putNumber("Drive Right X", driveController.getRightX());
        SmartDashboard.putNumber("Drive Left X", driveController.getLeftX());
        SmartDashboard.putNumber("Drive Left Y", driveController.getLeftY());
    }
}
