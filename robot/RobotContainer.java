package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.MechanismSubsystem;

public class RobotContainer {

    private double MaxSpeed = 2.0 * Constants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05)
            .withRotationalDeadband(MaxAngularRate * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // KUMANDALAR
    Joystick joystick = new Joystick(0);
    Joystick joystickMert = new Joystick(1);

    public final CommandSwerveDrivetrain drivetrain = Constants.createDrivetrain();
    private final MechanismSubsystem mech = new MechanismSubsystem();
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public RobotContainer() {
        configureBindings();
        drivetrain.seedFieldCentric();
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {

                double vx = -joystick.getRawAxis(1) * MaxSpeed;
                double vy = -joystick.getRawAxis(0) * MaxSpeed;
                double omega = -joystick.getRawAxis(4) * MaxAngularRate;

                return drive
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withRotationalRate(omega);
            })
        );

        final var idle = new SwerveRequest.Idle();

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // LB butonu Brake
        new Trigger(() -> joystick.getRawButton(5))
            .whileTrue(drivetrain.applyRequest(() -> brake));
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // B butonu Wheel pointing
        /* 
        new Trigger(() -> joystick.getRawButton(2))
            .whileTrue(
                drivetrain.applyRequest(() ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getRawAxis(1), -joystick.getRawAxis(0))
                    )
                )
            );
            */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // SysId
        /* 
        new Trigger(() -> joystick.getRawButton(7) && joystick.getRawButton(4))
            .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));

        new Trigger(() -> joystick.getRawButton(7) && joystick.getRawButton(3))
            .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        new Trigger(() -> joystick.getRawButton(8) && joystick.getRawButton(4))
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

        new Trigger(() -> joystick.getRawButton(8) && joystick.getRawButton(3))
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
        */
    
    // INTAKE (A)
        new Trigger(() -> joystick.getRawButton(5))
            .whileTrue(Commands.run(() -> mech.intakeStart(), mech))
            .onFalse(Commands.runOnce(() -> mech.intakeStop(), mech));

    // INDEXER (B)
        new Trigger(() -> joystickMert.getRawButton(2))
            .whileTrue(Commands.run(() -> mech.indexerForward(), mech))
            .onFalse(Commands.runOnce(() -> mech.indexerStop(), mech));

    // SHOOTER (X)
        new Trigger(() -> joystickMert.getRawButton(3))
            .whileTrue(Commands.run(() -> mech.shooterShoot(), mech))
            .onFalse(Commands.runOnce(() -> mech.shooterStop(), mech));

    // ELEVATOR UP (Y)
        new Trigger(() -> joystickMert.getRawButton(4))
            .whileTrue(Commands.run(() -> mech.elevatorUp(), mech))
            .onFalse(Commands.runOnce(() -> mech.elevatorStop(), mech));

    // ELEVATOR DOWN (LB)
        new Trigger(() -> joystickMert.getRawButton(5))
            .whileTrue(Commands.run(() -> mech.elevatorDown(), mech))
            .onFalse(Commands.runOnce(() -> mech.elevatorStop(), mech));
        }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Command getAutonomousCommand() {

        final var idle = new SwerveRequest.Idle();

        return Commands.sequence(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),

            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                     .withVelocityY(0)
                     .withRotationalRate(0)
            ).withTimeout(1.0),

            drivetrain.applyRequest(() -> idle)
        );
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////