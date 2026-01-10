package frc.robot.commands;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.Swerve;

public class SwerveCommands {
        private static final double DEADBAND = 0.15;
        private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
        private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

        private SwerveCommands() {
        }

        private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
                // Apply deadband
                double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
                Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

                // Square magnitude for more precise control
                linearMagnitude = linearMagnitude * linearMagnitude;

                // Return new linear velocity
                return new Pose2d(new Translation2d(), linearDirection)
                                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                                .getTranslation();
        }

        /**
         * Field relative drive command using two joysticks (controlling linear and
         * angular velocities).
         */
        public static Command joystickDrive(
                        Swerve drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                        DoubleSupplier omegaSupplier) {
                return Commands.run(
                                () -> {

                                        // Get linear velocity
                                        Translation2d linearVelocity = getLinearVelocityFromJoysticks(
                                                        xSupplier.getAsDouble(),
                                                        ySupplier.getAsDouble());

                                        // Apply rotation deadband
                                        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                                        // if (linearVelocity.getNorm() > 0.3) {
                                        // omega *= 1.3;
                                        // }

                                        // Square rotation value for more precise control
                                        omega = Math.copySign(omega * omega, omega);

                                        // Convert to field relative speeds & send command
                                        ChassisSpeeds speeds = new ChassisSpeeds(
                                                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        omega * drive.getMaxAngularSpeedRadPerSec());
                                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                                        && DriverStation.getAlliance().get() == Alliance.Red;
                                        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                                        speeds,
                                                        isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                                                        : drive.getRotation());

                                        drive.runVelocity(speeds, true);
                                },
                                drive).beforeStarting(Commands.runOnce(drive::resetKinematics, drive));
        }

        public static Command joystickDriveRobotRelative(
                        Swerve drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                        DoubleSupplier omegaSupplier) {
                return Commands.runEnd(
                                () -> {
                                        // Get linear velocity
                                        Translation2d linearVelocity = getLinearVelocityFromJoysticks(
                                                        xSupplier.getAsDouble(),
                                                        ySupplier.getAsDouble());

                                        // Apply rotation deadband
                                        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                                        // Square rotation value for more precise control
                                        omega = Math.copySign(omega * omega, omega);

                                        // Convert to field relative speeds & send command
                                        ChassisSpeeds speeds = new ChassisSpeeds(
                                                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        omega * drive.getMaxAngularSpeedRadPerSec());
                                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                                        && DriverStation.getAlliance().get() == Alliance.Red;
                                        drive.runVelocity(speeds, true);
                                },
                                () -> drive.runVelocity(new ChassisSpeeds(), true),
                                drive).beforeStarting(Commands.runOnce(drive::resetKinematics, drive));
        }

        /** Measures the robot's wheel radius by spinning in a circle. */
        public static Command wheelRadiusCharacterization(Swerve drive) {
                SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
                WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

                return Commands.parallel(
                                // Drive control sequence
                                Commands.sequence(
                                                // Reset acceleration limiter
                                                Commands.runOnce(
                                                                () -> {
                                                                        limiter.reset(0.0);
                                                                }),

                                                // Turn in place, accelerating up to full speed
                                                Commands.run(
                                                                () -> {
                                                                        double speed = limiter.calculate(
                                                                                        WHEEL_RADIUS_MAX_VELOCITY);
                                                                        drive.runVelocity(new ChassisSpeeds(0.0, 0.0,
                                                                                        speed), true);
                                                                },
                                                                drive)),

                                // Measurement sequence
                                Commands.sequence(
                                                // Wait for modules to fully orient before starting measurement
                                                Commands.waitSeconds(1.0),

                                                // Record starting measurement
                                                Commands.runOnce(
                                                                () -> {
                                                                        state.positions = drive
                                                                                        .getWheelRadiusCharacterizationPositions();
                                                                        state.lastAngle = drive.getRotation();
                                                                        state.gyroDelta = 0.0;
                                                                }),

                                                // Update gyro delta
                                                Commands.run(
                                                                () -> {
                                                                        var rotation = drive.getRotation();
                                                                        state.gyroDelta += Math.abs(
                                                                                        rotation.minus(state.lastAngle)
                                                                                                        .getRadians());
                                                                        state.lastAngle = rotation;
                                                                })

                                                                // When cancelled, calculate and print results
                                                                .finallyDo(
                                                                                () -> {
                                                                                        double[] positions = drive
                                                                                                        .getWheelRadiusCharacterizationPositions();
                                                                                        double wheelDelta = 0.0;
                                                                                        for (int i = 0; i < 4; i++) {
                                                                                                wheelDelta += Math.abs(
                                                                                                                positions[i] - state.positions[i])
                                                                                                                / 4.0;
                                                                                        }
                                                                                        double wheelRadius = (state.gyroDelta
                                                                                                        * DriveConstants.driveBaseRadius)
                                                                                                        / wheelDelta;

                                                                                        NumberFormat formatter = new DecimalFormat(
                                                                                                        "#0.000");
                                                                                        // System.out.println(
                                                                                        // "********** Wheel Radius
                                                                                        // Characterization Results
                                                                                        // **********");
                                                                                        // System.out.println(
                                                                                        // "\tWheel Delta: "
                                                                                        // + formatter.format(
                                                                                        // wheelDelta)
                                                                                        // + " radians");
                                                                                        // System.out.println(
                                                                                        // "\tGyro Delta: " + formatter
                                                                                        // .format(state.gyroDelta)
                                                                                        // + " radians");
                                                                                        // System.out.println(
                                                                                        // "\tWheel Radius: "
                                                                                        // + formatter.format(
                                                                                        // wheelRadius)
                                                                                        // + " meters, "
                                                                                        // + formatter.format(
                                                                                        // Units.metersToInches(
                                                                                        // wheelRadius))
                                                                                        // + " inches");
                                                                                })));
        }

        private static class WheelRadiusCharacterizationState {
                double[] positions = new double[4];
                Rotation2d lastAngle = new Rotation2d();
                double gyroDelta = 0.0;
        }

        public static Command stopDrive(Swerve m_drive) {
                return Commands.runOnce(() -> m_drive.stop(), m_drive);
        }
}
