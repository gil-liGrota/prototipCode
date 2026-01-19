// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.POM_lib.Joysticks.PomXboxController;
import frc.robot.commands.FirstMotorCommands;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.FirstMotor.FIrstIOSpark;
import frc.robot.subsystems.FirstMotor.First;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOReal;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Subsystems
        private final Swerve drive;
        // private final First first;

        // Controller
        private final PomXboxController driverController = new PomXboxController(0);

        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        private SwerveDriveSimulation driveSimulation = null;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                drive = new Swerve(
                                                new GyroIOPigeon(),
                                                new ModuleIOReal(0),
                                                new ModuleIOReal(1),
                                                new ModuleIOReal(2),
                                                new ModuleIOReal(3));
                                // first = new First(new FIrstIOSpark());
                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations

                                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                                drive = new Swerve(
                                                new GyroIOSim(this.driveSimulation.getGyroSimulation()),
                                                new ModuleIOSim(this.driveSimulation.getModules()[0]),
                                                new ModuleIOSim(this.driveSimulation.getModules()[1]),
                                                new ModuleIOSim(this.driveSimulation.getModules()[2]),
                                                new ModuleIOSim(this.driveSimulation.getModules()[3]));
                                // first = null;
                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                drive = new Swerve(
                                                new GyroIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                });

                                // first = null;

                                break;
                }

                SendableChooser<Command> c = new SendableChooser<>();

                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", c); // TODO use auto builder

                // Set up SysId routines

                // Configure the button bindings
                configureButtonBindings();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                // Default command, normal field-relative drive
                drive.setDefaultCommand(
                                SwerveCommands.joystickDriveRobotRelative(
                                                drive,
                                                () -> driverController.getLeftY() * 0.6,
                                                () -> driverController.getLeftX() * 0.6,
                                                () -> driverController.getRightX() * 0.6));
                driverController.y().onTrue(drive.resetGyroCommand());

                driverController.b().whileTrue(SwerveCommands.joystickDriveRobotRelative(
                                drive,
                                () -> 0.6,
                                () -> 0.0,
                                () -> 0.0));

                // driverController.a().onTrue(FirstMotorCommands.setVoltage(first, 5.0));
        }

        public void displaSimFieldToAdvantageScope() {
                if (Constants.currentMode != Constants.Mode.SIM)
                        return;

                Logger.recordOutput(
                                "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
                Logger.recordOutput(
                                "FieldSimulation/Notes", SimulatedArena.getInstance().getGamePiecesArrayByType("Note"));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.get();
        }
}
