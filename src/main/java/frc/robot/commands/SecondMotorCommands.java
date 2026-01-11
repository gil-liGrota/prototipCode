package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SecondMotor.Second;

public class SecondMotorCommands {

    public static Command setVoltage(Second second, double voltage) {
        return Commands.startEnd(() -> second.getIO().setVoltage(voltage),
                () -> second.getIO().stop(), second);
    }

}
