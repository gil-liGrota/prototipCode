package frc.robot.subsystems.FirstMotor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class First extends SubsystemBase {

    private FirstIO io;
    public FirstIOInputsAutoLogged inputs = new FirstIOInputsAutoLogged();

    public First(FirstIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("First", inputs);
    }

    public FirstIO getIO() {
        return io;
    }
}
