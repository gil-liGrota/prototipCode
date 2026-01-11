package frc.robot.subsystems.SecondMotor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Second extends SubsystemBase {

    private SecondIO io;
    public SecondIOInputsAutoLogged inputs = new SecondIOInputsAutoLogged();

    public Second(SecondIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Second", inputs);
    }

    public SecondIO getIO() {
        return io;
    }
}
