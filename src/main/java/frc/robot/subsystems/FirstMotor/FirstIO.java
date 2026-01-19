package frc.robot.subsystems.FirstMotor;

import org.littletonrobotics.junction.AutoLog;

public interface FirstIO {

    @AutoLog
    public static class FirstIOInputs {
        public double motorPosition = 0.0;
        public double motorVelocity = 0.0;
        public boolean sensor = false;
    }

    public default void updateInputs(FirstIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void stop() {
    }

}
