package frc.robot.subsystems.SecondMotor;

import org.littletonrobotics.junction.AutoLog;

public interface SecondIO {

    @AutoLog
    public static class SecondIOInputs {
        public double motorPosition = 0.0;
        public double motorVelocity = 0.0;
    }

    public default void updateInputs(SecondIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void stop() {
    }

}