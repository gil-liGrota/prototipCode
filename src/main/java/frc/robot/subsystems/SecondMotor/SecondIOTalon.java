package frc.robot.subsystems.SecondMotor;

import frc.robot.POM_lib.Motors.POMTalonFX;
import frc.robot.subsystems.FirstMotor.FirstIO.FirstIOInputs;

public class SecondIOTalon implements SecondIO {
    private POMTalonFX motor;

    public SecondIOTalon() {
        motor = new POMTalonFX(1);
    }

    @Override
    public void updateInputs(SecondIOInputs inputs) {
        inputs.motorPosition = motor.getPosition().getValueAsDouble();
        inputs.motorVelocity = motor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        motor.stop();
    }

}
