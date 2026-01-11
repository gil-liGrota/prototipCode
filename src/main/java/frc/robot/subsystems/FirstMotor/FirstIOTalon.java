package frc.robot.subsystems.FirstMotor;

import frc.robot.POM_lib.Motors.POMTalonFX;

public class FirstIOTalon implements FirstIO {
    private POMTalonFX motor;

    public FirstIOTalon() {
        motor = new POMTalonFX(1);
    }

    @Override
    public void updateInputs(FirstIOInputs inputs) {
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
