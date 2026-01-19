package frc.robot.subsystems.FirstMotor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class FIrstIOSpark implements FirstIO {

    private POMSparkMax motor;
    private RelativeEncoder encoder;
    private SparkMaxConfig config;
    private POMDigitalInput sensor;

    public FIrstIOSpark() {
        // motor = new POMSparkMax(1);
        // encoder = motor.getEncoder();
        sensor = new POMDigitalInput(0);

        // config.idleMode(IdleMode.kCoast);

        // motor.configure(config, ResetMode.kNoResetSafeParameters,
        // PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(FirstIOInputs inputs) {
        // inputs.motorPosition = encoder.getPosition();
        // inputs.motorVelocity = encoder.getVelocity();
        inputs.sensor = sensor.get();
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
