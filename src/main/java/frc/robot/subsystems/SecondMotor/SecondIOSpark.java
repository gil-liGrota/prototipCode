package frc.robot.subsystems.SecondMotor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.subsystems.FirstMotor.FirstIO.FirstIOInputs;

public class SecondIOSpark implements SecondIO {
    private POMSparkMax motor;
    private RelativeEncoder encoder;
    private SparkMaxConfig config;

    public SecondIOSpark() {
        motor = new POMSparkMax(1);
        encoder = motor.getEncoder();

        config.idleMode(IdleMode.kCoast);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(SecondIOInputs inputs) {
        inputs.motorPosition = encoder.getPosition();
        inputs.motorVelocity = encoder.getVelocity();
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
