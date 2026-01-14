package frc.robot.subsystems.FirstMotor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static frc.robot.subsystems.FirstMotor.FirstConst.*;

import frc.robot.POM_lib.Motors.POMSparkMax;

public class FIrstIOSpark implements FirstIO {

    private POMSparkMax motor;
    // private RelativeEncoder encoder;
    private AnalogInput encoder;
    private SparkMaxConfig config;
    private AnalogPotentiometer absEncoder;

    public FIrstIOSpark() {
        // motor = new POMSparkMax(1);
        absEncoder = new AnalogPotentiometer(1, 100);

        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);

        // motor.configure(config, ResetMode.kNoResetSafeParameters,
        // PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(FirstIOInputs inputs) {
        inputs.motorPosition = absEncoder.get();
        // inputs.motorVelocity = absEncoder.();
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
