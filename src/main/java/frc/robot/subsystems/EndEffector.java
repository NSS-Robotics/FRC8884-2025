package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
    private final TalonFX motor;
    private VelocityVoltage velocityVoltage;
    private final Slot0Configs slot0Configs = new Slot0Configs();
    private final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
  
    public EndEffector() {
        motor = new TalonFX(Constants.EndEffectorConstants.motorID);

        slot0Configs.kP = Constants.EndEffectorConstants.kP;
        slot0Configs.kI = Constants.EndEffectorConstants.kI;
        slot0Configs.kD = Constants.EndEffectorConstants.kD;

        currentLimitsConfigs.SupplyCurrentLimit = Constants.EndEffectorConstants.currentLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;

        motor.getConfigurator().apply(slot0Configs);
        motor.getConfigurator().apply(currentLimitsConfigs);
    }

    public void setEndEffector(double velocity) {
        velocity = Math.max(0, Math.min(Constants.ElevatorConstants.maxRotations, velocity));
        velocityVoltage = new VelocityVoltage(velocity);

        motor.setControl(velocityVoltage);
    }

    public void resetEncoders() {
        motor.setPosition(0);
    }
}