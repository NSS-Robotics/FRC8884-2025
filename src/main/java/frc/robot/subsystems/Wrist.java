package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {

    private static TalonFX motor = new TalonFX(
        Constants.WristConstants.motorID
    );

    private static TalonFXConfiguration motorConfig =
        new TalonFXConfiguration();
    private static Slot0Configs slot0Configs = motorConfig.Slot0;

    private static PositionVoltage pivotPositionVoltage;
    private CANcoder encoder = new CANcoder(Constants.WristConstants.encoder);

    public Wrist() {
        encoder.clearStickyFaults();
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection =
            SensorDirectionValue.Clockwise_Positive;
        canCoderConfig.MagnetSensor.MagnetOffset =
            Constants.WristConstants.magnetSensorOffset;
        encoder.getConfigurator().apply(canCoderConfig);

        slot0Configs.kP = Constants.WristConstants.kP;
        slot0Configs.kI = Constants.WristConstants.kI;
        slot0Configs.kD = Constants.WristConstants.kD;

        motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource =
            FeedbackSensorSourceValue.RemoteCANcoder;

        motor.getConfigurator().apply(motorConfig);
        motor.getConfigurator().apply(slot0Configs);
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void resetEncoders() {
        encoder.setPosition(0);
    }

    public void setWrist(double position) {
        position = Math.max(
            0,
            Math.min(Constants.WristConstants.maxRotations, position)
        );

        pivotPositionVoltage = new PositionVoltage(position);

        motor.setControl(pivotPositionVoltage);
    }

    public double getPosition() {
        return encoder.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return encoder.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
            "Claw Pivot Encoder",
            encoder.getPosition().getValueAsDouble()
        );

        SmartDashboard.putNumber(
            "Claw Pivot Motor",
            motor.getPosition().getValueAsDouble()
        );
    }
}
