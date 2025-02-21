package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class ClawPivot extends SubsystemBase {

    private static TalonFX motor = new TalonFX(
        Constants.ClawPivotConstants.motorID
    );

    private static TalonFXConfiguration motorConfig =
        new TalonFXConfiguration();
    private static Slot0Configs slot0Configs = motorConfig.Slot0;

    private static PositionVoltage pivotPositionVoltage;
    private CANcoder encoder = new CANcoder(
        Constants.ClawPivotConstants.encoder
    );

    public ClawPivot() {
        encoder.clearStickyFaults();
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        canCoderConfig.MagnetSensor.SensorDirection =
            SensorDirectionValue.Clockwise_Positive;
        canCoderConfig.MagnetSensor.MagnetOffset =
            Constants.ClawPivotConstants.magnetSensorOffset;
        encoder.getConfigurator().apply(canCoderConfig);

        slot0Configs.kP = Constants.ClawPivotConstants.kP;
        slot0Configs.kI = Constants.ClawPivotConstants.kI;
        slot0Configs.kD = Constants.ClawPivotConstants.kD;

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

    public void setPivot(double position) {
        position = Math.max(
            0,
            Math.min(Constants.ClawPivotConstants.maxRotations, position)
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
