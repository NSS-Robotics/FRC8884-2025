package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;

public class Elevator extends SubsystemBase {

    private static TalonFX motor = new TalonFX(
        Constants.ElevatorConstants.motorID
    );

    private static CANcoder encoder = new CANcoder(
        Constants.ElevatorConstants.encoder
    );
    private static TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    private static Slot0Configs slot0Configs = new Slot0Configs();
    private static PositionVoltage elevatorPositionVoltage;
    

    public Elevator() {

        CANcoderConfiguration CANcoderConfig = new CANcoderConfiguration();

        CANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CANcoderConfig.MagnetSensor.MagnetOffset = 0;
        encoder.getConfigurator().apply(CANcoderConfig);

        slot0Configs.kP = Constants.ElevatorConstants.kP;
        slot0Configs.kI = Constants.ElevatorConstants.kI;
        slot0Configs.kD = Constants.ElevatorConstants.kD;

        talonFXConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        motor.getConfigurator().apply(talonFXConfig);
        motor.getConfigurator().apply(slot0Configs);
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setElevator(double position) {
        position = Math.max(0, Math.min(Constants.ElevatorConstants.maxRotations, position));

        elevatorPositionVoltage = new PositionVoltage(position);

        motor.setControl(elevatorPositionVoltage);
    }
}
