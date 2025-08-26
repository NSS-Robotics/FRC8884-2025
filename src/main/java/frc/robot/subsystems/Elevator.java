package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {

    private static TalonFX motor = new TalonFX(
        Constants.ElevatorConstants.motorID
    );

    private static CANcoder encoder = new CANcoder(
        Constants.ElevatorConstants.encoder
    );
    private static TalonFXConfiguration talonFXConfig =
        new TalonFXConfiguration();
    private static CurrentLimitsConfigs currentLimitsConfigs =
        new CurrentLimitsConfigs();
    private static Slot0Configs slot0Configs = new Slot0Configs();
    private static Slot0Configs slot0ConfigsL4 = new Slot0Configs();
    private static Slot1Configs slot1Configs = new Slot1Configs();
    private static Slot2Configs slot2Configs = new Slot2Configs();
    private static PositionVoltage elevatorPositionVoltage;

    private final RobotContainer rob;

    public Elevator(RobotContainer rob) {
        this.rob = rob;
        CANcoderConfiguration CANcoderConfig = new CANcoderConfiguration();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

        CANcoderConfig.MagnetSensor.SensorDirection =
            SensorDirectionValue.Clockwise_Positive;
        CANcoderConfig.MagnetSensor.MagnetOffset =
            Constants.ElevatorConstants.magnetSensorOffset;
        encoder.getConfigurator().apply(CANcoderConfig);

        slot0Configs.kP = Constants.ElevatorConstants.upKP;
        slot0Configs.kI = Constants.ElevatorConstants.upKI;
        slot0Configs.kD = Constants.ElevatorConstants.upKD;

        slot0ConfigsL4.kP = Constants.ElevatorConstants.upL4KP;
        slot0ConfigsL4.kI = Constants.ElevatorConstants.upL4KI;
        slot0ConfigsL4.kD = Constants.ElevatorConstants.upL4KD;

        slot1Configs.kP = Constants.ElevatorConstants.downKP;
        slot1Configs.kI = Constants.ElevatorConstants.downKI;
        slot1Configs.kD = Constants.ElevatorConstants.downKD;

        slot2Configs.kP = Constants.ElevatorConstants.algaekP;
        slot2Configs.kI = Constants.ElevatorConstants.algaekI;
        slot2Configs.kD = Constants.ElevatorConstants.algaekD;

        talonFXConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        talonFXConfig.Feedback.FeedbackSensorSource =
            FeedbackSensorSourceValue.RemoteCANcoder;

        motorOutputConfigs.withInverted(InvertedValue.Clockwise_Positive);
        motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);
        currentLimitsConfigs.StatorCurrentLimit =
            Constants.ElevatorConstants.currentLimit;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(slot0Configs);
        motor.getConfigurator().apply(slot1Configs);
        motor.getConfigurator().apply(slot2Configs);
        motor.getConfigurator().apply(motorOutputConfigs);
        motor.getConfigurator().apply(currentLimitsConfigs);
    }

    public void setElevator(double position, int slot) {
        position = Math.max(
            0,
            Math.min(Constants.ElevatorConstants.maxRotations, position)
        );
        SmartDashboard.putNumber("elevator setpoint", position);
        if (slot == 0) {
            if (rob.scoringLevel.equals(Constants.RobotState.l4)) {
                motor.getConfigurator().apply(slot0ConfigsL4);
            } else if (
                rob.scoringLevel.equals(Constants.RobotState.l1) ||
                rob.scoringLevel.equals(Constants.RobotState.l2) ||
                rob.scoringLevel.equals(Constants.RobotState.l3)
            ) {
                motor.getConfigurator().apply(slot0Configs);
            }
        }
        elevatorPositionVoltage = new PositionVoltage(position).withSlot(slot);

        motor.setControl(elevatorPositionVoltage);
    }

    public double getPosition() {
        return encoder.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
            "Elevator Rotations",
            encoder.getPosition().getValueAsDouble()
        );
        SmartDashboard.putNumber(
            "elev current",
            motor.getStatorCurrent().getValueAsDouble()
        );

        SmartDashboard.putBoolean("Align on bumper press", rob.alignOnBumperPress);
    }
}
