package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private static TalonFX pivotMotor = new TalonFX(
        Constants.IntakeConstants.pivotMotorID
    );

    private static TalonFX intakeMotor = new TalonFX(
        Constants.IntakeConstants.intakeMotorID
    );

    private static CANcoder pivotEncoder = new CANcoder(
        Constants.IntakeConstants.encoderID
    );

    CurrentLimitsConfigs intakeCurrentLimitsConfigs =
        new CurrentLimitsConfigs();

    private static TalonFXConfiguration pivotMotorConfig =
        new TalonFXConfiguration();
    private static Slot0Configs pivotSlot0Configs = new Slot0Configs();
    private static Slot1Configs pivotSlot1Configs = new Slot1Configs();
    private static Slot0Configs intakeSlot0Configs = new Slot0Configs();
    private static PositionVoltage pivotPositionVoltage;
    private static VelocityVoltage intakeVelocityVoltage;

    public Intake() {
        CANcoderConfiguration pivotCANcoderConfig = new CANcoderConfiguration();
        pivotCANcoderConfig.MagnetSensor.SensorDirection =
            SensorDirectionValue.CounterClockwise_Positive;
        pivotCANcoderConfig.MagnetSensor.MagnetOffset =
            Constants.IntakeConstants.encoderOffset;
        pivotEncoder.getConfigurator().apply(pivotCANcoderConfig);

        pivotMotorConfig.Feedback.FeedbackRemoteSensorID =
            pivotEncoder.getDeviceID();
        pivotMotorConfig.Feedback.FeedbackSensorSource =
            FeedbackSensorSourceValue.RemoteCANcoder;

        pivotSlot0Configs.kP = Constants.IntakeConstants.upKP;
        pivotSlot0Configs.kI = Constants.IntakeConstants.upKI;
        pivotSlot0Configs.kD = Constants.IntakeConstants.upKD;
        pivotSlot1Configs.kP = Constants.IntakeConstants.downKP;
        pivotSlot1Configs.kI = Constants.IntakeConstants.downKI;
        pivotSlot1Configs.kD = Constants.IntakeConstants.downKD;

        pivotMotor.getConfigurator().apply(pivotMotorConfig);
        pivotMotor.getConfigurator().apply(pivotSlot0Configs);
        pivotMotor.getConfigurator().apply(pivotSlot1Configs);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        intakeSlot0Configs.kP = Constants.IntakeConstants.intakeKP;
        intakeSlot0Configs.kI = Constants.IntakeConstants.intakeKI;
        intakeSlot0Configs.kD = Constants.IntakeConstants.intakeKD;
        intakeSlot0Configs.kS = Constants.IntakeConstants.intakeKS;
        intakeSlot0Configs.kV = Constants.IntakeConstants.intakeKV;
        intakeSlot0Configs.kA = Constants.IntakeConstants.intakeKA;
        intakeCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
        intakeCurrentLimitsConfigs.StatorCurrentLimit = 55;

        intakeMotor.getConfigurator().apply(intakeSlot0Configs);
        intakeMotor.getConfigurator().apply(intakeCurrentLimitsConfigs);
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Resets the encoder position.
     */
    public void resetEncoders() {
        intakeMotor.setPosition(0);
    }

    public double getIntakeCurrent() {
        return intakeMotor.getStatorCurrent().getValueAsDouble();
    }

    public void setPivot(double position, int slot) {
        position = Math.min(
            0,
            Math.max(Constants.IntakeConstants.pivotMaxRotations, position)
        );

        pivotPositionVoltage = new PositionVoltage(position).withSlot(slot);

        pivotMotor.setControl(pivotPositionVoltage);
    }

    public void setIntake(double velocity, boolean l1) {
        intakeVelocityVoltage = new VelocityVoltage(velocity / 60);
        intakeCurrentLimitsConfigs.StatorCurrentLimit = 80;
        intakeMotor.getConfigurator().apply(intakeCurrentLimitsConfigs);

        intakeMotor.setControl(intakeVelocityVoltage);
    }

    public void stopIntake() {
        VoltageOut intakeVoltageOut = new VoltageOut(0);
        intakeMotor.setControl(intakeVoltageOut);
    }

    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
            "Intake Pivot Encoder",
            pivotEncoder.getPosition().getValueAsDouble()
        );

        SmartDashboard.putNumber(
            "Intake Motor Encoder",
            pivotMotor.getPosition().getValueAsDouble()
        );

        SmartDashboard.putNumber(
            "Intake Velocity",
            pivotEncoder.getVelocity().getValueAsDouble()
        );
        SmartDashboard.putNumber("Intake Current", getIntakeCurrent());
    }
}
