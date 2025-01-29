package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.ControlType;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;

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


    private static TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
    private static Slot0Configs pivotSlot0Configs = new Slot0Configs();
    private static Slot0Configs intakeSlot0Configs = new Slot0Configs();
    private static PositionVoltage pivotPositionVoltage;
    private static VelocityVoltage intakeVelocityVoltage;
    

    public Intake() {
        CANcoderConfiguration pivotCANcoderConfig = new CANcoderConfiguration();

        pivotCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        pivotCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        pivotCANcoderConfig.MagnetSensor.MagnetOffset = 0;
        pivotEncoder.getConfigurator().apply(pivotCANcoderConfig);

        pivotMotorConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
        pivotMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        pivotSlot0Configs.kP = Constants.IntakeConstants.pivotKP;
        pivotSlot0Configs.kI = Constants.IntakeConstants.pivotKI;
        pivotSlot0Configs.kD = Constants.IntakeConstants.pivotKD;

        pivotMotor.getConfigurator().apply(pivotMotorConfig);
        pivotMotor.getConfigurator().apply(pivotSlot0Configs);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        
        intakeSlot0Configs.kP = Constants.IntakeConstants.intakeKP;
        intakeSlot0Configs.kI = Constants.IntakeConstants.intakeKI;
        intakeSlot0Configs.kD = Constants.IntakeConstants.intakeKD;

        intakeMotor.getConfigurator().apply(intakeSlot0Configs);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setPivot(double position) {
        position = Math.max(0, Math.min(Constants.IntakeConstants.pivotMaxRotations, position));

        pivotPositionVoltage = new PositionVoltage(position);

        pivotMotor.setControl(pivotPositionVoltage);
    }

    public void setIntake(double velocity) {
        intakeVelocityVoltage = new VelocityVoltage(velocity);

        intakeMotor.setControl(intakeVelocityVoltage);
    }

}