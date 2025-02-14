package frc.robot.subsystems;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;

public class Intake extends SubsystemBase {
    private static TalonFX pivotMotor = new TalonFX(
            Constants.IntakeConstants.pivotMotorID);

    private static TalonFX intakeMotor = new TalonFX(
            Constants.IntakeConstants.intakeMotorID);

    private static CANcoder pivotEncoder = new CANcoder(
            Constants.IntakeConstants.encoderID);

    /** START: SYSID */
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
    private final MutVoltage m_pivotAppliedVoltage = Volts.mutable(0);
    private final MutDistance m_pivotDistance = Meters.mutable(0);
    private final MutLinearVelocity m_pivotVelocity = MetersPerSecond.mutable(0);

    private final SysIdRoutine m_intakeSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    voltage -> {
                        intakeMotor.setVoltage(voltage.in(Volts));
                    },
                    log -> {
                        log.motor("intake")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                intakeMotor.get() * RobotController.getBatteryVoltage(), Volts))
                                .linearPosition(
                                        m_distance.mut_replace(intakeMotor.getPosition().getValueAsDouble(), Meters))
                                .linearVelocity(
                                        m_velocity.mut_replace(intakeMotor.getVelocity().getValueAsDouble(),
                                                MetersPerSecond));
                    },
                    this));

    private final SysIdRoutine m_pivotSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    voltage -> {
                        pivotMotor.setVoltage(voltage.in(Volts));
                    },
                    log -> {
                        log.motor("pivot")
                                .voltage(
                                        m_pivotAppliedVoltage.mut_replace(
                                                pivotMotor.get() * RobotController.getBatteryVoltage(), Volts))
                                .linearPosition(
                                        m_pivotDistance.mut_replace(pivotMotor.getPosition().getValueAsDouble(),
                                                Meters))
                                .linearVelocity(
                                        m_pivotVelocity.mut_replace(pivotMotor.getVelocity().getValueAsDouble(),
                                                MetersPerSecond));
                    },
                    this));

    /**
     * Returns a command that will execute a quasistatic test in the given
     * direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command intakeSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return m_intakeSysIdRoutine.quasistatic(dir);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     * 
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command intakeSysIdDynamic(SysIdRoutine.Direction dir) {
        return m_intakeSysIdRoutine.dynamic(dir);
    }

    /**
     * Returns a command that will execute a quasistatic test in the given
     * direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command pivotSysIdQuasistatic(SysIdRoutine.Direction dir) {
        return m_pivotSysIdRoutine.quasistatic(dir);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     * 
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command pivotSysIdDynamic(SysIdRoutine.Direction dir) {
        return m_pivotSysIdRoutine.dynamic(dir);
    }

    /* END: SYSID */

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
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Resets the encoder position.
     */
    public void resetEncoders() {
        intakeMotor.setPosition(0);
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