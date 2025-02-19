package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
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

    /** START: SYSID */
    private final MutVoltage m_intakeAppliedVoltage = Volts.mutable(0);
    private final MutAngle m_intakeAngle = Rotations.mutable(0);
    private final MutAngularVelocity m_intakeAngularVelocity =
        RotationsPerSecond.mutable(0);
    private final MutVoltage m_pivotAppliedVoltage = Volts.mutable(0);
    private final MutAngle m_pivotAngle = Rotations.mutable(0);
    private final MutAngularVelocity m_pivotAngularVelocity =
        RotationsPerSecond.mutable(0);

    private final SysIdRoutine m_intakeSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            voltage -> {
                intakeMotor.setVoltage(voltage.in(Volts));
            },
            log -> {
                log
                    .motor("intake")
                    .voltage(
                        m_intakeAppliedVoltage.mut_replace(
                            intakeMotor.get() *
                            RobotController.getBatteryVoltage(),
                            Volts
                        )
                    )
                    .angularPosition(
                        m_intakeAngle.mut_replace(
                            intakeMotor.getPosition().getValueAsDouble(),
                            Rotations
                        )
                    )
                    .angularVelocity(
                        m_intakeAngularVelocity.mut_replace(
                            intakeMotor.getVelocity().getValueAsDouble(),
                            RotationsPerSecond
                        )
                    );
            },
            this
        )
    );

    private final SysIdRoutine m_pivotSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            voltage -> {
                pivotMotor.setVoltage(voltage.in(Volts));
            },
            log -> {
                log
                    .motor("pivot")
                    .voltage(
                        m_pivotAppliedVoltage.mut_replace(
                            pivotMotor.get() *
                            RobotController.getBatteryVoltage(),
                            Volts
                        )
                    )
                    .angularPosition(
                        m_pivotAngle.mut_replace(
                            pivotEncoder.getPosition().getValueAsDouble(),
                            Rotations
                        )
                    )
                    .angularVelocity(
                        m_pivotAngularVelocity.mut_replace(
                            pivotEncoder.getVelocity().getValueAsDouble(),
                            RotationsPerSecond
                        )
                    );
            },
            this
        )
    );

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
            SensorDirectionValue.Clockwise_Positive;
        pivotCANcoderConfig.MagnetSensor.MagnetOffset =
            Constants.IntakeConstants.encoderOffset;
        pivotEncoder.getConfigurator().apply(pivotCANcoderConfig);

        pivotMotorConfig.Feedback.FeedbackRemoteSensorID =
            pivotEncoder.getDeviceID();
        pivotMotorConfig.Feedback.FeedbackSensorSource =
            FeedbackSensorSourceValue.RemoteCANcoder;

        pivotSlot0Configs.kP = Constants.IntakeConstants.uppivotKP;
        pivotSlot0Configs.kI = Constants.IntakeConstants.uppivotKI;
        pivotSlot0Configs.kD = Constants.IntakeConstants.uppivotKD;
        pivotSlot1Configs.kP = Constants.IntakeConstants.downpivotKD;
        pivotSlot1Configs.kI = Constants.IntakeConstants.downpivotKI;
        pivotSlot1Configs.kD = Constants.IntakeConstants.downpivotKD;

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

        intakeMotor.getConfigurator().apply(intakeSlot0Configs);
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Resets the encoder position.
     */
    public void resetEncoders() {
        intakeMotor.setPosition(0);
    }

    public void setPivot(double position, int slot) {
        position = Math.max(
            0,
            Math.min(Constants.IntakeConstants.pivotMaxRotations, position)
        );

        pivotPositionVoltage = new PositionVoltage(position).withSlot(slot);

        pivotMotor.setControl(pivotPositionVoltage);
    }

    public void setIntake(double velocity) {
        intakeVelocityVoltage = new VelocityVoltage(velocity / 60);

        intakeMotor.setControl(intakeVelocityVoltage);
    }

    public void stopIntake() {
        VoltageOut intakeVoltageOut = new VoltageOut(0);
        intakeMotor.setControl(intakeVoltageOut);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
            "Intake Pivot Encoder",
            pivotEncoder.getPosition().getValueAsDouble()
        );

        SmartDashboard.putNumber(
            "Intake Velocity",
            pivotEncoder.getVelocity().getValueAsDouble()
        );
    }
}
