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

public class Pivot extends SubsystemBase {

    private static TalonFX motor = new TalonFX(
        Constants.PivotConstants.motorID
    );

    private static TalonFXConfiguration motorConfig =
        new TalonFXConfiguration();
    private static Slot0Configs slot0Configs = motorConfig.Slot0;

    private static PositionVoltage pivotPositionVoltage;
    private CANcoder encoder = new CANcoder(Constants.PivotConstants.encoder);

    /** START: SYSID */
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutAngle m_angle = Rotations.mutable(0);
    private final MutAngularVelocity m_angularVelocity =
        RotationsPerSecond.mutable(0);

    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            voltage -> {
                motor.setVoltage(voltage.in(Volts));
            },
            log -> {
                log
                    .motor("pivot")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            motor.get() * RobotController.getBatteryVoltage(),
                            Volts
                        )
                    )
                    .angularPosition(
                        m_angle.mut_replace(
                            encoder.getPosition().getValueAsDouble(),
                            Rotations
                        )
                    )
                    .angularVelocity(
                        m_angularVelocity.mut_replace(
                            encoder.getVelocity().getValueAsDouble(),
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
    public Command sysIdQuasistatic(SysIdRoutine.Direction dir) {
        return m_sysIdRoutine.quasistatic(dir);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction dir) {
        return m_sysIdRoutine.dynamic(dir);
    }

    /* END: SYSID */

    private Swerve m_swerve;
    private double yOffset;

    public Pivot(Swerve swerve) {
        yOffset = 0;

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        canCoderConfig.MagnetSensor.SensorDirection =
            SensorDirectionValue.CounterClockwise_Positive;
        canCoderConfig.MagnetSensor.MagnetOffset =
            Constants.PivotConstants.magnetSensorOffset;
        encoder.getConfigurator().apply(canCoderConfig);

        slot0Configs.kP = Constants.PivotConstants.kP;
        slot0Configs.kI = Constants.PivotConstants.kI;
        slot0Configs.kD = Constants.PivotConstants.kD;

        motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource =
            FeedbackSensorSourceValue.RemoteCANcoder;

        motor.getConfigurator().apply(motorConfig);
        motor.getConfigurator().apply(slot0Configs);
        motor.setNeutralMode(NeutralModeValue.Brake);

        m_swerve = swerve;
    }

    public void resetEncoders() {
        encoder.setPosition(0);
    }

    public void setPivot(double position) {
        position = Math.max(
            0,
            Math.min(Constants.PivotConstants.maxRotations, position)
        );

        pivotPositionVoltage = new PositionVoltage(position);

        motor.setControl(pivotPositionVoltage);
    }

    public double getEncoderPosition() {
        return encoder.getPosition().getValueAsDouble();
    }

    public double getEncoderVelocity() {
        return encoder.getPosition().getValueAsDouble();
    }

    public void setYOffset(double y) {
        yOffset = y;
    }

    public double getYOffset() {
        return yOffset;
    }

    public void changeYOffset(double amount) {
        yOffset += amount;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Y Offset", yOffset);
    }
}
