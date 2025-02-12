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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

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

    /** START: SYSID */
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    voltage -> {
                        motor.setVoltage(voltage.in(Volts));
                    },
                    log -> {
                        log.motor("elevator")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                motor.get() * RobotController.getBatteryVoltage(), Volts))
                                .linearPosition(m_distance.mut_replace(motor.getPosition().getValueAsDouble(), Meters))
                                .linearVelocity(
                                        m_velocity.mut_replace(motor.getVelocity().getValueAsDouble(),
                                                MetersPerSecond));
                    },
                    this));

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

    public Elevator() {

        CANcoderConfiguration CANcoderConfig = new CANcoderConfiguration();
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
        motor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setElevator(double position) {
        position = Math.max(0, Math.min(Constants.ElevatorConstants.maxRotations, position));

        elevatorPositionVoltage = new PositionVoltage(position);

        motor.setControl(elevatorPositionVoltage);
    }
}
