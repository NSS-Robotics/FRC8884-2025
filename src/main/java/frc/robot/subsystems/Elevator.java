package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
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

public class Elevator extends SubsystemBase {

    private static TalonFX motor = new TalonFX(
        Constants.ElevatorConstants.motorID
    );

    private static CANcoder encoder = new CANcoder(
        Constants.ElevatorConstants.encoder
    );
    private static TalonFXConfiguration talonFXConfig =
        new TalonFXConfiguration();
    private static Slot0Configs slot0Configs = new Slot0Configs();
    private static Slot1Configs slot1Configs = new Slot1Configs();
    private static PositionVoltage elevatorPositionVoltage;

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
                    .motor("elevator")
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

    public Elevator() {
        CANcoderConfiguration CANcoderConfig = new CANcoderConfiguration();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

        CANcoderConfig.MagnetSensor.SensorDirection =
            SensorDirectionValue.Clockwise_Positive;
        CANcoderConfig.MagnetSensor.MagnetOffset =
            Constants.ElevatorConstants.magnetSensorOffset;
        encoder.getConfigurator().apply(CANcoderConfig);

        slot0Configs.kP = Constants.ElevatorConstants.intialkP;
        slot0Configs.kI = Constants.ElevatorConstants.intialkI;
        slot0Configs.kD = Constants.ElevatorConstants.intialkD;

        slot1Configs.kP = Constants.ElevatorConstants.middlekP;
        slot1Configs.kI = Constants.ElevatorConstants.middlekI;
        slot1Configs.kD = Constants.ElevatorConstants.middlekD;

        talonFXConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        talonFXConfig.Feedback.FeedbackSensorSource =
            FeedbackSensorSourceValue.RemoteCANcoder;

        motorOutputConfigs.withInverted(InvertedValue.Clockwise_Positive);
        motorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);
        motor.getConfigurator().apply(slot0Configs);
        motor.getConfigurator().apply(slot1Configs);
        motor.getConfigurator().apply(motorOutputConfigs);
    }

    public void setElevator(double position, int slot) {
        position = Math.max(
            0,
            Math.min(Constants.ElevatorConstants.maxRotations, position)
        );

        elevatorPositionVoltage = new PositionVoltage(position).withSlot(slot);

        motor.setControl(elevatorPositionVoltage);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
            "Elevator Rotations",
            encoder.getPosition().getValueAsDouble()
        );
    }
}
