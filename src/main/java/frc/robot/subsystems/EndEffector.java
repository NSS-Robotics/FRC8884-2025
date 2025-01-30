package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Constants.EndEffectorConstants.motorID); // The motor in the End Effector.
    private VelocityVoltage velocityVoltage;
    private final Slot0Configs slot0Configs = new Slot0Configs();
    private final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

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
                        log.motor("motor")
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

    public EndEffector() {
        slot0Configs.kP = Constants.EndEffectorConstants.kP;
        slot0Configs.kI = Constants.EndEffectorConstants.kI;
        slot0Configs.kD = Constants.EndEffectorConstants.kD;

        currentLimitsConfigs.SupplyCurrentLimit = Constants.EndEffectorConstants.currentLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;

        motor.getConfigurator().apply(slot0Configs);
        motor.getConfigurator().apply(currentLimitsConfigs);
    }

    /**
     * Sends the correct amount of voltage to the motor to move it at the given
     * velocity.
     * 
     * @param velocity The velocity at which to spin the motor.
     */
    public void setEndEffector(double velocity) {
        velocity = Math.max(0, Math.min(Constants.ElevatorConstants.maxRotations, velocity));
        velocityVoltage = new VelocityVoltage(velocity);

        motor.setControl(velocityVoltage);
    }

    /**
     * Resets the motor encoder positions to 0.
     */
    public void resetEncoders() {
        motor.setPosition(0);
    }
}