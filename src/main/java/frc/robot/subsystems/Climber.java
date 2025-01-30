package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private static TalonFX lMotor = new TalonFX(Constants.ClimberConstants.lMotorID);
    private static TalonFX rMotor = new TalonFX(Constants.ClimberConstants.rMotorID);
    private static Follower leader = new Follower(Constants.ClimberConstants.lMotorID, true);
    private static Slot0Configs slot0configs = new Slot0Configs();
    private static PositionVoltage positionPID;

    /** START: SYSID */
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    voltage -> {
                        lMotor.setVoltage(voltage.in(Volts));
                        rMotor.setVoltage(voltage.in(Volts));
                    },
                    log -> {
                        log.motor("climber")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                lMotor.get() * RobotController.getBatteryVoltage(), Volts))
                                .linearPosition(m_distance.mut_replace(lMotor.getPosition().getValueAsDouble(), Meters))
                                .linearVelocity(
                                        m_velocity.mut_replace(lMotor.getVelocity().getValueAsDouble(),
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

    public Climber() {
        lMotor.clearStickyFaults();
        rMotor.clearStickyFaults();
        lMotor.setNeutralMode(NeutralModeValue.Brake);
        rMotor.setNeutralMode(NeutralModeValue.Brake);
        rMotor.setControl(leader);

        slot0configs.kP = Constants.ClimberConstants.kP;
        slot0configs.kI = Constants.ClimberConstants.kI;
        slot0configs.kD = Constants.ClimberConstants.kD;

        rMotor.getConfigurator().apply(slot0configs);
    }

    public void resetEncoders() {
        lMotor.setPosition(0);
        rMotor.setPosition(0);
    }

    public void setClimber(double position) {
        position = Math.max(0, Math.min(Constants.ElevatorConstants.maxRotations, position));
        positionPID = new PositionVoltage(position);
        lMotor.setControl(positionPID);
    }
}
