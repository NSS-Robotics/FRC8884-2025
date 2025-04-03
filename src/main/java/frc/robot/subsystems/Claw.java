package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import java.io.ObjectInputFilter.Status;

public class Claw extends SubsystemBase {

    private final TalonFX motor = new TalonFX(
        Constants.EndEffectorConstants.motorID
    ); // The motor in the End Effector.
    private VelocityVoltage velocityVoltage;
    private VoltageOut voltageOut;
    private final Slot0Configs slot0Configs = new Slot0Configs();
    private final CurrentLimitsConfigs currentLimitsConfigs =
        new CurrentLimitsConfigs();
    private final LaserCan lasercan = new LaserCan(43);
    private final RobotContainer rob;

    public Claw(RobotContainer rob) {
        slot0Configs.kP = Constants.EndEffectorConstants.kP;
        slot0Configs.kI = Constants.EndEffectorConstants.kI;
        slot0Configs.kD = Constants.EndEffectorConstants.kD;
        slot0Configs.kS = Constants.EndEffectorConstants.kS;
        slot0Configs.kA = Constants.EndEffectorConstants.kA;
        slot0Configs.kV = Constants.EndEffectorConstants.kV;
        this.rob = rob;

        currentLimitsConfigs.SupplyCurrentLimit =
            Constants.EndEffectorConstants.currentLimit;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;

        motor.getConfigurator().apply(slot0Configs);
        motor.getConfigurator().apply(currentLimitsConfigs);
        motor.setNeutralMode(NeutralModeValue.Brake);

        try {
            lasercan.setRangingMode(LaserCan.RangingMode.SHORT);
            lasercan.setRegionOfInterest(
                new LaserCan.RegionOfInterest(8, 8, 10, 10)
            );
            lasercan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    /**
     * Sends the correct amount of voltage to the motor to move it at the given
     * velocity.
     *
     * @param velocity The velocity at which to spin the motor.
     */
    public void setClaw(double velocity) {
        velocityVoltage = new VelocityVoltage(velocity / 60);

        motor.setControl(velocityVoltage);
    }

    public void stopClaw() {
        voltageOut = new VoltageOut(0);

        motor.setControl(voltageOut);
    }

    /**
     * Resets the motor encoder positions to 0.
     */
    public void resetEncoders() {
        motor.setPosition(0);
    }

    public boolean gamePieceDetected() {
        Measurement m = lasercan.getMeasurement();
        if (m == null) {
            return false;
        }
        SmartDashboard.putNumber("LaserCAN dist", m.distance_mm);
        return m.distance_mm < (rob.isCoral ? 80 : 120);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
            "Claw Velocity",
            motor.getVelocity().getValueAsDouble()
        );
        SmartDashboard.putBoolean(
            "Claw Game Piece Detected",
            gamePieceDetected()
        );

        SmartDashboard.putNumber(
            "Claw Current",
            motor.getStatorCurrent().getValueAsDouble()
        );
    }
}
