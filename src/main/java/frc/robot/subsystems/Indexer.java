package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

    private final SparkMax motor = new SparkMax(
        Constants.IndexerConstants.motorID,
        MotorType.kBrushless
    );
    private final SparkMaxConfig motorConfig;
    private final LaserCan lasercan;
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(
        Constants.IndexerConstants.kS,
        Constants.IndexerConstants.kV,
        Constants.IndexerConstants.kA
    );

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
                    .motor("indexer")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            motor.getAppliedOutput() * motor.getBusVoltage(),
                            Volts
                        )
                    )
                    .angularPosition(
                        m_angle.mut_replace(
                            motor.getEncoder().getPosition(),
                            Rotations
                        )
                    )
                    .angularVelocity(
                        m_angularVelocity.mut_replace(
                            motor.getEncoder().getVelocity(),
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

    public Indexer() {
        motorConfig = new SparkMaxConfig();
        lasercan = new LaserCan(Constants.IndexerConstants.laserCANID);

        // motorConfig.closedLoop
        //     .p(Constants.IndexerConstants.kP)
        //     .i(Constants.IndexerConstants.kI)
        //     .d(Constants.IndexerConstants.kD);

        motorConfig.smartCurrentLimit(40).idleMode(IdleMode.kCoast);
        motor.configure(
            motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        try {
            lasercan.setRangingMode(LaserCan.RangingMode.SHORT);
            lasercan.setRegionOfInterest(
                new LaserCan.RegionOfInterest(8, 8, 16, 16)
            );
            lasercan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    // public void setIndexer(double velocity) {
    //     motor
    //         .getClosedLoopController()
    //         .setReference(ff.calculate(velocity / 60), ControlType.kVelocity);
    // }

    public void setIndexer(double speed) {
        motor.set(speed);
    }

    public void resetEncoders() {
        motor.getEncoder().setPosition(0);
    }

    public boolean gamepieceDetected() {
        double measurement = lasercan.getMeasurement().distance_mm;
        return measurement < 20;
    }

    public void stopIndexer() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
            "Indexer Velocity",
            motor.getEncoder().getVelocity()
        );
        SmartDashboard.putBoolean(
            "Indexer Game Piece Detected",
            gamepieceDetected()
        );
    }
}
