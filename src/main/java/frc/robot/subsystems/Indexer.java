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

    public Indexer() {
        motorConfig = new SparkMaxConfig();
        lasercan = new LaserCan(Constants.IndexerConstants.laserCANID);

        // motorConfig.closedLoop
        // .p(Constants.IndexerConstants.kP)
        // .i(Constants.IndexerConstants.kI)
        // .d(Constants.IndexerConstants.kD);

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
    // motor
    // .getClosedLoopController()
    // .setReference(ff.calculate(velocity / 60), ControlType.kVelocity);
    // }

    public void setIndexer(double speed) {
        motor.set(speed);
    }

    public void resetEncoders() {
        motor.getEncoder().setPosition(0);
    }

  public boolean gamepieceDetected() {
    // double measurement = lasercan.getMeasurement().distance_mm;
    // return measurement < 20;
    return 4 < 20;
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
        // SmartDashboard.putBoolean(
        // "Indexer Game Piece Detected",
        // gamepieceDetected()
        // );
    }
}
