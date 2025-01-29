package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;
  private final LaserCan lasercan;

  public Indexer() {
    motor = new SparkMax(Constants.IndexerConstants.motorID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    lasercan = new LaserCan(Constants.IndexerConstants.laserCANID);

    motorConfig.closedLoop
      .p(Constants.IndexerConstants.kP)
      .i(Constants.IndexerConstants.kI)
      .d(Constants.IndexerConstants.kD)
      .velocityFF(Constants.IndexerConstants.kFF);

    motorConfig.smartCurrentLimit(40);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    try {
      lasercan.setRangingMode(LaserCan.RangingMode.SHORT);
      lasercan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lasercan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  public void setIndexer(double velocity) {
    motor.getClosedLoopController().setReference(velocity, ControlType.kVelocity);
  }

  public void resetEncoders() {
    motor.getEncoder().setPosition(0);
  }

  public boolean coralDetected() {
    double measurement = lasercan.getMeasurement().distance_mm;
    return measurement <= 20;
  }
}
