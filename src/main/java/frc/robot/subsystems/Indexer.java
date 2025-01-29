package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;

  public Indexer() {
    motor = new SparkMax(Constants.IndexerConstants.motorID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    motorConfig.closedLoop
      .p(Constants.IndexerConstants.kP)
      .i(Constants.IndexerConstants.kI)
      .d(Constants.IndexerConstants.kD)
      .velocityFeedForward(Constants.IndexerConstants.kFF);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setIndexer(double velocity) {
    motor.getClosedLoopController().setReference(velocity, ControlType.kVelocity);
  }

  public void resetEncoders() {
    motor.getEncoder().setPosition(0);
  }
}
