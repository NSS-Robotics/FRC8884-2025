package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Pivot extends SubsystemBase {
  private static TalonFX motor = new TalonFX(
    Constants.PivotConstants.motorID
  );

  private static TalonFXConfiguration motorConfig = new TalonFXConfiguration();
  private static Slot0Configs slot0Configs = motorConfig.Slot0;

  private static PositionVoltage pivotPositionVoltage;
  private CANcoder encoder = new CANcoder(
    Constants.PivotConstants.encoder
  );

  private Swerve m_swerve;
  private double yOffset;
  
  public Pivot(Swerve swerve) {
    yOffset = 0;

    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        0.5;
    canCoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfig.MagnetSensor.MagnetOffset = Constants.PivotConstants.magnetSensorOffset;
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

  public double getRotations() {
    return 0.146 * Math.pow(getDistance(), -0.586) + yOffset;
  }

  public double getEncoderPosition() {
    return encoder.getPosition().getValueAsDouble();
  }

  public double getEncoderVelocity() {
    return encoder.getPosition().getValueAsDouble();
  }
  
  public void printPivotData() {
    System.out.println("Distance to speaker: " + getDistance());
    System.out.println("Pivot position     : " + getEncoderPosition());
    System.out.println("Shoot position     : " + getRotations()); 
    System.out.println("Difference         : " + (getRotations() - getEncoderPosition()));
  }

  public void setyOffset(double y) {
    yOffset = y;
  }

  public double getYOffset() {
    return yOffset;
  }

  public void changeYOffset(double amount) {
    yOffset += amount;
  }

  private double getDistance(){
    double[] dist = m_swerve.getReefDistances();
    return Math.sqrt(dist[0] * dist[0] + dist[1] * dist[1]);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Y Offset", yOffset);
    SmartDashboard.putNumber("Distance to speaker", getDistance());
  }
}
