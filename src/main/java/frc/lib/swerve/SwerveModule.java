package frc.lib.swerve;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;

public class SwerveModule {
  public int moduleNumber;

  private Rotation2d angleOffset;

  // TalonFX is the controller class for Kraken motors.
  private final TalonFX m_angleMotor;
  private final TalonFX m_driveMotor;
  // private final CANCODERTYPE m_angleEncoder;

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;
    this.m_angleMotor = new TalonFX(moduleConstants.angleMotorID);
    this.m_driveMotor = new TalonFX(moduleConstants.angleMotorID);
    //this.m_angleEncoder = new CANCODERTYPE(moduleConstants.canCoderID);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      Conversions.rotationsToMeters(
         m_driveMotor.getVelocity().getValueAsDouble(),
         Constants.Swerve.wheelCircumference
      ),
      Rotation2d.fromRotations(m_angleMotor.getPosition().getValueAsDouble())
    );
  }
}
