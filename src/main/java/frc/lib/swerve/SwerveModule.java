package frc.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {
  public int moduleNumber;

  private Rotation2d angleOffset;

  // private final MOTORTYPE m_angleMotor;
  // private final MOTORTYPE m_driveMotor;
  // private final CANCODERTYPE m_angleEncoder;

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;
    // this.m_angleMotor = new MOTORTYPE(moduleConstants.angleMotorID);
    // this.m_driveMotor = new MOTORTYPE(moduleConstants.angleMotorID);
    // this.m_angleEncoder = new CANCODERTYPE(moduleConstants.canCoderID);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      // Conversions.rotationsToMeters(
      //   m_driveMotor.getVelocity().getValue(),
      //   Constants.Swerve.wheelCircumference
      // ),
      // Rotation2d.fromRotations(m_angleMotor.getPosition().getValue())
    );
  }
}
