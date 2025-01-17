package frc.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int canCoderID;
  public final Rotation2d angleOffset;

  public SwerveModuleConstants(
    int driveMotorID,
    int angleMotorID,
    int canCoderID,
    Rotation2d angleOffset
  ) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.canCoderID = canCoderID;
    this.angleOffset = angleOffset;
  }
}
