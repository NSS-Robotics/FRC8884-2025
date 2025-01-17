package frc.lib.swerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.Conversions;
import frc.robot.Constants;

public class SwerveModule {
  public int moduleNumber;

  private Rotation2d angleOffset;

  // TalonFX is the controller class for Kraken motors.
  private final TalonFX m_angleMotor;
  private final TalonFX m_driveMotor;
  private final CANcoder m_angleEncoder;

  private final PositionVoltage anglePosition = new PositionVoltage(0);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;
    this.m_angleMotor = new TalonFX(moduleConstants.angleMotorID);
    this.m_driveMotor = new TalonFX(moduleConstants.angleMotorID);
    this.m_angleEncoder = new CANcoder(moduleConstants.canCoderID);
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

  public void setState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    m_angleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      Conversions.rpsToMPS(
        m_driveMotor.getVelocity().getValueAsDouble(),
        Constants.Swerve.wheelCircumference
      ),
      Rotation2d.fromRotations(m_angleMotor.getPosition().getValueAsDouble())
    );
  }
}
