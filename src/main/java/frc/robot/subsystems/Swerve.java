package frc.robot.subsystems;

import frc.lib.swerve.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;

public class Swerve extends SubsystemBase {
  private Canandgyro gyro; // The gyroscope for getting... wait what is it actually for?
  private SwerveModule[] m_swerveModules;
  private Pose2d m_pose;
  private SwerveDriveOdometry odometry;

  /**
   * Creates a new Swerve. If you're doing that more than once, something is
   * horribly wrong.
   */
  public Swerve() {
    // Set up the gyroscope.
    gyro = new Canandgyro(Constants.Swerve.GyroCanID);
    gyro.resetFactoryDefaults(0.35);
    gyro.setYaw(0);

    // Start the gyroscope because it would be bad not to.
    CanandEventLoop.getInstance();

    // The swerve-y stuff.
    m_swerveModules = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Module0), // Front left.
        new SwerveModule(1, Constants.Swerve.Module1), // Front right.
        new SwerveModule(2, Constants.Swerve.Module2), // Back left.
        new SwerveModule(3, Constants.Swerve.Module3), // Back right.
    };

    odometry = new SwerveDriveOdometry(
        Constants.Swerve.kinematics,
        getGyroYaw(),
        getModulePositions(),
        new Pose2d(0, 0, new Rotation2d()));
    m_pose = odometry.update(getGyroYaw(), getModulePositions());
  }

  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(360 - gyro.getYaw());
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4]; // Because we have 4 wheels.
    for (SwerveModule module : m_swerveModules) {
      positions[module.moduleNumber] = module.getPosition();
    }
    return positions;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule module : m_swerveModules) {
      module.setState(desiredStates[module.moduleNumber], false);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
