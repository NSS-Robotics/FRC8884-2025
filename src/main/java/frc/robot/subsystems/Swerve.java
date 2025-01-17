// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.lib.swerve.SwerveModule;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;

public class Swerve extends SubsystemBase {
  private Canandgyro gyro; // The gyroscope for getting... wait what is it actually for?
  private SwerveModule[] m_swerveModules;
  

  /** Creates a new Swerve. If you're doing that more than once, something is
   * horribly wrong. */
  public Swerve() {
    gyro = new Canandgyro(Constants.Swerve.GyroCanID);
    gyro.resetFactoryDefaults(0.35);
    gyro.setYaw(0);

    CanandEventLoop.getInstance();

    m_swerveModules = new SwerveModule[] {
      new SwerveModule(0, Constants.Swerve.Module0),
      new SwerveModule(1, Constants.Swerve.Module1),
      new SwerveModule(2, Constants.Swerve.Module2),
      new SwerveModule(3, Constants.Swerve.Module3),
    };
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