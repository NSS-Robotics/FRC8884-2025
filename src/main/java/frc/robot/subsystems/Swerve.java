// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.lib.swerve.SwerveModule;

import java.util.Optional;

import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;

public class Swerve extends SubsystemBase {
    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private Canandgyro gyro;

    private Pose2d m_pose;
    private Limelight l_limelight;
    public boolean wtfIsRunning = false;

    // WPILib
    StructPublisher<Pose2d> publisher = NetworkTableInstance
        .getDefault()
        .getStructTopic("MyPose", Pose2d.struct)
        .publish();

    public Swerve(Limelight limelight) {
        gyro = new Canandgyro(14);
        gyro.resetFactoryDefaults(0.35);
        gyro.setYaw(0);
        CanandEventLoop.getInstance();

        mSwerveMods =
            new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Module0),
                new SwerveModule(1, Constants.Swerve.Module1),
                new SwerveModule(2, Constants.Swerve.Module2),
                new SwerveModule(3, Constants.Swerve.Module3),
            };

        swerveOdometry = createOdometry(new Pose2d(0, 0, new Rotation2d()));
        m_pose = swerveOdometry.update(gyro.getRotation2d(), getModulePositions());
        // driveInvert = (isRed() ? 1 : -1);
        l_limelight = limelight;
    }

    public void autoDrive(ChassisSpeeds speed) {
        SwerveModuleState[] moduleStates = Constants.Swerve.kinematics.toSwerveModuleStates(
            speed
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 4);
        setModuleStates(moduleStates);
    }

    public void driveFromSpeeds(ChassisSpeeds speeds, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.kinematics.toSwerveModuleStates(
            speeds
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates,
            Constants.Swerve.maxSpeed
        );
        SmartDashboard.putNumber("chassisspeedsvX", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("chassisspeedsvy", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber(
            "chassisspeedsomega",
            speeds.omegaRadiansPerSecond
        );
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(
                swerveModuleStates[mod.moduleNumber],
                isOpenLoop
            );
        }
    }

    public void drive(
        Translation2d translation,
        double rotation,
        boolean fieldRelative,
        boolean isOpenLoop
    ) {
        SmartDashboard.putNumber("translation getx", translation.getX());
        SmartDashboard.putNumber("translation gety", translation.getY());
        SmartDashboard.putNumber("rotation in drive", rotation);
        SmartDashboard.putBoolean("isopenloop", isOpenLoop);
        driveFromSpeeds(
            fieldRelative
                ? isRed()
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        gyro.getRotation2d()
                    )
                    : ChassisSpeeds.fromFieldRelativeSpeeds(
                        -translation.getX(),
                        -translation.getY(),
                        -rotation,
                        gyro.getRotation2d()
                    )
                : new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation
                ),
            isOpenLoop
        );
    }


    public void drivee(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    -translation.getX(), 
                                    -translation.getY(), 
                                    -rotation, 
                                    gyro.getRotation2d()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return Constants.Swerve.kinematics.toChassisSpeeds(
            getModuleStates()
        );
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates,
            Constants.Swerve.maxSpeed
        );

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void zeroGyro() {
        gyro.setYaw(0);
        setHeading(new Rotation2d(Units.degreesToRadians(isRed() ? 0 : 180)));
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(
            gyro.getRotation2d(),
            getModulePositions(),
            new Pose2d(getPose().getTranslation(), heading)
        );
    }

    public Rotation2d getGyroYaw() {
        double yaw = gyro.getYaw() * 360;
        return isRed()
            ? Rotation2d.fromDegrees(-yaw)
            : Rotation2d.fromDegrees(yaw >= 0 ? 180 - yaw : -180 - yaw);
    }

    public Pose2d getLimelightBotPose() {
        return m_pose;
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public boolean isRed() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    public double[] getSpeakerDistances() {
        Pose2d pose = getLimelightBotPose();

        double x =
            (isRed() ? Constants.redReefX : Constants.blueReefX) -
            pose.getX();
        double y = Constants.reefY - pose.getY();
        return new double[] { x, y };
    }

    public void turnStates(double angularSpeed, double x, double y) {
        var swerveModuleStates = Constants.Swerve.kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                x,
                y,
                angularSpeed,
                gyro.getRotation2d()
            )
        );
        setModuleStates(swerveModuleStates);
    }

    public SwerveDriveOdometry createOdometry(Pose2d pose) {
        return new SwerveDriveOdometry(
            Constants.Swerve.kinematics,
            gyro.getRotation2d(),
            getModulePositions(),
            pose
        );
    }

    @Override
    public void periodic() {
        // if (l_limelight.tv > 0) {
        // swerveOdometry = createOdometry(l_limelight.botPose);
        // m_pose = l_limelight.botPose;
        // } else {
        // m_pose = swerveOdometry.update(getGyroYaw(), getModulePositions());
        // }

        if (l_limelight.gettv() > 0) {
            swerveOdometry = createOdometry(l_limelight.botPose);
            m_pose = l_limelight.botPose;
        } else {
            m_pose = swerveOdometry.update(gyro.getRotation2d(), getModulePositions());
        }

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber(
                "Mod " + mod.moduleNumber + " CANcoder",
                mod.getCANCoder().getDegrees()
            );
            SmartDashboard.putNumber(
                "Mod " + mod.moduleNumber + " Angle",
                mod.getPosition().angle.getDegrees()
            );
            SmartDashboard.putNumber(
                "Mod " + mod.moduleNumber + " Velocity",
                mod.getState().speedMetersPerSecond
            );
        }

        SmartDashboard.putNumber("Pos X", getPose().getX());
        SmartDashboard.putNumber("Pos Y", getPose().getY());
        SmartDashboard.putNumber("Pos R", m_pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Yaw", gyro.getYaw() * 360);
        SmartDashboard.putNumber(
            "gyro getrotation2d",
            gyro.getRotation2d().getDegrees()
        );


        publisher.set(m_pose);
    }
}
