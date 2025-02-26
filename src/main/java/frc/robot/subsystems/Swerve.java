// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;
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
import frc.robot.RobotContainer;
import java.util.Optional;

public class Swerve extends SubsystemBase {

    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private Canandgyro gyro;

    private Limelight l_limelightlow;
    private Limelight l_limelighthigh;
    private RobotContainer robotContainer;

    // WPILib
    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
        .getStructTopic("MyPose", Pose2d.struct)
        .publish();

    public Swerve(
        Limelight limelightlow,
        Limelight limelighthigh,
        RobotContainer robotContainer
    ) {
        gyro = new Canandgyro(Constants.Swerve.gyroID);
        gyro.resetFactoryDefaults(0.35);
        gyro.setYaw(0);
        CanandEventLoop.getInstance();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants),
        };

        swerveOdometry = createOdometry(new Pose2d(0, 0, new Rotation2d()));
        swerveOdometry.update(gyro.getRotation2d(), getModulePositions());
        // driveInvert = (isRed() ? 1 : -1);
        l_limelightlow = limelightlow;
        l_limelighthigh = limelighthigh;
        this.robotContainer = robotContainer;
    }

    public void autoDrive(ChassisSpeeds speed) {
        SwerveModuleState[] moduleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(speed);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 4);
        setModuleStates(moduleStates);
    }

    public void driveFromSpeeds(ChassisSpeeds speeds, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
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
                        translation.getX(),
                        translation.getY(),
                        rotation,
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

    public void drivee(
        Translation2d translation,
        double rotation,
        boolean fieldRelative,
        boolean isOpenLoop
    ) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        -translation.getX(),
                        -translation.getY(),
                        -rotation,
                        gyro.getRotation2d()
                    )
                    : new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation
                    )
            );
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates,
            Constants.Swerve.maxSpeed
        );

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(
                swerveModuleStates[mod.moduleNumber],
                isOpenLoop
            );
        }
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(
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
        swerveOdometry.resetPosition(
            gyro.getRotation2d(),
            getModulePositions(),
            pose
        );
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
        return Rotation2d.fromDegrees(
            isRed() ? yaw : yaw > 0 ? yaw - 180 : yaw + 180
        );
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

    public void turnStates(double angularSpeed, double x, double y) {
        var swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
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
            Constants.Swerve.swerveKinematics,
            gyro.getRotation2d(),
            getModulePositions(),
            pose
        );
    }

    @Override
    public void periodic() {
        Limelight limelight = null;

        boolean htv = l_limelighthigh.tv > 0;
        boolean ltv = l_limelightlow.tv > 0;

        if (htv && ltv) {
            limelight = l_limelighthigh.ta > l_limelightlow.ta
                ? l_limelighthigh
                : l_limelightlow;
        } else if (htv) {
            limelight = l_limelighthigh;
        } else if (ltv) {
            limelight = l_limelightlow;
        }

        if (limelight != null) {
            Pose2d pose = new Pose2d(
                limelight.botPose.getX(),
                limelight.botPose.getY(),
                getGyroYaw()
            );
            swerveOdometry = createOdometry(pose);
        } else {
            swerveOdometry.update(getGyroYaw(), getModulePositions());
        }

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber(
                "Mod " + mod.moduleNumber + " CANcoder",
                mod.getCANcoder().getDegrees()
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
        SmartDashboard.putNumber("Pos R", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Yaw", gyro.getYaw() * 360);
        SmartDashboard.putNumber(
            "gyro getrotation2d",
            gyro.getRotation2d().getDegrees()
        );
        // publisher.set(m_pose);
    }
}
