package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double redReefX = 100;
    public static final double blueReefX = 100;
    public static final double reefY = 4;

    public static class OperatorConstants {

        public static final int kDriverControllerPort = 0;
    }

    public static final double stickDeadband = 0.1;

    /** Swerve constants. */
    public static class Swerve {

        public static final int gyroID = 13;
        public static final boolean invertGyro = true;

        public static final COTSTalonFXSwerveConstants chosenModuleDRIVE =
            COTSTalonFXSwerveConstants.SDS.MK4n.KrakenX60(
                COTSTalonFXSwerveConstants.SDS.MK4n.driveRatios.L2_plus
            );
        public static final COTSTalonFXSwerveConstants chosenModuleTURN =
            COTSTalonFXSwerveConstants.SDS.MK4n.KrakenX60(
                COTSTalonFXSwerveConstants.SDS.MK4n.driveRatios.L2_plus
            );

        public static final double angleKP = 100;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;

        public static final InvertedValue driveMotorInvert =
            InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue angleMotorInvert =
            InvertedValue.Clockwise_Positive;
        public static final SensorDirectionValue cancoderInvert =
            SensorDirectionValue.CounterClockwise_Positive;

        public static final double stickDeadband = 0.1;

        /* Drivetrain Constants */
        public static final double trackWidth = 0.32;
        public static final double wheelBase = 0.32;
        public static final double wheelCircumference =
            chosenModuleDRIVE.wheelCircumference;
        public static final double driveBaseRadius =
            Math.sqrt(trackWidth * trackWidth + wheelBase * wheelBase) / 2;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
            );

        /* Module Gear Ratios */
        public static final double driveGearRatio =
            chosenModuleDRIVE.driveGearRatio;
        public static final double angleGearRatio =
            chosenModuleTURN.angleGearRatio;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32;
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode =
            NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode =
            NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {

            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(
                -142.3828125 + 180
            ); // was -148.5
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(
                    driveMotorID,
                    angleMotorID,
                    canCoderID,
                    angleOffset
                );
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {

            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(
                -82.08984375
            ); // was -111.55
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(
                    driveMotorID,
                    angleMotorID,
                    canCoderID,
                    angleOffset
                );
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {

            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(
                93.779296875 + 180
            ); // was 162.25
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(
                    driveMotorID,
                    angleMotorID,
                    canCoderID,
                    angleOffset
                );
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {

            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(
                -11.6015625
            ); // was -141.5
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(
                    driveMotorID,
                    angleMotorID,
                    canCoderID,
                    angleOffset
                );
        }
    }

    public static class ElevatorConstants {

        public static final int motorID = 30;
        public static final int encoder = 31;
        public static final double magnetSensorOffset = 0.599609375;
        public static final double intialkP = 1.5;
        public static final double intialkI = 0;
        public static final double intialkD = 0;

        public static final double middlekP = 9;
        public static final double middlekI = 0;
        public static final double middlekD = 0;

        public static final double pidOffset = 1.0 - 0.947998046875;

        public static final double maxRotations = 4.45;
    }

    public static final class PivotConstants {

        public static final int encoder = 42;
        public static final int motorID = 40;
        public static final double magnetSensorOffset = -0.413818359375; // This should be undefbeefed
        public static final double kP = 13.868; // Why so much bad food
        public static final double kI = 0; // Why so much bad food
        public static final double kD = 0; // Why so much bad food
        public static final double maxRotations = 0.593505859375; // This should also be undefbeefed
    }

    public static final class ClimberConstants {

        public static final int lMotorID = 20;
        public static final int rMotorID = 21;
        public static final int lChannel = 7;
        public static final int rChannel = 8;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double maxRotations = 0;
        public static final double minRot = 0;
    }

    public static final class IndexerConstants {

        public static final int motorID = 50;
        public static final int laserCANID = 51;
        public static final double kP = 0.17;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;
    }

    public static final class IntakeConstants {

        public static final int pivotMotorID = 60;
        public static final int intakeMotorID = 61;
        public static final int encoderID = 62;
        public static final double pivotKP = 0;
        public static final double pivotKI = 0;
        public static final double pivotKD = 0;
        public static final double maxRot = 0;
        public static final double intakeKP = 0;
        public static final double intakeKI = 0;
        public static final double intakeKD = 0;
        public static final double pivotMaxRotations = 0;
    }

    public static final class EndEffectorConstants {

        public static final double kP = 0.17;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0.3274;
        public static final double kV = 0.16686;
        public static final double kA = 0.019518;
        public static final double currentLimit = 4.123456789; // That's a very small current limit
        public static final int motorID = 41;
        public static final int laserCANID = 43;
        public static final double climbPosition = 0.233;
        public static final double intakePosition = 0.49;
        public static final double elevatorUpMaxValue = 0.18;
        public static final double elevatorUpMinValue = -0.12;
    }
}
