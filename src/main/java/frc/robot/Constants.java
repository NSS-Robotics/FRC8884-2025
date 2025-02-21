package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

    public enum RobotState {
        l1,
        l2,
        l3,
        l4,
        stationIntake,
        handoff,
        barge,
        algaeGround,
        algaeReefLow,
        algaeReefHigh,
        climb,
    }

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
        public static final double currentLimit = 30;
        public static final double upKP = 27;
        public static final double upKI = 0;
        public static final double upKD = 0;
        public static final double downKP = 20;
        public static final double downKI = 0;
        public static final double downKD = 0;
        public static final int upSlot = 0;
        public static final int downSlot = 1;
        public static final double stationPos = 0.984619140625;
        public static final double[] pos = {
            0, // l1
            1.82, // l2
            2.8, // l3
            4.28, // l4
            0.984619140625, // stationIntake
            0, // handoff
            4.28, // barge
            0, // algaeGround
            1.82, // algaeReefLow
            2.8, // algaeReefHigh
            0, // climb
        };

        public static final double maxRotations = 4.3;
    }

    public static final class WristConstants {

        public static final int encoder = 42;
        public static final int motorID = 40;
        public static final double magnetSensorOffset = -0.413818359375; // This should be undefbeefed
        public static final double kP = 16; // Why so much bad food
        public static final double kI = 0; // Why so much bad food
        public static final double kD = 0; // Why so much bad food
        // public static final double handoffPos = 0.02976171875;
        public static final double[] pos = {
            0.5, // l1
            0.626, // l2
            0.626, // l3
            0.6625, // l4
            0.455810546875, // stationIntake
            0.03576171875, // handoff
            0.4111328125, // barge
            0.61, // algaeGround
            0.626, // algaeReefLow
            0.626, // algaeReefHigh
            0.4, // climb
        };
        public static final double handoffPos = 0.03576171875;
        public static final double algaeIntakePos = 0.61;
        public static final double stationPos = 0.455810546875;
        public static final double bargePos = 0.4111328125;

        public static final double l2Pos = 0.626;
        public static final double l4Pos = 0.6625;

        public static final double maxRotations = 0.72; // This should also be undefbeefed
        // 0.593505859375; // This should also be undefbeefed
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
        public static final double kP = 9.999E-04;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0.5;
        public static final double kV = 0.5;
        public static final double kA = 0.1;
        public static final double velocity = 0.5;
    }

    public static final class IntakeConstants {

        public static final int pivotMotorID = 60;
        public static final int intakeMotorID = 61;
        public static final int encoderID = 62;
        public static final double encoderOffset = -0.69384765625;
        public static final double uppivotKP = 20.946;
        public static final double uppivotKI = 0;
        public static final double uppivotKD = 1.1742;
        public static final double downpivotKP = 3;
        public static final double downpivotKI = 0;
        public static final double downpivotKD = 0;
        public static final double maxRot = 0;
        public static final double intakeKP = 0.053;
        public static final double intakeKI = 0;
        public static final double intakeKD = 0;
        public static final double intakeKS = 0.23712;
        public static final double intakeKV = 0.12479;
        public static final double intakeKA = 0.0070548;
        public static final double pivotMaxRotations = 0.90353515625;
        public static final double upPosition = 0.886484375;
        public static final double intakePosition = 0;
        public static final double velocity = 2000;
    }

    public static final class EndEffectorConstants {

        public static final double kP = 4;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 1.0274;
        public static final double kV = 1.16686;
        public static final double kA = 0.119518;
        public static final double currentLimit = 20; // That's a very small current limit
        public static final int motorID = 41;
        public static final int laserCANID = 43;
        public static final double climbPosition = 0.233;
        public static final double intakePosition = 0.49;
        public static final double elevatorUpMaxValue = 0.18;
        public static final double elevatorUpMinValue = -0.12;
        public static final double velocity = 5000;
        public static final double outtakeVelocity = 2000;
    }
}
