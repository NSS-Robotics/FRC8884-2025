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
 * Luke, I am your father
 * hwllo from the bombynation
 * ;>s
 */
public final class Constants {

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
        processor,
        climb,
    }

    public static class OperatorConstants {

        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
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
        public static final double currentLimit = 40;
        public static final double upKP = 63;
        public static final double upKI = 0;
        public static final double upKD = 5;
        public static final double downKP = 37;
        public static final double downKI = 0;
        public static final double downKD = 4;
        public static final double algaekP = 16; // testing
        public static final double algaekI = 0;
        public static final double algaekD = 0.020493;
        public static final int upSlot = 0;
        public static final int downSlot = 1;
        public static final int algaeSlot = 2;
        public static final double upThreshold = 0.1;
        public static final double wristDownSafeThreshold = 0.3;
        public static final double posTolerance = 0.05;
        public static final double[] pos = {
            0.5, // l1
            1.82, // l2
            2.8, // l3
            4.28, // l4
            // 1.204619140625, // stationIntake
            1.1, // stationIntake
            0, // handoff
            4.1, // barge
            0.2, // algaeGround
            1.82, // algaeReefLow
            2.8, // algaeReefHigh
            0.66, // processor
            0, // climb
        };

        public static final double maxRotations = 4.28;
    }

    public static final class WristConstants {

        public static final int encoder = 42;
        public static final int motorID = 40;
        public static final double magnetSensorOffset = -0.413818359375; // This should be undefbeefed
        public static final double kP = 20;
        public static final double kI = 0;
        public static final double kD = 0.3;
        // public static final double handoffPos = 0.02976171875;
        public static final double[] pos = {
            0.52, // l1
            0.63, // l2
            0.63, // l3
            0.63, // l4
            0.455810546875, // stationIntake
            0.0306484375, // handoff
            0.39, // barge
            0.622, // algaeGround
            0.626, // algaeReefLow
            0.626, // algaeReefHigh
            0.622, // processor
            0.270751953125, // climb
        };
        public static final double posTolerance = 0.05;
        public static final double maxRotations = 0.72;
        public static final double minElevatorRaisedPos = 0.323;
        public static final double maxElevatorLoweredPos = 0.64;
        public static final double minOuttakePos = 0.45;
        public static final double pedroDoIt = 20;
    }

    public static final class ClimberConstants {

        public static final int lMotorID = 20;
        public static final int rMotorID = 21;
        public static final int lChannel = 7;
        public static final int rChannel = 8;
        public static final double upkP = 2.1952;
        public static final double upkI = 0;
        public static final double upkD = 0.020493;
        // public static final double downkP = 3.781;
        public static final double downkP = 57; // testing
        public static final double downkI = 0;
        public static final double downkD = 0.020493;
        // public static final double downkD = 0.020493;
        public static final double maxRotations = 98;
        //55.9296875
        public static final double restingRot = 50.9296875;
        public static final double climbRot = 95.5;
        public static final double minRot = 0;
        public static final double halfIntakeRot = 19.7880859375;
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
        public static final double velocity = 0.7;
        public static final double halfIntakeVelocity = 0.3;
    }

    public static final class IntakeConstants {

        public static final int pivotMotorID = 60;
        public static final int intakeMotorID = 61;
        public static final int encoderID = 62;
        public static final double encoderOffset = 0.213623046875;
        public static final double upKP = 12.75;
        public static final double upKI = 0;
        public static final double upKD = 2;
        public static final double downKP = 8.25;
        public static final double downKI = 0;
        public static final double downKD = 0;
        public static final double maxRot = 0;
        public static final double intakeKP = 0.053;
        public static final double intakeKI = 0;
        public static final double intakeKD = 0;
        public static final double intakeKS = 0.23712;
        public static final double intakeKV = 0.12479;
        public static final double intakeKA = 0.0070548;
        public static final double pivotMaxRotations = -0.4;
        public static final double intakeStartPos = -0.3;
        public static final double algaePosition = 0;
        public static final double upPosition = 0;
        // public static final double intakePosition = -0.3583984375;
        public static final double intakePosition = -0.348388671875;
        public static final double climbPosition = intakePosition / 5.0;
        public static final double autoIntakePosition = -0.348388671875;
        public static final double autoIntakeUpPosition = -0.061279296875;
        // public static final double velocity = 2000;
        public static final double velocity = 3000;
        public static final double stationVelocity = 300;
        public static final double halfIntakeVelocity = 3000;
        public static final double l1velocity = 1000;
    }

    public static final class EndEffectorConstants {

        // public static final double kP = 8;
        public static final double kP = 4;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 1.0274;
        public static final double kV = 1.16686;
        public static final double kA = 0.119518;
        public static final double currentLimit = 55; // That's a very small current limit
        public static final int motorID = 41;
        public static final int laserCANID = 43;
        public static final double climbPosition = 0.233;
        public static final double intakePosition = 0.49;
        public static final double elevatorUpMaxValue = 0.18;
        public static final double elevatorUpMinValue = -0.12;
        public static final double velocity = 11000;
        public static final double autoOuttakeVelocity = 1250;
        public static final double teleopOuttakeVelocity = 750;
        public static final double l1Velocity = 1100;
        public static final double holdingVelocity = 3500;
    }

    public static final class LEDConstants {

        public static final int length = 154;
        public static final int channel = 9;
    }
}
