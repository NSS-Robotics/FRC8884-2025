package frc.robot;

import frc.lib.swerve.SwerveModuleConstants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
    public static final int GyroCanID = 13;

    public static final boolean invertGyro = true;

        /** (6.75 : 1) */
        public static final double L2 = (6.75 / 1.0);

        public static final double angleKP = 0.10;
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
        public static final double trackWidth = Units.inchesToMeters(20.5);
        public static final double wheelBase = Units.inchesToMeters(26.5);
        public static final double wheelCircumference =
            Units.inchesToMeters(4.0);
        public static final double driveBaseRadius =
            Math.sqrt(trackWidth * trackWidth + wheelBase * wheelBase) / 2;

        /* Module Gear Ratios */
        public static final double driveGearRatio = 5.9;
        public static final double angleGearRatio = 150/7;

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

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode =
            NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode =
            NeutralModeValue.Brake;
    /** Module specific constants */

    // No but seriously, we have to change the CanIDs.
    // or maybe... we could set them to these values!
    /** Module 0 constants. Front left. */
    public static final SwerveModuleConstants Module0 = new SwerveModuleConstants(
        1,
        2,
        3,
        Rotation2d.fromDegrees(37.265625)
    );

    /** Module 1 constants. Front right. */
    public static final SwerveModuleConstants Module1 = new SwerveModuleConstants(
        4,
        5,
        6,
        Rotation2d.fromDegrees(37.265625)
    );

    /** Module 2 constants. Back left. */
    public static final SwerveModuleConstants Module2 = new SwerveModuleConstants(
        10,
        11,
        12,
        Rotation2d.fromDegrees(-85.95703125));

    /** Module 3 constants. Back right. */
    public static final SwerveModuleConstants Module3 = new SwerveModuleConstants(
        7,
        8,
        9,
        Rotation2d.fromDegrees(-11.513671875));

    // By the way, all the hex numbers are from someone's blog except a few...
    // https://nedbatchelder.com/text/hexwords.html

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    public static final double maxSpeed = 4.5;
    public static final double maxAngularVelocity = 10.0;
  }

  public static class ElevatorConstants {
    public static final int motorID = 30;
    public static final int encoder = 31;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double maxRotations = 0;
  }

  public static final class PivotConstants {
    public static final int encoder = 42;
    public static final int motorID = 40;
    public static final double magnetSensorOffset = 0xdeafbeef; // This should be undefbeefed
    public static final double kP = 0xBadF00d; // Why so much bad food
    public static final double kI = 0xBadF00d; // Why so much bad food
    public static final double kD = 0xBadF00d; // Why so much bad food
    public static final double maxRotations = 0xdeafbeef; // This should also be undefbeefed
  }

  public static final class ClimberConstants {
    public static final int lMotorID = 20;
    public static final int rMotorID = 21;
    public static final int lChannel = 9;
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
    public static final double kP = 0;
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
    public static final double kP = 0x1abe1;
    public static final double kI = 0xACE;
    public static final double kD = 0xDEADBEEF;
    public static final double currentLimit = 4.123456789; // That's a very small current limit
    public static final int motorID = 41;
    public static final int laserCANID = 43;

  }
}
