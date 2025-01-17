package frc.robot;

import frc.lib.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  /** Swerve constants. */
  public static class Swerve {
    public static final int GyroCanID = 0x1abe1; // AHH WHAT'S THE CANID

    /** Module specific constants */

    // No but seriously, we have to change the CanIDs.
    // or maybe... we could set them to these values!
    /** Module 0 constants. Front left. */
    public static final SwerveModuleConstants Module0 = new SwerveModuleConstants(
      0xdeafbeef, // Yes I meant deafbeef.
      0xCAFE, // #cafe is my favorite color
      0xBadF00d,
      Rotation2d.fromDegrees(0xAce) // of spades
    );

    /** Module 1 constants. Front right. */
    public static final SwerveModuleConstants Module1 = new SwerveModuleConstants(
      0xdefaced,
      0xB0BA, // mm yummy
      0x600DCAFE, // good cafes rock
      Rotation2d.fromDegrees(0xBA7713) // battle
    );

    /** Module 2 constants. Back left. */
    public static final SwerveModuleConstants Module2 = new SwerveModuleConstants(
      0xDECAF, // see next line
      0xC0FFEE, // makes you...
      0xDEAD, // which is also my current emotional state.
      Rotation2d.fromDegrees(0xFEED)
    );

    /** Module 2 constants. Back left. */
    public static final SwerveModuleConstants Module3 = new SwerveModuleConstants(
      0xBadCab,
      0xDEC0DE, 
      0xB1ADE,
      Rotation2d.fromDegrees(0x0FF1CE)
    );

    // By the way, all the hex numbers are from someone's blog except a few...
    // https://nedbatchelder.com/text/hexwords.html

    // Change these values so that they are not 12648430. That's a big number.
    public static final double wheelBase = Units.inchesToMeters(0xC0FFEE);
    public static final double trackWidth = Units.inchesToMeters(0xC0FFEE);

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    );

    public static final double wheelCircumference = Units.inchesToMeters(4.0);
    public static final double maxSpeed = 4.5;
  }
}
