package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class Align2 extends Command {

    private static final double BOT_RADIUS = mag(0.84, 0.98) / 2;
    private static final double PEDRO_GO_UP = 2.5;
    private static final double RED_BLUE_OFFSET = 8.569576;

    private RobotContainer rob;
    private Swerve m_swerve;
    private AlignPIDController pidController;
    private Pose2d target;
    private boolean atSetpoint;

    // RED CORAL
    private final Pose2d[] redCoralPoses = {
        // 6
        new Pose2d(
            13.427346264776789,
            2.515705824540677,
            Rotation2d.fromDegrees(98.88435146986305)
        ),
        new Pose2d(
            14.19910430572695,
            3.0641515335854375,
            Rotation2d.fromDegrees(149.81445534558955)
        ),
        // 7
        new Pose2d(
            14.474698233432257,
            3.511730009749397,
            Rotation2d.fromDegrees(149.11166780153667)
        ),
        new Pose2d(
            14.510433090775525,
            4.47864233717442,
            Rotation2d.fromDegrees(-155.88179827623432)
        ),
        // 8
        new Pose2d(
            14.21768773408878,
            5.035340508559569,
            Rotation2d.fromDegrees(-146.35153267274092)
        ),
        new Pose2d(
            13.407790682074818,
            5.488221076776056,
            Rotation2d.fromDegrees(-96.29061063053797)
        ),
        // 9
        new Pose2d(
            12.741693919859538,
            5.527083506044402,
            Rotation2d.fromDegrees(-82.5972415409531)
        ),
        new Pose2d(
            12.009193989444643,
            5.107374161108507,
            Rotation2d.fromDegrees(-40.15473991942403)
        ),
        // 10
        new Pose2d(
            11.633289217372596,
            4.562910020151264,
            Rotation2d.fromDegrees(-29.400396067552148)
        ),
        new Pose2d(
            11.634745884926582,
            3.552937500751069,
            Rotation2d.fromDegrees(28.17598558596463)
        ),
        // 11
        new Pose2d(
            11.93383383675274,
            3.032111893217508,
            Rotation2d.fromDegrees(35.029174811110195)
        ),
        new Pose2d(
            12.737658259261673,
            2.567307667187257,
            Rotation2d.fromDegrees(84.29466186011739)
        ),
    };

    // BLUE CORAL
    private final Pose2d[] blueCoralPoses = {
        // 19
        new Pose2d(
            4.224899579823936,
            5.5344501521126634,
            Rotation2d.fromDegrees(-88.4521317697861)
        ),
        new Pose2d(
            3.3666678936356473,
            5.007216787737023,
            Rotation2d.fromDegrees(-31.01720028633875)
        ),
        // 18
        new Pose2d(
            3.053954412530211,
            4.565141119232801,
            Rotation2d.fromDegrees(-29.173936504496957)
        ),
        new Pose2d(
            3.071654694238828,
            3.5378292020069964,
            Rotation2d.fromDegrees(29.78052470205579)
        ),
        // 17
        new Pose2d(
            3.3052187861031546,
            3.05038747187902,
            Rotation2d.fromDegrees(30.625131794523984)
        ),
        new Pose2d(
            4.19345985524054,
            2.5656987211536366,
            Rotation2d.fromDegrees(88.71774517314905)
        ),
        // 22
        new Pose2d(
            4.737597160312245,
            2.5121039548371664,
            Rotation2d.fromDegrees(90.6691484645772)
        ),
        new Pose2d(
            5.609083255706641,
            3.0343413870803384,
            Rotation2d.fromDegrees(148.50942957515792)
        ),
        // 21
        new Pose2d(
            5.9278214797597615,
            3.4911714638763875,
            Rotation2d.fromDegrees(151.23565550662357)
        ),
        new Pose2d(
            5.905854932649149,
            4.512771462417399,
            Rotation2d.fromDegrees(-151.1744710358385)
        ),
        // 20
        new Pose2d(
            5.66315743098829,
            5.002576394543338,
            Rotation2d.fromDegrees(-149.00439104983965)
        ),
        new Pose2d(
            4.777538613599835,
            5.499334065613224,
            Rotation2d.fromDegrees(-91.60050909753255)
        ),
    };

    // RED ALGAE
    private final Pose2d[] redAlgaePoses = {
        // 6
        new Pose2d(
            13.9058840710117,
            2.589065008965157,
            Rotation2d.fromDegrees(120)
        ),
        // 7
        new Pose2d(
            14.976717628500175,
            4.016416363281362,
            Rotation2d.fromDegrees(180)
        ),
        // 8
        new Pose2d(
            13.855123749362358,
            5.4322932277611455,
            Rotation2d.fromDegrees(-120)
        ),
        // 9
        new Pose2d(
            12.215237734878354,
            5.41132468625061,
            Rotation2d.fromDegrees(-60)
        ),
        // 10
        new Pose2d(
            11.37581442204965,
            3.990139793686492,
            Rotation2d.fromDegrees(0)
        ),
        // 11
        new Pose2d(
            12.267215685543233,
            2.5672275681612033,
            Rotation2d.fromDegrees(60)
        ),
    };

    // BLUE ALGAE
    private final Pose2d[] blueAlgaePoses = new Pose2d[redAlgaePoses.length];

    // RED REEF CORNERS
    private final Pose2d[] redReefCorners = {
        new Pose2d(13.046, 3.059, new Rotation2d()),
        new Pose2d(13.903, 3.546, new Rotation2d()),
        new Pose2d(13.903, 4.483, new Rotation2d()),
        new Pose2d(13.061, 4.984, new Rotation2d()),
        new Pose2d(12.218, 4.497, new Rotation2d()),
        new Pose2d(12.218, 3.546, new Rotation2d()),
        new Pose2d(13.046, 3.059, new Rotation2d()),
    };
    // BLUE REEF CORNERS
    private final Pose2d[] blueReefCorners = {
        new Pose2d(4.489, 3.059, new Rotation2d()),
        new Pose2d(5.325, 3.546, new Rotation2d()),
        new Pose2d(5.332, 4.483, new Rotation2d()),
        new Pose2d(4.489, 4.984, new Rotation2d()),
        new Pose2d(3.632, 4.497, new Rotation2d()),
        new Pose2d(3.632, 3.546, new Rotation2d()),
        new Pose2d(4.489, 3.059, new Rotation2d()),
    };

    private boolean canAlign;
    private Up upCommand;
    private int postIndex;

    public Align2(
        RobotContainer rob,
        Swerve swerve,
        Elevator elevator,
        Wrist wrist,
        Claw claw,
        CommandXboxController driveController,
        int postIndex
    ) {
        this.postIndex = postIndex;
        this.upCommand = new Up(
            rob,
            elevator,
            wrist,
            claw,
            swerve,
            driveController,
            () -> atSetpoint
        );
        this.rob = rob;
        this.m_swerve = swerve;
        pidController = new AlignPIDController(swerve, 0.2, 3);

        for (int i = 0; i < blueAlgaePoses.length; i++) {
            Pose2d pose = redAlgaePoses[i];

            blueAlgaePoses[i] = new Pose2d(
                pose.getX() - RED_BLUE_OFFSET,
                pose.getY(),
                pose.getRotation()
            );
        }

        addRequirements(swerve);
    }

    public static double mag(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    @Override
    public void initialize() {
        if (!m_swerve.gyroZeroed) return;

        upCommand.initialize();
        canAlign = false;
        atSetpoint = false;

        Pose2d[] poses = m_swerve.isRed()
            ? rob.isCoral ? redCoralPoses : redAlgaePoses
            : rob.isCoral ? blueCoralPoses : blueAlgaePoses;

        // TODO: target index based on button
        target = poses[postIndex];

        Pose2d[] corners = m_swerve.isRed() ? redReefCorners : blueReefCorners;

        Pose2d botPose = m_swerve.getPose();
        double botX = botPose.getX();
        double botY = botPose.getY();

        double m_b = (target.getY() - botY) / (target.getX() - botX);

        double theta = Math.atan(-1 / m_b);
        double shiftX = BOT_RADIUS * Math.cos(theta);
        double shiftY = BOT_RADIUS * Math.sin(theta);

        double botX1 = botX - shiftX;
        double botY1 = botY - shiftY;
        double botX2 = botX + shiftX;
        double botY2 = botY + shiftY;

        for (int i = 0; i < corners.length - 1; i++) {
            Pose2d r1 = corners[i];
            Pose2d r2 = corners[i + 1];

            double _x_r1 = r1.getX();
            double y_r1 = r1.getY();
            double _x_r2 = r2.getX();
            double y_r2 = r2.getY();

            double x_r1 = Math.min(_x_r1, _x_r2);
            double x_r2 = Math.max(_x_r1, _x_r2);

            double m_r = (y_r2 - y_r1) / (x_r2 - x_r1);

            if (m_b != m_r) {
                double x1 =
                    (m_r * x_r1 - y_r1 - m_b * botX1 + botY1) / (m_b - m_r);
                double x2 =
                    (m_r * x_r1 - y_r1 - m_b * botX2 + botY2) / (m_b - m_r);

                if ((x_r1 <= x1 && x1 <= x_r2) || (x_r1 <= x2 && x2 <= x_r2)) {
                    canAlign = false;
                    break;
                }
            }
        }
    }

    @Override
    public void execute() {
        if (
            canAlign &&
            mag(
                pidController.getXError(target),
                pidController.getYError(target)
            ) <
            PEDRO_GO_UP
        ) {
            upCommand.execute();
        }

        if (
            canAlign &&
            !atSetpoint &&
            Math.abs(pidController.getXError(target)) < 0.01 &&
            Math.abs(pidController.getYError(target)) < 0.01 &&
            Math.abs(pidController.getAngleError(target)) < 0.5
        ) {
            atSetpoint = true;
            m_swerve.stopSwerve();
        }

        double maxAlign = postIndex != -1 ? 3 : 2;

        if (
            !atSetpoint &&
            canAlign &&
            Math.abs(pidController.getXError(target)) < maxAlign &&
            Math.abs(pidController.getYError(target)) < maxAlign
        ) {
            pidController.alignLimelight(target);
        }

        SmartDashboard.putNumber("Cur Target X", target.getX());
        SmartDashboard.putNumber("Cur Target Y", target.getY());
        SmartDashboard.putNumber(
            "Cur Target R",
            target.getRotation().getDegrees()
        );
    }

    @Override
    public boolean isFinished() {
        return atSetpoint;
    }
}
