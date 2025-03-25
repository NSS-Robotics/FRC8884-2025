package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotState;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import java.util.Arrays;

public class Align2 extends Command {

    private static final double BOT_W = 0.84;
    private static final double BOT_L = 0.98;
    private static final double BOT_RADIUS = mag(BOT_W, BOT_L) / 2;

    private static final double PEDRO_GO_UP = 2.5;
    private static final double RED_BLUE_OFFSET = 8.569576;

    private RobotContainer rob;
    private Swerve m_swerve;
    private AlignPIDController pidController;
    private Pose2d target;
    private boolean atSetpoint;

    // RED CORAL LEVEL 1
    private final Pose2d[] redL1Poses = {
        // L1 - 7 Left
        new Pose2d(
            14.201058007081613,
            2.90157053344676,
            Rotation2d.fromDegrees(114.36672494755307)
        ),
        // L2 - 6 Right
        new Pose2d(
            14.560310462335117,
            3.558725472835589,
            Rotation2d.fromDegrees(-175.30412722281832)
        ),
        // L3 - 6 Left
        new Pose2d(
            12.648779945166266,
            2.4684989462587814,
            Rotation2d.fromDegrees(53.98024239366128)
        ),
        // L4 - 11 Right
        new Pose2d(
            13.37806864381898,
            2.484554728269225,
            Rotation2d.fromDegrees(124.14802881172679)
        ),
        // L5 - 11 Left
        new Pose2d(
            11.514464849624483,
            3.6065191063628173,
            Rotation2d.fromDegrees(-6.089001164643469)
        ),
        // L6 - 10 Right
        new Pose2d(
            11.894687198114651,
            2.978409464013095,
            Rotation2d.fromDegrees(64.56303823886117)
        ),
        // R1 - 7 Right
        new Pose2d(
            14.201058007081613,
            5.09842946655324,
            Rotation2d.fromDegrees(-114.36672494755307)
        ),
        // R2 - 8 Left
        new Pose2d(
            14.60273292271843,
            4.442631425979867,
            Rotation2d.fromDegrees(173.9508284788378)
        ),
        // R3 - 8 Right
        new Pose2d(
            12.720033253898677,
            5.551081323729148,
            Rotation2d.fromDegrees(-54.99991727968953)
        ),
        // R4 - 9 Left
        new Pose2d(
            13.468060384967364,
            5.567111252491732,
            Rotation2d.fromDegrees(-126.33236078083564)
        ),
        // R5 - 9 Right
        new Pose2d(
            11.570111821884012,
            4.505731222931736,
            Rotation2d.fromDegrees(4.692670352466536)
        ),
        // R6 - 10 Left
        new Pose2d(
            11.932886066650994,
            5.127923898600779,
            Rotation2d.fromDegrees(-66.48094346729839)
        ),
    };

    // BLUE CORAL LEVEL 1
    private final Pose2d[] blueL1Poses = {
        // L1 - 18 Left
        new Pose2d(
            3.370223398743634,
            5.185742035103555,
            Rotation2d.fromDegrees(-66.54292329621606)
        ),
        // L2 - 19 Right
        new Pose2d(
            3.0093958308432756,
            4.482416931920206,
            Rotation2d.fromDegrees(5.244653279838466)
        ),
        // L3 - 19 Left
        new Pose2d(
            4.916662567085468,
            5.600762810456995,
            Rotation2d.fromDegrees(-125.74868403149233)
        ),
        // L4 - 20 Right
        new Pose2d(
            4.136044861501736,
            5.5474082926978125,
            Rotation2d.fromDegrees(-54.62810044573254)
        ),
        // L5 - 20 Left
        new Pose2d(
            6.040638216749063,
            4.442352828754763,
            Rotation2d.fromDegrees(174.31919809409044)
        ),
        // L6 - 21 Right
        new Pose2d(
            5.624467998972,
            5.087008110547992,
            Rotation2d.fromDegrees(-114.26117886248036)
        ),
        // R1 - 18 Right
        new Pose2d(
            3.3556953727680328,
            2.979633590250977,
            Rotation2d.fromDegrees(65.34416226013789)
        ),
        // R2 - 17 Left
        new Pose2d(
            2.929293224307501,
            3.611890139904748,
            Rotation2d.fromDegrees(-5.784001316973641)
        ),
        // R3 - 17 Right
        new Pose2d(
            4.839822201325507,
            2.5085741431425754,
            Rotation2d.fromDegrees(125.12885038238431)
        ),
        // R4 - 22 Left
        new Pose2d(
            4.054235827487181,
            2.4628621954561694,
            Rotation2d.fromDegrees(53.77604300375448)
        ),
        // R5 - 22 Right
        new Pose2d(
            5.9700579196124615,
            3.569791861710402,
            Rotation2d.fromDegrees(-174.35485674381573)
        ),
        // R6 - 21 Left
        new Pose2d(
            5.628495347469564,
            2.875846396139371,
            Rotation2d.fromDegrees(113.84256649659072)
        ),
    };

    // RED CORAL
    private final Pose2d[] redCoralPoses = {
        // L1 - 7 Left
        new Pose2d(
            14.474698233432257,
            3.511730009749397,
            Rotation2d.fromDegrees(149.11166780153667)
        ),
        // L2 - 6 Right
        new Pose2d(
            14.19910430572695,
            3.0641515335854375,
            Rotation2d.fromDegrees(149.81445534558955)
        ),
        // L3 - 6 Left
        new Pose2d(
            13.427346264776789,
            2.515705824540677,
            Rotation2d.fromDegrees(98.88435146986305)
        ),
        // L4 - 11 Right
        new Pose2d(
            12.737658259261673,
            2.567307667187257,
            Rotation2d.fromDegrees(84.29466186011739)
        ),
        // L5 - 11 Left
        new Pose2d(
            11.93383383675274,
            3.032111893217508,
            Rotation2d.fromDegrees(35.029174811110195)
        ),
        // L6 - 10 Right
        new Pose2d(
            11.634745884926582,
            3.552937500751069,
            Rotation2d.fromDegrees(28.17598558596463)
        ),
        // R1 - 7 Right
        new Pose2d(
            14.510433090775525,
            4.47864233717442,
            Rotation2d.fromDegrees(-155.88179827623432)
        ),
        // R2 - 8 Left
        new Pose2d(
            14.21768773408878,
            5.035340508559569,
            Rotation2d.fromDegrees(-146.35153267274092)
        ),
        // R3 - 8 Right
        new Pose2d(
            13.407790682074818,
            5.488221076776056,
            Rotation2d.fromDegrees(-96.29061063053797)
        ),
        // R4 - 9 Left
        new Pose2d(
            12.741693919859538,
            5.527083506044402,
            Rotation2d.fromDegrees(-82.5972415409531)
        ),
        // R5 - 9 Right
        new Pose2d(
            12.009193989444643,
            5.107374161108507,
            Rotation2d.fromDegrees(-40.15473991942403)
        ),
        // R6 - 10 Left
        new Pose2d(
            11.633289217372596,
            4.562910020151264,
            Rotation2d.fromDegrees(-29.400396067552148)
        ),
    };

    // BLUE CORAL
    private final Pose2d[] blueCoralPoses = {
        // L1 - 18 Left
        new Pose2d(
            3.053954412530211,
            4.565141119232801,
            Rotation2d.fromDegrees(-29.173936504496957)
        ),
        // L2 - 19 Right
        new Pose2d(
            3.3666678936356473,
            5.007216787737023,
            Rotation2d.fromDegrees(-31.01720028633875)
        ),
        // L3 - 19 left
        new Pose2d(
            4.224899579823936,
            5.5344501521126634,
            Rotation2d.fromDegrees(-88.4521317697861)
        ),
        // L4 - 20 Right
        new Pose2d(
            4.777538613599835,
            5.499334065613224,
            Rotation2d.fromDegrees(-91.60050909753255)
        ),
        // L5 - 20 Left
        new Pose2d(
            5.66315743098829,
            5.002576394543338,
            Rotation2d.fromDegrees(-149.00439104983965)
        ),
        // L6 - 21 Right
        new Pose2d(
            5.905854932649149,
            4.512771462417399,
            Rotation2d.fromDegrees(-151.1744710358385)
        ),
        // R1 - 18 Right
        new Pose2d(
            3.071654694238828,
            3.5378292020069964,
            Rotation2d.fromDegrees(29.78052470205579)
        ),
        // R2 - 17 Left
        new Pose2d(
            3.3052187861031546,
            3.05038747187902,
            Rotation2d.fromDegrees(30.625131794523984)
        ),
        // R3 - 17 Right
        new Pose2d(
            4.19345985524054,
            2.5656987211536366,
            Rotation2d.fromDegrees(88.71774517314905)
        ),
        // R4 - 22 Left
        new Pose2d(
            4.737597160312245,
            2.5121039548371664,
            Rotation2d.fromDegrees(90.6691484645772)
        ),
        // R5 - 22 Right
        new Pose2d(
            5.609083255706641,
            3.0343413870803384,
            Rotation2d.fromDegrees(148.50942957515792)
        ),
        // R6 - 21 Left
        new Pose2d(
            5.9278214797597615,
            3.4911714638763875,
            Rotation2d.fromDegrees(151.23565550662357)
        ),
    };

    // RED ALGAE
    private final Pose2d[] rawRedAlgaePoses = {
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

    private final Pose2d[] redAlgaePoses = new Pose2d[12];

    // BLUE ALGAE
    private final Pose2d[] blueAlgaePoses = new Pose2d[12];

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

    public Align2(
        RobotContainer rob,
        Swerve swerve,
        Elevator elevator,
        Wrist wrist,
        Claw claw,
        CommandXboxController driveController
    ) {
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

        int[] indices = { 1, 0, 0, 5, 5, 4, 1, 2, 2, 3, 3, 4 };

        for (int i = 0; i < 12; i++) {
            redAlgaePoses[i] = rawRedAlgaePoses[indices[i]];
        }

        for (int i = 0; i < blueAlgaePoses.length; i++) {
            Pose2d pose = redAlgaePoses[i];

            blueAlgaePoses[blueAlgaePoses.length - i - 1] = new Pose2d(
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

    private static double[] getLRBounds(Pose2d center, double m_path) {
        double botTheta = center.getRotation().getRadians();
        double cornerTheta = Math.atan(BOT_W / BOT_L);

        double cornerTheta1 = botTheta - cornerTheta;
        double cornerTheta2 = botTheta + cornerTheta;

        cornerTheta = Math.abs(Math.tan(cornerTheta1) - m_path) >
            Math.abs(Math.tan(cornerTheta2) - m_path)
            ? cornerTheta1
            : cornerTheta2;

        double shiftX = BOT_RADIUS * Math.cos(cornerTheta);
        double shiftY = BOT_RADIUS * Math.sin(cornerTheta);

        return new double[] {
            center.getX() - shiftX,
            center.getY() - shiftY,
            center.getX() + shiftX,
            center.getY() + shiftY,
        };
    }

    private static boolean intersects(
        // line 1
        double m_1,
        double x_1,
        double y_1,
        // line 2
        double m_2,
        double x_2,
        double y_2,
        // domain of line 1: [a_1, b_1] or [b_1, a_2]
        double a_1,
        double b_1,
        // domain of line 2: [a_2, b_2] or [b_2, a_2]
        double a_2,
        double b_2
    ) {
        if (m_1 == m_2) return false;

        double x = (m_1 * x_1 - y_1 - m_2 * x_2 + y_2) / (m_1 - m_2);
        return (
            ((a_1 <= x && x <= b_1) || (b_1 <= x && x <= a_1)) &&
            ((a_2 <= x && x <= b_2) || (b_2 <= x && x <= a_2))
        );
    }

    private static void swap(double[] arr, int i, int j) {
        double temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
    }

    @Override
    public void initialize() {
        if (!m_swerve.gyroZeroed) return;

        upCommand.initialize();
        canAlign = true;
        atSetpoint = false;

        boolean isL1 = rob.scoringLevel.equals(RobotState.l1);

        Pose2d[] poses = m_swerve.isRed()
            ? rob.isCoral ? (isL1 ? redL1Poses : redCoralPoses) : redAlgaePoses
            : rob.isCoral
                ? (isL1 ? blueL1Poses : blueCoralPoses)
                : blueAlgaePoses;

        Pose2d[] corners = m_swerve.isRed() ? redReefCorners : blueReefCorners;

        target = poses[rob.postIndex];

        Pose2d botPose = m_swerve.getPose();

        double m_path =
            (botPose.getY() - target.getY()) / (botPose.getX() - target.getX());

        double[] botCorners = getLRBounds(botPose, m_path);
        double[] targetCorners = getLRBounds(target, m_path);

        double m_path1 =
            (targetCorners[1] - botCorners[1]) /
            (targetCorners[0] - botCorners[0]);
        double m_path2 =
            (targetCorners[3] - botCorners[3]) /
            (targetCorners[2] - botCorners[2]);

        if (
            intersects(
                // path line 1
                m_path1,
                botCorners[0],
                botCorners[1],
                // path line 2
                m_path2,
                botCorners[2],
                botCorners[3],
                // path line 1 domain
                botCorners[0],
                targetCorners[0],
                // path line 2 domain
                botCorners[2],
                targetCorners[2]
            )
        ) {
            swap(botCorners, 0, 2);
            swap(botCorners, 1, 3);

            m_path1 =
                (targetCorners[1] - botCorners[1]) /
                (targetCorners[0] - botCorners[0]);
            m_path2 =
                (targetCorners[3] - botCorners[3]) /
                (targetCorners[2] - botCorners[2]);
        }

        // check intersection for each reef side
        for (int i = 0; i < corners.length - 1; i++) {
            Pose2d r1 = corners[i];
            Pose2d r2 = corners[i + 1];

            double x_r1 = r1.getX();
            double y_r1 = r1.getY();
            double x_r2 = r2.getX();
            double y_r2 = r2.getY();

            double m_r = (y_r2 - y_r1) / (x_r2 - x_r1);

            if (
                intersects(
                    // reef side line
                    m_r,
                    x_r1,
                    y_r1,
                    // path line 1
                    m_path1,
                    botCorners[0],
                    botCorners[1],
                    // reef side line domain
                    x_r1,
                    x_r2,
                    // path line 1 domain
                    botCorners[0],
                    targetCorners[0]
                ) ||
                intersects(
                    // reef side line
                    m_r,
                    x_r1,
                    y_r1,
                    // path line 2
                    m_path2,
                    botCorners[2],
                    botCorners[3],
                    // reef side line domain
                    x_r1,
                    x_r2,
                    // path line 2 domain
                    botCorners[2],
                    targetCorners[2]
                )
            ) {
                canAlign = false;
                break;
            }
        }
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Can Align", canAlign);
        SmartDashboard.putNumber("Cur Target X", target.getX());
        SmartDashboard.putNumber("Cur Target Y", target.getY());
        SmartDashboard.putNumber(
            "Cur Target R",
            target.getRotation().getDegrees()
        );

        if (!canAlign) return;

        if (
            mag(
                pidController.getXError(target),
                pidController.getYError(target)
            ) <
            PEDRO_GO_UP
        ) {
            upCommand.execute();
        }

        if (
            !atSetpoint &&
            Math.abs(pidController.getXError(target)) < 0.01 &&
            Math.abs(pidController.getYError(target)) < 0.01 &&
            Math.abs(pidController.getAngleError(target)) < 0.5
        ) {
            atSetpoint = true;
            m_swerve.stopSwerve();
        }

        if (
            !atSetpoint &&
            Math.abs(pidController.getXError(target)) < 3 &&
            Math.abs(pidController.getYError(target)) < 3
        ) {
            pidController.alignLimelight(target);
        }
    }

    @Override
    public boolean isFinished() {
        return atSetpoint;
    }
}
