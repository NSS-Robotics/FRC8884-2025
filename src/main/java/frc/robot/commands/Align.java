package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotState;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class Align extends Command {

    private static final double PEDRO_GO_UP = 2.5;
    private static final double PEDRO_GO_UP_AUTO = 2.5;
    private static final double RED_BLUE_OFFSET = 8.569576;

    private RobotContainer rob;
    private Swerve m_swerve;
    private AlignPIDController pidController;
    private Pose2d target;
    private boolean atSetpoint;
    private double xTolerance;
    private double yTolerance;
    private double rTolerance;

    private final Pose2d[] redL1Poses = {
        // 6
        new Pose2d(
            12.648779945166266,
            2.4684989462587814,
            Rotation2d.fromDegrees(53.98024239366128)
        ),
        new Pose2d(
            14.560310462335117,
            3.558725472835589,
            Rotation2d.fromDegrees(-175.30412722281832)
        ),
        // 7
        new Pose2d(
            14.201058007081613,
            8 - 5.09842946655324,
            Rotation2d.fromDegrees(114.36672494755307)
        ),
        new Pose2d(
            14.201058007081613,
            5.09842946655324,
            Rotation2d.fromDegrees(-114.36672494755307)
        ),
        // 8
        new Pose2d(
            14.60273292271843,
            4.442631425979867,
            Rotation2d.fromDegrees(173.9508284788378)
        ),
        new Pose2d(
            12.720033253898677,
            5.551081323729148,
            Rotation2d.fromDegrees(-54.99991727968953)
        ),
        // 9
        new Pose2d(
            13.468060384967364,
            5.567111252491732,
            Rotation2d.fromDegrees(-126.33236078083564)
        ),
        new Pose2d(
            11.570111821884012,
            4.505731222931736,
            Rotation2d.fromDegrees(4.692670352466536)
        ),
        // 10
        new Pose2d(
            11.932886066650994,
            5.127923898600779,
            Rotation2d.fromDegrees(-66.48094346729839)
        ),
        new Pose2d(
            11.894687198114651,
            2.978409464013095,
            Rotation2d.fromDegrees(64.56303823886117)
        ),
        // 11
        new Pose2d(
            11.514464849624483,
            3.6065191063628173,
            Rotation2d.fromDegrees(-6.089001164643469)
        ),
        new Pose2d(
            13.37806864381898,
            2.484554728269225,
            Rotation2d.fromDegrees(124.14802881172679)
        ),
    };

    private final Pose2d[] blueL1Poses = {
        // 19
        new Pose2d(
            4.916662567085468,
            5.600762810456995,
            Rotation2d.fromDegrees(-125.74868403149233)
        ),
        new Pose2d(
            3.0093958308432756,
            4.482416931920206,
            Rotation2d.fromDegrees(5.244653279838466)
        ),
        // 18
        new Pose2d(
            3.534583206364148,
            5.1294588090887725,
            Rotation2d.fromDegrees(-65.30523239251472)
        ),
        new Pose2d(
            3.3556953727680328,
            2.979633590250977,
            Rotation2d.fromDegrees(65.34416226013789)
        ),
        // 17
        new Pose2d(
            2.929293224307501,
            3.611890139904748,
            Rotation2d.fromDegrees(-5.784001316973641)
        ),
        new Pose2d(
            4.839822201325507,
            2.5085741431425754,
            Rotation2d.fromDegrees(125.12885038238431)
        ),
        // 22
        new Pose2d(
            4.054235827487181,
            2.4628621954561694,
            Rotation2d.fromDegrees(53.77604300375448)
        ),
        new Pose2d(
            5.9700579196124615,
            3.569791861710402,
            Rotation2d.fromDegrees(-174.35485674381573)
        ),
        // 21
        new Pose2d(
            5.628495347469564,
            2.875846396139371,
            Rotation2d.fromDegrees(113.84256649659072)
        ),
        new Pose2d(
            5.624467998972,
            5.087008110547992,
            Rotation2d.fromDegrees(-114.26117886248036)
        ),
        // 20
        new Pose2d(
            6.040638216749063,
            4.442352828754763,
            Rotation2d.fromDegrees(174.31919809409044)
        ),
        new Pose2d(
            4.136044861501736,
            5.5474082926978125,
            Rotation2d.fromDegrees(-54.62810044573254)
        ),
    };

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

    // private final double redBargeX = 10.253391158244787;
    // private final double blueBargeX = 7.3377792414058485;

    private final Pose2d redProcessorPose = new Pose2d(
        11.584401479408479,
        7.454700444315582,
        Rotation2d.fromDegrees(90)
    );
    private final Pose2d blueProcessorPose = new Pose2d(
        5.940399864994866,
        0.5292682158376304,
        Rotation2d.fromDegrees(-90)
    );

    private final Pose2d autoStationAlign = new Pose2d(
        1.5338556202793288,
        0.9687364585534288,
        Rotation2d.fromDegrees(52.08101368211756)
    );

    private Timer timer;
    private Up upCommand;

    private int postIndex;

    public Align(
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

    public double mag(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    public double dot(double x1, double y1, double x2, double y2) {
        return x1 * x2 + y1 * y2;
    }

    public int signOfAngle(double x1, double y1, double x2, double y2) {
        return x1 * y2 - y1 * x2 > 0 ? 1 : -1;
    }

    @Override
    public void initialize() {
        if (!m_swerve.gyroZeroed) return;

        upCommand.initialize();
        atSetpoint = false;

        // if (rob.scoringLevel.equals(RobotState.barge)) {
        //     timer = new Timer();
        //     xTolerance = 0.01;
        //     yTolerance = 0.05;
        //     rTolerance = 0.5;

        //     Pose2d pose = m_swerve.getPose();
        //     boolean isRed =
        //         Math.abs(pose.getX() - redBargeX) <
        //         Math.abs(pose.getX() - blueBargeX);

        //     target = new Pose2d(
        //         isRed ? redBargeX : blueBargeX,
        //         pose.getY(),
        //         Rotation2d.fromDegrees(isRed ? 180 : 0)
        //     );
        //     return;
        // }

        // if (rob.scoringLevel.equals(RobotState.processor)) {
        //     xTolerance = 0.08;
        //     yTolerance = 0.08;
        //     rTolerance = 0.5;
        //     target = m_swerve.isRed() ? redProcessorPose : blueProcessorPose;
        //     return;
        // }

        xTolerance = 0.01;
        yTolerance = 0.01;
        rTolerance = 0.5;

        Pose2d botPose = m_swerve.getPose();

        Pose2d min1 = new Pose2d();
        Pose2d min2 = new Pose2d();

        double minDist = Double.MAX_VALUE;
        double minDist2 = Double.MAX_VALUE;

        Pose2d[] poses = m_swerve.isRed()
            ? rob.isCoral
                ? (rob.scoringLevel.equals(RobotState.l1)
                        ? redL1Poses
                        : redCoralPoses)
                : redAlgaePoses
            : rob.isCoral
                ? (rob.scoringLevel.equals(RobotState.l1)
                        ? blueL1Poses
                        : blueCoralPoses)
                : blueAlgaePoses;

        // find target with minimum distance
        for (Pose2d target : poses) {
            double dist = mag(
                target.getX() - botPose.getX(),
                target.getY() - botPose.getY()
            );

            if (dist < minDist) {
                minDist = dist;
                min1 = target;
            }
        }

        // find target with second minimum distance
        // and associated left/right positions only if scoring coral
        if (rob.isCoral) {
            for (Pose2d target : poses) {
                double dist = mag(
                    target.getX() - botPose.getX(),
                    target.getY() - botPose.getY()
                );

                if (dist < minDist2 && !target.equals(min1)) {
                    minDist2 = dist;
                    min2 = target;
                }
            }

            double botRotation = botPose.getRotation().getRadians();
            // line of sight vector
            double lX = Math.cos(botRotation);
            double lY = Math.sin(botRotation);
            // vector a: closest target
            double aX = min1.getX() - botPose.getX();
            double aY = min1.getY() - botPose.getY();
            // vector b: second closest target
            double bX = min2.getX() - botPose.getX();
            double bY = min2.getY() - botPose.getY();

            double angleAL =
                Math.acos((dot(aX, aY, lX, lY) / mag(aX, aY))) *
                signOfAngle(lX, lY, aX, aY);
            double angleBL =
                Math.acos((dot(bX, bY, lX, lY) / mag(bX, bY))) *
                signOfAngle(lX, lY, bX, bY);

            Pose2d left;
            Pose2d right;

            if (angleAL > angleBL) {
                left = min1;
                right = min2;
            } else {
                left = min2;
                right = min1;
            }

            target = postIndex != -1
                ? poses[postIndex]
                : rob.isLeft ? left : right;
        } else {
            target = min1;
        }
    }

    @Override
    public void execute() {
        if (!m_swerve.gyroZeroed) return;

        if (
            mag(
                pidController.getXError(target),
                pidController.getYError(target)
            ) <
            (DriverStation.isAutonomousEnabled()
                    ? PEDRO_GO_UP_AUTO
                    : PEDRO_GO_UP)
        ) {
            upCommand.execute();
        }

        if (
            !atSetpoint &&
            Math.abs(pidController.getXError(target)) < xTolerance &&
            Math.abs(pidController.getYError(target)) < yTolerance &&
            Math.abs(pidController.getAngleError(target)) < rTolerance
        ) {
            atSetpoint = true;
            m_swerve.stopSwerve();
        }

        double maxAlign = postIndex != -1 ? 3 : 2;

        if (
            !atSetpoint &&
            Math.abs(pidController.getXError(target)) < maxAlign &&
            Math.abs(pidController.getYError(target)) < maxAlign
        ) {
            // double botX = m_swerve.getPose().getX();

            // if (
            //     rob.scoringLevel.equals(RobotState.barge) &&
            //     (blueBargeX < botX && botX < redBargeX)
            // ) {
            //     return;
            // }

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
        // boolean isBarge = rob.scoringLevel.equals(Constants.RobotState.barge);

        // if (isBarge && !timer.isRunning()) {
        //     timer.restart();
        // }

        // return (!isBarge || timer.hasElapsed(2)) ? atSetpoint : false;
        return atSetpoint;
    }
}
