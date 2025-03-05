package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class Align extends Command {

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
        // 17
        new Pose2d(
            3.4262242976656996,
            2.919854276977841,
            Rotation2d.fromDegrees(40.88202068828051)
        ),
        new Pose2d(
            4.231882968245696,
            2.5333526571604104,
            Rotation2d.fromDegrees(89.57861213295725)
        ),
        // 18
        new Pose2d(
            2.9951088752426487,
            4.4084558948742885,
            Rotation2d.fromDegrees(-19.667031946761643)
        ),
        new Pose2d(
            3.0535927099289584,
            3.50306401578839,
            Rotation2d.fromDegrees(29.57005577056487)
        ),
        // 19
        new Pose2d(
            4.085112465590101,
            5.513184762437938,
            Rotation2d.fromDegrees(-79.09741439099274)
        ),
        new Pose2d(
            3.3148819377449614,
            4.999087951557525,
            Rotation2d.fromDegrees(-30.72914539869436)
        ),
        // 20
        new Pose2d(
            5.5671103940629765,
            5.125866653006128,
            Rotation2d.fromDegrees(-139.247780993426)
        ),
        new Pose2d(
            4.725074687581623,
            5.536871238624893,
            Rotation2d.fromDegrees(-90.59682516308365)
        ),
        // 21
        new Pose2d(
            5.98140383441108,
            3.646841125383143,
            Rotation2d.fromDegrees(160.5931605707581)
        ),
        new Pose2d(
            5.912799104870625,
            4.572151931157451,
            Rotation2d.fromDegrees(-150.0842691212723)
        ),
        // 22
        new Pose2d(
            4.905252041858879,
            2.5448529264687982,
            Rotation2d.fromDegrees(100.86324577638341)
        ),
        new Pose2d(
            5.673102655617341,
            3.062575736600282,
            Rotation2d.fromDegrees(149.41473467519194)
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
            14.924302814848982,
            4.150537615247084,
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

    private final double redBargeX = 10.253391158244787;
    private final double blueBargeX = 8.24660884176;

    private final Pose2d redProcessorPose = new Pose2d();
    private final Pose2d blueProcessorPose = new Pose2d();

    private Timer timer;

    public Align(RobotContainer rob, Swerve swerve) {
        this.rob = rob;
        this.m_swerve = swerve;
        pidController = new AlignPIDController(swerve);

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
        atSetpoint = false;

        if (rob.scoringLevel.equals(RobotState.barge)) {
            timer = new Timer();

            Pose2d pose = m_swerve.getPose();
            boolean isRed =
                Math.abs(pose.getX() - redBargeX) <
                Math.abs(pose.getX() - blueBargeX);

            target = new Pose2d(
                isRed ? redBargeX : blueBargeX,
                pose.getY(),
                Rotation2d.fromDegrees(isRed ? 180 : 0)
            );
            return;
        }

        if (rob.scoringLevel.equals(RobotState.processor)) {
            target = m_swerve.isRed() ? redProcessorPose : blueProcessorPose;
            return;
        }

        Pose2d botPose = m_swerve.getPose();

        Pose2d min1 = new Pose2d();
        Pose2d min2 = new Pose2d();

        double minDist = Double.MAX_VALUE;
        double minDist2 = Double.MAX_VALUE;

        Pose2d[] poses = m_swerve.isRed()
            ? rob.isCoral ? redCoralPoses : redAlgaePoses
            : rob.isCoral ? blueCoralPoses : blueAlgaePoses;

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

            target = rob.isLeft ? left : right;
        } else {
            target = min1;
        }
    }

    @Override
    public void execute() {
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
            Math.abs(pidController.getXError(target)) < 2 &&
            Math.abs(pidController.getYError(target)) < 2
        ) {
            double botX = m_swerve.getPose().getX();

            if (
                rob.scoringLevel.equals(RobotState.barge) &&
                (blueBargeX < botX && botX < redBargeX)
            ) {
                return;
            }

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
        boolean isBarge = rob.scoringLevel.equals(Constants.RobotState.barge);

        if (isBarge && !timer.isRunning()) {
            timer.restart();
        }

        return (!isBarge || timer.hasElapsed(2)) ? atSetpoint : false;
    }
}
