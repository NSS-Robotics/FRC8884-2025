package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotState;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class Align extends Command {

    private static final double RED_BLUE_OFFSET = 8.57;

    private RobotContainer rob;
    private Swerve m_swerve;
    private AlignPIDController pidController;
    private Pose2d target;
    private boolean atSetpoint;

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
        // 11 Fix Left
        new Pose2d(
            //TODO: WE NEED TO FIX THIS GUYS GUYS GUYS GUYS
            11.936229220556926,
            3.023850285434846,
            Rotation2d.fromDegrees(32.357416721193864)
        ),
        new Pose2d(
            12.737658259261673,
            2.567307667187257,
            Rotation2d.fromDegrees(84.29466186011739)
        ),
    };

    private final Pose2d[] redAlgaePoses = {
        // 6
        new Pose2d(
            13.372168987698355,
            2.515705824540677,
            Rotation2d.fromDegrees(101.72460218856601)
        ),
        // 7
        new Pose2d(
            14.474698233432257,
            3.511730009749397,
            Rotation2d.fromDegrees(149.11166780153667)
        ),
        // 8
        new Pose2d(
            14.102636158059832,
            5.143072887067664,
            Rotation2d.fromDegrees(-138.1602471700646)
        ),
        // 9
        new Pose2d(
            12.741693919859538,
            5.527083506044402,
            Rotation2d.fromDegrees(-82.5972415409531)
        ),
        // 10
        new Pose2d(
            11.633289217372596,
            4.562910020151264,
            Rotation2d.fromDegrees(-29.400396067552148)
        ),
        // 11
        new Pose2d(
            11.936229220556926,
            3.023850285434846,
            Rotation2d.fromDegrees(32.357416721193864)
        ),
    };

    private final Pose2d[] blueCoralPoses = new Pose2d[redCoralPoses.length];
    private final Pose2d[] blueAlgaePoses = new Pose2d[redAlgaePoses.length];

    private final double redBargeX = 0;
    private final double blueBargeX = 0;

    public Align(RobotContainer rob, Swerve swerve) {
        this.rob = rob;
        this.m_swerve = swerve;
        pidController = new AlignPIDController(swerve);

        for (int i = 0; i < blueCoralPoses.length; i++) {
            Pose2d pose = redCoralPoses[i];

            blueCoralPoses[i] = new Pose2d(
                pose.getX() - RED_BLUE_OFFSET,
                pose.getY(),
                pose.getRotation()
            );
        }

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
        if (rob.scoringLevel.equals(RobotState.barge)) {
            Pose2d pose = m_swerve.getPose();
            boolean isRed = Math.abs(pose.getX() - redBargeX) < Math.abs(pose.getX() - blueBargeX);

            target = new Pose2d(
                isRed ? redBargeX : blueBargeX,
                pose.getY(),
                new Rotation2d(isRed ? 180 : 0)
            );
            return;
        }

        atSetpoint = false;
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
        }

        if (
            !atSetpoint &&
            Math.abs(pidController.getXError(target)) < 2 &&
            Math.abs(pidController.getYError(target)) < 2
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
    public void end(boolean interrupted) {
        m_swerve.stopSwerve();
    }

    @Override
    public boolean isFinished() {
        return atSetpoint;
    }
}
