package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class Align extends Command {

    private static final double PRE_ALIGN_POS_OFFSET = 0.5;

    private RobotContainer rob;
    private Swerve m_swerve;
    private AlignPIDController pidController;
    private Pose2d target;
    private Pose2d preAlignTarget;
    private boolean preAligned;

    private Pose2d[] redCoralPoses = {
        // 6
        new Pose2d(
            13.7138173692834,
            2.6072262240858235,
            Rotation2d.fromDegrees(120)
        ),
        new Pose2d(
            14.00444650156854,
            2.834064075170478,
            Rotation2d.fromDegrees(120)
        ),
        // 7
        new Pose2d(14.429, 3.860, Rotation2d.fromDegrees(180)),
        new Pose2d(14.429, 4.140, Rotation2d.fromDegrees(180)),
        // 8
        new Pose2d(13.910, 5.197, Rotation2d.fromDegrees(-120)),
        new Pose2d(13.621, 5.386, Rotation2d.fromDegrees(-120)),
        // 9
        new Pose2d(12.455, 5.316, Rotation2d.fromDegrees(-60)),
        new Pose2d(12.155, 5.167, Rotation2d.fromDegrees(-60)),
        // 10
        new Pose2d(11.577, 4.160, Rotation2d.fromDegrees(0)),
        new Pose2d(11.577, 3.831, Rotation2d.fromDegrees(0)),
        // 11
        new Pose2d(
            12.165801050386174,
            2.7832427673359215,
            Rotation2d.fromDegrees(60)
        ),
        new Pose2d(
            12.500078054561705,
            2.662783290683802,
            Rotation2d.fromDegrees(60)
        ),
    };

    private Pose2d[] blueCoralPoses = {
        // 17
        new Pose2d(3.64, 2.863, Rotation2d.fromDegrees(60)),
        new Pose2d(3.943, 2.664, Rotation2d.fromDegrees(60)),
        // 18
        new Pose2d(3.053, 4.111, Rotation2d.fromDegrees(0)),
        new Pose2d(3.053, 3.84, Rotation2d.fromDegrees(0)),
        // 19
        new Pose2d(3.943, 5.359, Rotation2d.fromDegrees(-60)),
        new Pose2d(3.64, 5.173, Rotation2d.fromDegrees(-60)),
        // 20
        new Pose2d(5.35, 5.173, Rotation2d.fromDegrees(-120)),
        new Pose2d(5.098, 5.359, Rotation2d.fromDegrees(-120)),
        // 21
        new Pose2d(5.88, 3.84, Rotation2d.fromDegrees(180)),
        new Pose2d(5.88, 4.111, Rotation2d.fromDegrees(180)),
        // 22
        new Pose2d(5.098, 2.664, Rotation2d.fromDegrees(120)),
        new Pose2d(5.35, 2.863, Rotation2d.fromDegrees(120)),
    };

    public Align(RobotContainer rob, Swerve swerve) {
        this.rob = rob;
        this.m_swerve = swerve;
        pidController = new AlignPIDController(swerve);

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
        preAligned = false;
        Pose2d botPose = m_swerve.getPose();

        Pose2d min1 = new Pose2d();
        Pose2d min2 = new Pose2d();

        double minDist = Double.MAX_VALUE;
        double minDist2 = Double.MAX_VALUE;

        Pose2d[] coralPoses = m_swerve.isRed() ? redCoralPoses : blueCoralPoses;

        // find target with minimum distance
        for (Pose2d target : coralPoses) {
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
        for (Pose2d target : coralPoses) {
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

        Rotation2d rot = target.getRotation();
        preAlignTarget = new Pose2d(
            target.getX() - PRE_ALIGN_POS_OFFSET * rot.getCos(),
            target.getY() - PRE_ALIGN_POS_OFFSET * rot.getSin(),
            rot
        );
    }

    @Override
    public void execute() {
        if (
            !preAligned &&
            Math.abs(pidController.getXError(preAlignTarget)) < 0.1 &&
            Math.abs(pidController.getYError(preAlignTarget)) < 0.1 &&
            Math.abs(pidController.getAngleError(preAlignTarget)) < 0.75
        ) {
            preAligned = true;
        }

        Pose2d curTarget = preAligned ? target : preAlignTarget;
        if (
            Math.abs(pidController.getXError(curTarget)) < 2 &&
            Math.abs(pidController.getYError(curTarget)) < 2
        ) {
            pidController.alignLimelight(curTarget, preAligned);
        }

        SmartDashboard.putNumber("Cur Target X", curTarget.getX());
        SmartDashboard.putNumber("Cur Target Y", curTarget.getY());
        SmartDashboard.putNumber(
            "Cur Target R",
            curTarget.getRotation().getDegrees()
        );
    }

    @Override
    public void end(boolean interrupted) {}
}
