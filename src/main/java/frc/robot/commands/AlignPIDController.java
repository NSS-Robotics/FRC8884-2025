package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;

public class AlignPIDController extends PIDController {

    private Swerve m_swerve;
    private double kPT;
    private double kPR;
    private double kDR;
    private double kDT;

    public AlignPIDController(
        Swerve swerve,
        double kPR,
        double kPT,
        double kDR,
        double kDT
    ) {
        super(0.01, 0.001, 0);
        setTolerance(3);
        enableContinuousInput(-180, 180);
        this.m_swerve = swerve;
        this.kPT = kPT;
        this.kPR = kPR;
        this.kDT = kDT;
        this.kDR = kDR;
    }

    public void alignLimelight(Pose2d target) {
        double[] translation = translationPID(target);
        SmartDashboard.putNumberArray(
            "target",
            new double[] {
                target.getX(),
                target.getY(),
                target.getRotation().getDegrees(),
            }
        );

        if (m_swerve.isRed()) {
            m_swerve.turnStates(
                turnPID(target),
                translation[0],
                translation[1]
            );
        } else {
            m_swerve.turnStates(
                turnPID(target),
                -translation[0],
                -translation[1]
            );
        }
    }

    public double getAngleError(Pose2d target) {
        Rotation2d poseR = m_swerve.getPose().getRotation();
        Rotation2d targetR = target.getRotation();

        double errorR = poseR.plus(targetR.unaryMinus()).getDegrees();
        SmartDashboard.putNumber("angle error", errorR);
        return errorR;
    }

    public double turnPID(Pose2d target) {
        setPID(kPR, 0, kDR);
        double rx = calculate(getAngleError(target), 0);
        return rx;
    }

    public double[] translationPID(Pose2d target) {
        setPID(kPT, 0, kDT);
        double tx = calculate(getXError(target), 0);
        double ty = calculate(getYError(target), 0);
        return new double[] { tx, ty };
    }

    public double getXError(Pose2d target) {
        double poseX = m_swerve.getPose().getX();
        double errorX = poseX - target.getX();
        SmartDashboard.putNumber("x error", errorX);
        return errorX;
    }

    public double getYError(Pose2d target) {
        double poseY = m_swerve.getPose().getY();
        double errorY = poseY - target.getY();
        SmartDashboard.putNumber("y error", errorY);
        return errorY;
    }
}
