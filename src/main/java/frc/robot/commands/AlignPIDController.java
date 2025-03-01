package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;

public class AlignPIDController extends PIDController {

    private Swerve m_swerve;

    public AlignPIDController(Swerve swerve) {
        super(0.01, 0.001, 0);
        setTolerance(3);
        enableContinuousInput(-180, 180);
        this.m_swerve = swerve;
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
        m_swerve.turnStates(turnPID(target), translation[0], translation[1]);
    }

    public double getAngleError(Pose2d target) {
        double poseR = m_swerve.getPose().getRotation().getDegrees();
        double targetR = target.getRotation().getDegrees();

        if (targetR > 90 || targetR < -90) {
            if (poseR < 0) {
                poseR += 360;
            }
            if (targetR < 0) {
                targetR += 360;
            }
        }

        double errorR = poseR - targetR;
        SmartDashboard.putNumber("angle error", errorR);
        return errorR;
    }

    public double turnPID(Pose2d target) {
        setPID(0.1, 0, 0);
        double rx = calculate(getAngleError(target), 0);
        return rx;
    }

    public double[] translationPID(Pose2d target) {
        setPID(2.5, 0, 0);
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
