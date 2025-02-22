package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlignPIDController extends PIDController {
    private Swerve swerve;

    public AlignPIDController(Swerve swerve) {
        super(0.01, 0.001, 0);
        setTolerance(5);
        enableContinuousInput(-180, 180);
        this.swerve = swerve;
    }

    public void alignLimelight(Pose2d target) {
        double[] translation = translationPID(target);
        SmartDashboard.putNumberArray("target", new double[] {target.getX(), target.getY(), target.getRotation().getDegrees()});
        swerve.turnStates(turnPID(target), translation[0], translation[1]);
    }

    public static double getAngleError(Swerve swerve, Pose2d target) {
        double poseR = swerve.getPose().getRotation().getDegrees();
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
        setPID(0.05, 0, 0);
        double rx = calculate(getAngleError(swerve, target), 0);
        return rx;  
    }

    public double[] translationPID(Pose2d target) {
        setPID(2, 0, 0.00003 );
        double tx = calculate(getXError(swerve, target), 0);
        double ty = calculate(getYError(swerve, target), 0);
        return new double[] {tx, ty};

    }

    public double getXError(Swerve swerve, Pose2d target) {
        double poseX = swerve.getPose().getX();
        double errorX = poseX - target.getX();
        SmartDashboard.putNumber("x error", errorX);
        return errorX;
    }

    public double getYError(Swerve swerve, Pose2d target) {
        double poseY = swerve.getPose().getY();
        double errorY = poseY - target.getY();
        SmartDashboard.putNumber("y error", errorY);
        return errorY;
    }
}
