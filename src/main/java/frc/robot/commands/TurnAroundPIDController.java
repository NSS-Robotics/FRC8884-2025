package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;

public class TurnAroundPIDController extends PIDController {

    private Swerve m_swerve;

    public TurnAroundPIDController(Swerve swerve) {
        super(0.3, 0, 0.03);
        setTolerance(1);
        enableContinuousInput(-180, 180);

        this.m_swerve = swerve;
    }

    public void turn(double targetYaw) {
        double yaw = calculate(getError(), 0);

        m_swerve.turnStates(yaw, 0, 0);
    }

    public double getError(double targetYaw) {
        double error = Rotation2d.fromDegrees(m_swerve.gyro.getYaw() * 360)
            .plus(Rotation2d.fromDegrees(targetYaw).unaryMinus())
            .getDegrees();
        SmartDashboard.putNumber("Yaw Error", error);
        return error;
    }
}
