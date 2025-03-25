package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;

public class TurnAroundPIDController extends PIDController {

    private static final double TARGET_YAW = 55;

    private Swerve m_swerve;

    public TurnAroundPIDController(Swerve swerve) {
        super(0.3, 0, 0.03);
        setTolerance(1);
        enableContinuousInput(-180, 180);

        this.m_swerve = swerve;
    }

    public void turn() {
        double yaw = calculate(getError(), 0);

        m_swerve.turnStates(yaw, 0, 0);
    }

    public double getError() {
        double error = m_swerve.gyro.getYaw() * 360 - TARGET_YAW;
        SmartDashboard.putNumber("Yaw Error", error);
        return error;
    }
}
