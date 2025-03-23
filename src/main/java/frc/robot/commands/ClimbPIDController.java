package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;

public class ClimbPIDController extends PIDController {

    private static final double TARGET_PITCH = 0;

    private Swerve m_swerve;
    private Intake m_intake;

    public ClimbPIDController(Swerve swerve, Intake intake) {
        super(0.01, 0.001, 0);
        setTolerance(3);
        enableContinuousInput(-180, 180);

        this.m_swerve = swerve;
        this.m_intake = intake;
    }

    public void balance() {
        double pitch = calculate(getError(), 0);

        m_intake.setPivot(m_intake.getPosition() + pitch, 0);
    }

    public double getError() {
        double error = m_swerve.gyro.getPitch() - TARGET_PITCH;
        SmartDashboard.putNumber("Pitch Error", error);
        return error;
    }
}
