package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ClimbBalance extends Command {

    private Swerve m_swerve;
    private Intake m_intake;
    private ClimbPIDController pidController;

    private boolean atSetpoint;

    public ClimbBalance(Swerve swerve, Intake intake) {
        this.m_swerve = swerve;
        this.m_intake = intake;

        addRequirements(m_swerve, m_intake);
    }

    @Override
    public void initialize() {
        atSetpoint = false;
    }

    @Override
    public void execute() {
        if (!atSetpoint && Math.abs(pidController.getError()) < 0.1) {
            atSetpoint = true;
            m_swerve.stopSwerve();
        }

        pidController.balance();
    }

    @Override
    public boolean isFinished() {
        return atSetpoint;
    }
}
