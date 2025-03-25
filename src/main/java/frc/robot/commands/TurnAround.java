package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class TurnAround extends Command {

    private Swerve m_swerve;
    private TurnAroundPIDController pidController;

    private boolean atSetpoint;

    public TurnAround(Swerve swerve) {
        this.m_swerve = swerve;
        this.pidController = new TurnAroundPIDController(this.m_swerve);

        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        atSetpoint = false;
    }

    @Override
    public void execute() {
        if (!atSetpoint && Math.abs(pidController.getError()) < 1) {
            atSetpoint = true;
            m_swerve.stopSwerve();
        }

        pidController.turn();
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
