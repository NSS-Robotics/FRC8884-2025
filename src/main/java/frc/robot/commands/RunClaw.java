package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class RunClaw extends Command {

    private final Claw m_claw;
    private final double velocity;

    public RunClaw(Claw m_claw, double velocity) {
        this.m_claw = m_claw;
        this.velocity = velocity;

        addRequirements(m_claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_claw.setClaw(velocity);
    }

    // Called once the command ends or is interrupted%.
    @Override
    public void end(boolean interrupted) {
        m_claw.stopClaw();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
