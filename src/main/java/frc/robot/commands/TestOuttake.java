package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;

public class TestOuttake extends Command {

    private final Claw m_claw;
    private final Wrist m_wrist;
    private final double velocity;

    public TestOuttake(Claw m_claw, Wrist m_wrist, double velocity) {
        this.m_claw = m_claw;
        this.m_wrist = m_wrist;
        this.velocity = -velocity;

        addRequirements(m_claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_wrist.getPosition() > Constants.WristConstants.minOuttakePos) {
            m_claw.setClaw(velocity);
        }
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
