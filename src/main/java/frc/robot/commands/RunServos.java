package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class RunServos extends Command {

    private final Climber m_climber;
    private final boolean latchEngage;

    public RunServos(Climber m_climber, boolean latchEngage) {
        this.m_climber = m_climber;
        this.latchEngage = latchEngage;

        addRequirements(m_climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if (latchEngage) {
        //     m_climber.engageLatch();
        // } else {
        //     m_climber.disengageLatch();
        // }
    }

    // Called once the command ends or is interrupted%.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
