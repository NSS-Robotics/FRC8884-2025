package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class RunIndexer extends Command {

    private final Indexer m_indexer;
    private final double velocity;

    public RunIndexer(Indexer m_indexer, double velocity) {
        this.m_indexer = m_indexer;
        this.velocity = velocity;

        addRequirements(m_indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_indexer.setIndexer(velocity);
    }

    // Called once the command ends or is interrupted%.
    @Override
    public void end(boolean interrupted) {
        m_indexer.stopIndexer();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
