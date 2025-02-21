package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

/** An example command that uses an example subsystem. */
public class RunClawPivot extends Command {

    private final ClawPivot m_clawPivot;
    private final Elevator m_elevator;
    private final double pos;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public RunClawPivot(ClawPivot clawPivot, Elevator elevator, double pos) {
        m_clawPivot = clawPivot;
        m_elevator = elevator;
        this.pos = pos;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_clawPivot);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_elevator.getPosition() > 0.47) {
            if (0.55 < pos) {
                m_clawPivot.setPivot(pos);
            }
        } else {
            m_clawPivot.setPivot(pos);
        }
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
