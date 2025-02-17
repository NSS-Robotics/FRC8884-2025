package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

/** An example command that uses an example subsystem. */
public class IntakePos extends Command {

    private final ClawPivot m_clawPivot;

    /**
     * Creates a new
     * +.
     *
     * @param subsystem The subsystem used by this command.
     */
    public IntakePos(ClawPivot clawPivot) {
        m_clawPivot = clawPivot;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_clawPivot);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_clawPivot.setPivot(0.5);
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
