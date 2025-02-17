package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class RaiseElevator extends Command {

    private final Elevator m_elevator;
    private final double pos;
    private final int slot;

    public RaiseElevator(Elevator elevator, double pos, int slot) {
        m_elevator = elevator;
        this.pos = pos;
        this.slot = slot;

        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_elevator.setElevator(pos, slot);
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
