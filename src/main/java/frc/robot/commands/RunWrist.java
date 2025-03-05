package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

/** An example command that uses an example subsystem. */
public class RunWrist extends Command {

    private final Wrist m_wrist;
    private final Elevator m_elevator;
    private final double pos;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public RunWrist(Wrist clawPivot, Elevator elevator, double pos) {
        m_wrist = clawPivot;
        m_elevator = elevator;
        this.pos = pos;

        addRequirements(m_wrist);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if (m_elevator.getPosition() > 0.47) {
        //     if (0.40 < pos) {
        //         m_clawPivot.setWrist(pos);
        //     }
        // } else {
        if (
            m_elevator.getPosition() < Constants.ElevatorConstants.upThreshold
        ) {
            m_wrist.setWrist(pos);
        }
        // }
    }

    // Called once the command ends or is interrupted%.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (
            Math.abs(m_wrist.getPosition() - pos) <
            Constants.WristConstants.posTolerance
        );
    }
}
