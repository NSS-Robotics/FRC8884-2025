package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClawPivot;
import frc.robot.subsystems.Elevator;

public class ElevatorDown extends Command {

    private final Elevator m_elevator;
    private final ClawPivot m_clawPivot;
    private final double pos = 0;

    public ElevatorDown(Elevator elevator, ClawPivot clawPivot) {
        m_elevator = elevator;
        m_clawPivot = clawPivot;
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double clawPivotPosition = m_clawPivot.getPosition(); // temporary
        if (clawPivotPosition < 0.64) {
            m_elevator.setElevator(pos, Constants.ElevatorConstants.downSlot);
        }
        // if (m_elevator.getPosition() <= posSlot1) {
        //     slot = 1;
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
