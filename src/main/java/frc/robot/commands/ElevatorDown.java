package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Elevator;

public class ElevatorDown extends Command {
    private final Elevator m_elevator;
    private final Wrist m_wrist;
    private final double pos = 0;

    public ElevatorDown(Elevator elevator, Wrist wrist) {
        m_elevator = elevator;
        m_wrist = wrist;
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double wristPosition = m_wrist.getPosition(); // temporary
        if (wristPosition < 0.64) {
            m_elevator.setElevator(pos, Constants.ElevatorConstants.downSlot);
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
