package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorLevel extends Command {

    private final Elevator m_elevator;
    private int slot;
    private final double posSlot1;
    private final double pos;

    public ElevatorLevel(Elevator elevator, int level) {
        m_elevator = elevator;
        posSlot1 = Constants.ElevatorConstants.levelsSlot1[level];
        double targetPos = Constants.ElevatorConstants.levels[level];
        this.pos = targetPos + Constants.ElevatorConstants.pidOffset;
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        slot = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("Slot of elevator", slot);
        SmartDashboard.putNumber("posSlot1", posSlot1);
        m_elevator.setElevator(pos, slot);
        if (m_elevator.getPosition() >= posSlot1) {
            slot = 1;
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
