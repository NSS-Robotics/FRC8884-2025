package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorUp extends Command {

    private final Elevator m_elevator;
    private int slot = 0;
    private final double pos;

    public ElevatorUp(Elevator elevator, int level) {
        m_elevator = elevator;
        pos = Constants.ElevatorConstants.levels[level];
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_elevator.setElevator(pos, Constants.ElevatorConstants.upSlot);
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
