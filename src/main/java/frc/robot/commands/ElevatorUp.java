package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.Elevator;

public class ElevatorUp extends Command {

    private final Elevator m_elevator;
    private int slot = 0;
    private double pos = 0;
    private final Supplier<RobotState> getRobotState;

    public ElevatorUp(Elevator elevator, Supplier<RobotState> getRobotState) {
        m_elevator = elevator;
        this.getRobotState = getRobotState;
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pos = Constants.ElevatorConstants.pos[getRobotState.get().ordinal()];
    }

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
