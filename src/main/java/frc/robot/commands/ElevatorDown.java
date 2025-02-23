package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class ElevatorDown extends Command {

    private final Elevator m_elevator;
    private final Wrist m_wrist;
    private final double elevatorTargetPos = 
        Constants.ElevatorConstants.pos[RobotState.handoff.ordinal()];
    private final double outWristPos =
        Constants.WristConstants.pos[RobotState.algaeGround.ordinal()];
    private final double wristHandoffPos =
        Constants.WristConstants.pos[RobotState.handoff.ordinal()];

    public ElevatorDown(Elevator elevator, Wrist wrist) {
        m_elevator = elevator;
        m_wrist = wrist;
        addRequirements(m_elevator, m_wrist);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_elevator.getPosition() > Constants.ElevatorConstants.upThreshold) {
            m_wrist.setWrist(outWristPos);
        } else {
            m_wrist.setWrist(wristHandoffPos);
        }

        if (m_wrist.getPosition() < Constants.WristConstants.maxElevatorLoweredPos) {
            m_elevator.setElevator(elevatorTargetPos, Constants.ElevatorConstants.downSlot);
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
