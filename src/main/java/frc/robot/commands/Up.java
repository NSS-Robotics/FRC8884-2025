package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class Up extends Command {

    private final RobotContainer robotContainer;
    private final RobotState targetState;
    private final Elevator m_elevator;
    private final Wrist m_wrist;
    private final double targetElevatorPos;
    private final double targetWristPos;

    private final double outWristPos = Constants.WristConstants.pos[RobotState.algaeGround.ordinal()];
    private final RobotState[] safeFromStates = {RobotState.handoff, RobotState.algaeGround};
    private boolean isSafeFromState = false;

    public Up(RobotContainer robotContainer, RobotState targetState, Elevator elevator, Wrist wrist) {
        this.robotContainer = robotContainer;
        this.targetState = targetState;
        m_elevator = elevator;
        m_wrist = wrist;
        targetElevatorPos = Constants.ElevatorConstants.pos[targetState.ordinal()];
        targetWristPos = Constants.WristConstants.pos[targetState.ordinal()];
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isSafeFromState = false;
        for (RobotState safeFromState: safeFromStates) {
            if (robotContainer.state.equals(safeFromState)) {
                isSafeFromState = true;
                break;
            }
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (isSafeFromState) {
            m_wrist.setWrist(outWristPos); // safe position for elev to move up
            // only move elevator when wrist out (prevents dismembering)
            if (Math.abs(m_wrist.getPosition() - outWristPos) < Constants.WristConstants.posTolerance) {
                m_elevator.setElevator(targetElevatorPos, Constants.ElevatorConstants.upSlot);
            }
            // only move wrist when elevator up (prevents wrist/bumper collision)
            if (m_elevator.getPosition() > Constants.ElevatorConstants.wristDownSafePos) {
                m_wrist.setWrist(targetWristPos);
            }
        }
    }

    // Called once the command ends or is interrupted%.
    @Override
    public void end(boolean interrupted) {
        robotContainer.state = targetState;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
