package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class ElevatorDown extends Command {

    private final RobotContainer robotContainer;
    private final Elevator m_elevator;
    private final Wrist m_wrist;
    private final Claw m_claw;
    private final double elevatorTargetPos =
        Constants.ElevatorConstants.pos[RobotState.handoff.ordinal()];
    private final double outWristPos =
        Constants.WristConstants.pos[RobotState.algaeGround.ordinal()];
    private final double handoffWristPos =
        Constants.WristConstants.pos[RobotState.handoff.ordinal()];
    private final double algaeHoldWristPos =
        Constants.WristConstants.pos[RobotState.algaeGround.ordinal()];

    public ElevatorDown(
        RobotContainer robotContainer,
        Elevator elevator,
        Wrist wrist,
        Claw claw
    ) {
        this.robotContainer = robotContainer;
        m_elevator = elevator;
        m_wrist = wrist;
        m_claw = claw;
        addRequirements(m_elevator, m_wrist, m_claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        robotContainer.runningCommand = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // coral
        if (robotContainer.isCoral) {
            m_claw.stopClaw();
            if (
                m_elevator.getPosition() >
                Constants.ElevatorConstants.upThreshold
            ) {
                m_wrist.setWrist(outWristPos);
            } else {
                m_wrist.setWrist(handoffWristPos);
            }
            if (
                m_wrist.getPosition() <
                Constants.WristConstants.maxElevatorLoweredPos
            ) {
                m_elevator.setElevator(
                    elevatorTargetPos,
                    Constants.ElevatorConstants.downSlot
                );
            }
        }
        // algae
        else {
            m_wrist.setWrist(algaeHoldWristPos);
            if (
                Math.abs(m_wrist.getPosition() - algaeHoldWristPos) <
                Constants.WristConstants.posTolerance
            ) {
                m_elevator.setElevator(
                    Constants.ElevatorConstants.pos[RobotState.processor.ordinal()],
                    Constants.ElevatorConstants.algaeSlot
                );
            }

            // when elevator down, cmd doesn't end but is effectively over.
            if (!m_claw.gamePieceDetected()) {
                m_claw.stopClaw();
            }
        }
    }

    // Called once the command ends or is interrupted%.
    @Override
    public void end(boolean interrupted) {
        if (!m_claw.gamePieceDetected()) {
            m_claw.stopClaw();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (
            robotContainer.isCoral &&
            m_elevator.getPosition() < 0.05 &&
            Math.abs(m_wrist.getPosition() - handoffWristPos) <
            Constants.WristConstants.posTolerance
        );
    }
}
