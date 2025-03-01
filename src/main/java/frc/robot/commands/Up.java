package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import java.lang.invoke.ConstantCallSite;

public class Up extends Command {

    private final RobotContainer robotContainer;
    private final Elevator m_elevator;
    private final Wrist m_wrist;
    private final Claw m_claw;
    private final Timer timer;
    private RobotState targetState;
    private double targetElevatorPos;
    private double targetWristPos;
    private double outWristPos =
        Constants.WristConstants.pos[RobotState.algaeGround.ordinal()];

    public Up(
        RobotContainer robotContainer,
        Elevator elevator,
        Wrist wrist,
        Claw claw
    ) {
        this.robotContainer = robotContainer;
        m_elevator = elevator;
        m_wrist = wrist;
        m_claw = claw;
        timer = new Timer();
        addRequirements(m_elevator, m_wrist, m_claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        robotContainer.runningCommand = true;
        targetState = robotContainer.scoringLevel;
        targetElevatorPos =
            Constants.ElevatorConstants.pos[targetState.ordinal()];
        targetWristPos = Constants.WristConstants.pos[targetState.ordinal()];
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // CORAL
        if (robotContainer.isCoral) {
            // Wrist logic
            if (
                m_elevator.getPosition() <
                Constants.ElevatorConstants.upThreshold
            ) {
                m_wrist.setWrist(outWristPos); // safe position for elev to move up
            } else if (
                m_elevator.getPosition() >
                Constants.ElevatorConstants.wristDownSafeThreshold
            ) {
                m_wrist.setWrist(targetWristPos); // only move wrist down when elevator up (prevents wrist/bumper collision)
            }

            // only move elevator when wrist out (prevents dismembering)
            if (
                m_wrist.getPosition() >
                Constants.WristConstants.minElevatorRaisedPos
            ) {
                m_elevator.setElevator(
                    targetElevatorPos,
                    Constants.ElevatorConstants.upSlot
                );
            }
            if (
                Math.abs(m_elevator.getPosition() - targetElevatorPos) <
                Constants.ElevatorConstants.posTolerance
            ) {
                if (m_claw.gamePieceDetected()) {
                    m_claw.setClaw(
                        -Constants.EndEffectorConstants.outtakeVelocity
                    );
                    timer.restart();
                } else if (timer.hasElapsed(0.75)) {
                    m_claw.stopClaw();
                }
            }
        }
        // ALGAE
        else {
            // dont need to move elevator
            if (targetState.equals(RobotState.processor)) {
                if (
                    m_elevator.getPosition() <
                    Constants.ElevatorConstants.upThreshold
                ) {
                    m_wrist.setWrist(targetWristPos);
                    if (
                        Math.abs(m_wrist.getPosition() - targetWristPos) <
                            Constants.WristConstants.posTolerance &&
                        m_claw.gamePieceDetected()
                    ) {
                        m_claw.setClaw(
                            -Constants.EndEffectorConstants.outtakeVelocity
                        );
                        Timer.delay(2);
                        m_claw.stopClaw();
                    }
                    // if (!m_claw.gamePieceDetected()) {
                    //     m_claw.stopClaw();
                    // }
                    // need delay
                }
            }
            // for things we need elevator for
            else {
                if (
                    m_elevator.getPosition() <
                    Constants.ElevatorConstants.upThreshold
                ) {
                    m_wrist.setWrist(
                        Constants.WristConstants.pos[RobotState.barge.ordinal()]
                    ); // safe position for elev to move up
                } else if (
                    m_elevator.getPosition() >
                    Constants.ElevatorConstants.wristDownSafeThreshold
                ) {
                    m_wrist.setWrist(targetWristPos); // only move wrist down when elevator up (prevents wrist/bumper collision)
                }

                if (
                    m_wrist.getPosition() >
                    Constants.WristConstants.minElevatorRaisedPos
                ) {
                    m_elevator.setElevator(
                        targetElevatorPos,
                        Constants.ElevatorConstants.upSlot
                    );
                }
                if (
                    Math.abs(m_elevator.getPosition() - targetElevatorPos) <
                    Constants.ElevatorConstants.posTolerance
                ) {
                    if (
                        targetState.equals(RobotState.algaeReefHigh) ||
                        targetState.equals(RobotState.algaeReefLow)
                    ) {
                        m_claw.setClaw(Constants.EndEffectorConstants.velocity);
                    } else if (
                        m_claw.gamePieceDetected() &&
                        targetState.equals(RobotState.barge)
                    ) {
                        m_claw.setClaw(
                            -Constants.EndEffectorConstants.outtakeVelocity
                        );
                    }
                }
                if (
                    !m_claw.gamePieceDetected() &&
                    targetState.equals(RobotState.barge)
                ) {
                    m_claw.stopClaw();
                }
            }
        }
    }

    // Called once the command ends or is interrupted%.
    @Override
    public void end(boolean interrupted) {
        robotContainer.runningCommand = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
