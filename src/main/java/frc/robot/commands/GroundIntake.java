package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class GroundIntake extends Command {

    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Wrist m_wrist;
    private final Claw m_claw;
    private final Elevator m_elevator;
    private final Climber m_climber;
    private final RobotContainer robotContainer;
    private final LED l_leds;
    private final CommandXboxController m_driverController;
    private final double speed;

    // private final LED m_led;

    public GroundIntake(
        RobotContainer robotContainer,
        Intake m_intake,
        Indexer m_indexer,
        Wrist m_wrist,
        Claw m_claw,
        Elevator m_elevator,
        CommandXboxController driverController,
        Climber climber,
        LED l_leds,
        boolean faster
    ) {
        this.m_climber = climber;
        this.robotContainer = robotContainer;
        this.m_intake = m_intake;
        this.m_indexer = m_indexer;
        this.m_wrist = m_wrist;
        this.m_claw = m_claw;
        this.m_elevator = m_elevator;
        this.m_driverController = driverController;
        this.l_leds = l_leds;
        speed = faster ? 4000 : Constants.IntakeConstants.velocity;

        addRequirements(
            m_intake,
            m_indexer,
            m_wrist,
            m_claw,
            m_elevator,
            m_climber,
            l_leds
        );
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        robotContainer.runningCommand = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        l_leds.intakeLeds();
        // coral
        if (m_claw.gamePieceDetected()) {
            m_driverController.setRumble(RumbleType.kBothRumble, 1);
            l_leds.intakeCompleteLeds();
        }
        if (
            m_climber.getPosition() <
                Constants.ClimberConstants.restingRot - 5 &&
            robotContainer.isCoral
        ) {
            m_wrist.setWrist(0.1);
            if (m_wrist.getPosition() > 0.08) {
                m_climber.setClimber(Constants.ClimberConstants.restingRot, 0);
            }
        }
        if (
            robotContainer.isCoral &&
            m_climber.getPosition() > Constants.ClimberConstants.restingRot - 5
        ) {
            if (
                m_wrist.getPosition() <
                Constants.WristConstants.maxElevatorLoweredPos
            ) {
                m_elevator.setElevator(
                    Constants.ElevatorConstants.pos[RobotState.handoff.ordinal()],
                    Constants.ElevatorConstants.downSlot
                );
            }
            if (
                m_elevator.getPosition() <
                Constants.ElevatorConstants.upThreshold
            ) {
                m_wrist.setWrist(
                    Constants.WristConstants.pos[RobotState.handoff.ordinal()]
                );
                m_claw.setClaw(Constants.EndEffectorConstants.velocity);
            }
            if (!m_indexer.gamepieceDetected()) {
                m_intake.setPivot(
                    DriverStation.isAutonomousEnabled()
                        ? Constants.IntakeConstants.autoIntakePosition
                        : Constants.IntakeConstants.intakePosition,
                    1
                );
                if (
                    m_intake.getPosition() <
                        Constants.IntakeConstants.intakeStartPos &&
                    Math.abs(
                        m_wrist.getPosition() -
                        Constants.WristConstants.pos[RobotState.handoff.ordinal()]
                    ) <
                    Constants.WristConstants.posTolerance
                ) {
                    m_intake.setIntake(speed, false);
                    m_indexer.setIndexer(Constants.IndexerConstants.velocity);
                    m_claw.setClaw(Constants.EndEffectorConstants.velocity);
                }
            } else if (
                Math.abs(
                        m_wrist.getPosition() -
                        Constants.WristConstants.pos[RobotState.handoff.ordinal()]
                    ) <
                    Constants.WristConstants.posTolerance &&
                m_claw.getVelocity() > 40
            ) {
                m_intake.setIntake(Constants.IntakeConstants.velocity, false);
                m_indexer.setIndexer(Constants.IndexerConstants.velocity);
            }
        }
        // algae
        else if (!robotContainer.isCoral) {
            if (
                m_elevator.getPosition() <
                Constants.ElevatorConstants.upThreshold
            ) {
                m_wrist.setWrist(
                    Constants.WristConstants.pos[RobotState.algaeGround.ordinal()]
                );
            }
            if (
                Math.abs(
                    m_wrist.getPosition() -
                    Constants.WristConstants.pos[RobotState.algaeGround.ordinal()]
                ) <
                Constants.WristConstants.posTolerance
            ) {
                m_claw.setClaw(Constants.EndEffectorConstants.velocity);
                m_elevator.setElevator(
                    Constants.ElevatorConstants.pos[RobotState.algaeGround.ordinal()],
                    Constants.ElevatorConstants.upSlot
                );
            }
        }
    }

    // Called once the command ends or is interrupted%.
    @Override
    public void end(boolean interrupted) {
        m_intake.setPivot(
            robotContainer.intakeDown
                ? (DriverStation.isAutonomousEnabled()
                        ? Constants.IntakeConstants.autoIntakePosition
                        : Constants.IntakeConstants.intakePosition)
                : Constants.IntakeConstants.upPosition,
            robotContainer.intakeDown ? 1 : 0
        );
        m_intake.stopIntake();
        m_indexer.stopIndexer();
        m_driverController.setRumble(RumbleType.kBothRumble, 0);
        if (robotContainer.isCoral || !m_claw.gamePieceDetected()) {
            m_claw.stopClaw();
            // Reset the leds to their appropritate resting state
            l_leds.updateGamePiece();
        } else {
            m_claw.setClaw(Constants.EndEffectorConstants.holdingVelocity);
            m_elevator.setElevator(
                Constants.ElevatorConstants.pos[RobotState.processor.ordinal()],
                Constants.ElevatorConstants.upSlot
            );
        }
        l_leds.stop();
        robotContainer.runningCommand = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (
            m_claw.gamePieceDetected() && DriverStation.isAutonomousEnabled()
        );
    }
}
