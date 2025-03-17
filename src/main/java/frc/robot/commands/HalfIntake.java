package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

public class HalfIntake extends Command {

    private final Intake m_intake;
    private final Indexer m_indexer;
    private final Climber m_climber;
    private final RobotContainer robotContainer;
    private final LED l_leds;
    private final CommandXboxController m_driverController;

    // private final LED m_led;

    public HalfIntake(
        RobotContainer robotContainer,
        Intake m_intake,
        Indexer m_indexer,
        CommandXboxController driverController,
        Climber climber,
        LED l_leds
    ) {
        this.m_climber = climber;
        this.robotContainer = robotContainer;
        this.m_intake = m_intake;
        this.m_indexer = m_indexer;
        this.m_driverController = driverController;
        this.l_leds = l_leds;

        addRequirements(m_intake, m_indexer, m_climber, l_leds);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.setPivot(
            DriverStation.isAutonomousEnabled()
                ? Constants.IntakeConstants.autoIntakePosition
                : Constants.IntakeConstants.intakePosition,
            1
        );
        if (m_intake.getPosition() < Constants.IntakeConstants.intakeStartPos) {
            m_intake.setIntake(Constants.IntakeConstants.velocity, false);
            m_indexer.setIndexer(Constants.IndexerConstants.velocity);
        }
        if (m_indexer.gamepieceDetected()) {
            m_intake.stopIntake();
            m_indexer.stopIndexer();
            m_driverController.setRumble(RumbleType.kBothRumble, 1);
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
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (
            m_indexer.gamepieceDetected() && DriverStation.isAutonomousEnabled()
        );
    }
}
