package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.commands.Claw.*;
import frc.robot.commands.RaiseElevator;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ClawPivot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final Climber m_climber = new Climber();
    private final Elevator m_elevator = new Elevator();
    private final Claw m_endEffector = new Claw();
    private final Indexer m_indexer = new Indexer();
    private final Intake m_intake = new Intake();
    private final Limelight m_limelightLow = new Limelight("low");
    private final Limelight m_limelightHigh = new Limelight("high");
    private final Swerve m_swerve = new Swerve(m_limelightHigh);
    private final ClawPivot m_pivot = new ClawPivot();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private final int translationAxis = XboxController.Axis.kRightY.value;
    private final int strafeAxis = XboxController.Axis.kRightX.value;
    private final int rotationAxis = XboxController.Axis.kLeftX.value;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings

        // m_swerve.setDefaultCommand(
        //   new TeleopSwerve(
        //       m_swerve,
        //       () -> m_driverController.getRawAxis(translationAxis),
        //       () -> m_driverController.getRawAxis(strafeAxis),
        //       () -> -m_driverController.getRawAxis(rotationAxis) * 0.75,
        //       () -> false
        //   )
        // );

        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.

        // m_driverController.a().whileTrue(m_climber.sysIdDynamic(Direction.kForward));
        // m_driverController.b().whileTrue(m_climber.sysIdDynamic(Direction.kReverse));
        // m_driverController.x().whileTrue(m_climber.sysIdQuasistatic(Direction.kForward));
        // m_driverController.y().whileTrue(m_climber.sysIdQuasistatic(Direction.kReverse));

        // m_driverController.rightTrigger().whileTrue(new ClimbPos(m_pivot));
        m_driverController.leftTrigger().whileTrue(new IntakePos(m_pivot));

        m_driverController
            .x()
            .whileTrue(
                new RaiseElevator(
                    m_elevator,
                    0 - Constants.ElevatorConstants.pidOffset,
                    0
                )
            );
        m_driverController
            .a()
            .whileTrue(
                new RaiseElevator(
                    m_elevator,
                    0 - Constants.ElevatorConstants.pidOffset,
                    1
                )
            );
        m_driverController
            .b()
            .whileTrue(
                new RaiseElevator(
                    m_elevator,
                    1 + Constants.ElevatorConstants.pidOffset,
                    0
                )
            );
        m_driverController
            .y()
            .whileTrue(
                new RaiseElevator(
                    m_elevator,
                    1 + Constants.ElevatorConstants.pidOffset,
                    1
                )
            );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Autos.exampleAuto(m_swerve);
    }
}
