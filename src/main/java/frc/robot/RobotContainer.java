package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotState;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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
    private final Limelight l_limelightLow = new Limelight("low");
    private final Limelight l_limelightHigh = new Limelight("high");
    private final Swerve m_swerve = new Swerve(
        l_limelightLow,
        l_limelightHigh,
        this
    );
    private final Wrist m_wrist = new Wrist();
    // private final LED l_led = new LED(this);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private final CommandPS4Controller m_operatorController =
        new CommandPS4Controller(OperatorConstants.kOperatorControllerPort);

    private final int translationAxis = XboxController.Axis.kRightY.value;
    private final int strafeAxis = XboxController.Axis.kRightX.value;
    private final int rotationAxis = XboxController.Axis.kLeftX.value;
    private final Trigger povDown = m_driverController.povDown();
    public boolean isCoral = true;
    public boolean isLeft = true;
    public boolean fieldCentric = false;
    public RobotState scoringLevel;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        m_swerve.setDefaultCommand(
            new TeleopSwerve(
                m_swerve,
                () -> m_driverController.getRawAxis(translationAxis),
                () -> m_driverController.getRawAxis(strafeAxis),
                () -> m_driverController.getRawAxis(rotationAxis) * 0.75,
                () -> povDown.getAsBoolean()
            )
        );

        scoringLevel = RobotState.l2;
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
        m_driverController
            .povRight()
            .whileTrue(
                new RunWrist(
                    m_wrist,
                    m_elevator,
                    Constants.WristConstants.pos[RobotState.algaeGround.ordinal()]
                )
            );
        // m_driverController
        //     .b()
        //     .onTrue(
        //         new SequentialCommandGroup(
        //             new ParallelDeadlineGroup(
        //                 new WaitCommand(0.75),
        //                 new RunServos(m_climber, false)
        //             ),
        //             new UpClimb(m_climber, m_wrist, m_intake, m_elevator)
        //         )
        //     );

        // m_driverController
        //     .a()
        //     .onTrue(
        //         new SequentialCommandGroup(
        //             new ParallelDeadlineGroup(
        //                 new WaitCommand(0.5),
        //                 new RunServos(m_climber, true)
        //             ),
        //             new DownClimb(m_climber)
        //         )
        //     );
        m_driverController
            .y()
            .whileTrue(new InstantCommand(m_swerve::zeroGyro));

        m_driverController.povLeft().whileTrue(new Align(this, m_swerve));
        m_driverController
            .rightTrigger()
            .whileTrue(
                new Outtake(
                    m_endEffector,
                    m_wrist,
                    Constants.EndEffectorConstants.outtakeVelocity
                )
            );
        m_driverController
            .leftTrigger()
            .whileTrue(
                new CoralIntake(
                    m_intake,
                    m_indexer,
                    m_wrist,
                    m_endEffector,
                    m_elevator // ,
                    // l_led
                )
            );

        m_driverController
            .rightBumper()
            .onTrue(new Up(this, m_elevator, m_wrist));
        m_driverController
            .leftBumper()
            .onTrue(new ElevatorDown(m_elevator, m_wrist));
        m_operatorController
            .square()
            .whileTrue(new InstantCommand(() -> this.isCoral = true));
        m_operatorController
            .circle()
            .whileTrue(new InstantCommand(() -> this.isCoral = false));
        m_operatorController
            .L1()
            .whileTrue(new InstantCommand(() -> this.isLeft = true));
        m_operatorController
            .R1()
            .whileTrue(new InstantCommand(() -> this.isLeft = false));

        m_operatorController
            .povUp()
            .onTrue(
                new InstantCommand(() -> this.scoringLevel = RobotState.l4)
            );
        m_operatorController
            .povRight()
            .onTrue(
                new InstantCommand(() -> this.scoringLevel = RobotState.l3)
            );
        m_operatorController
            .povLeft()
            .onTrue(
                new InstantCommand(() -> this.scoringLevel = RobotState.l2)
            );
        m_operatorController
            .povDown()
            .onTrue(
                new InstantCommand(() -> this.scoringLevel = RobotState.l1)
            );
        // m_operatorController.cross().whileTrue(new RunLEDs(l_led));
    }

    public Command setupRobot() {
        return new SequentialCommandGroup(
            // new ParallelDeadlineGroup(
            //     new WaitCommand(0.5),
            //     new RunServos(m_climber, true)
            // ),
            // new RestingClimb(m_climber)
        );
    }

    // public RobotState getRobotState() {

    //     return RobotState.l2;
    // }

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
