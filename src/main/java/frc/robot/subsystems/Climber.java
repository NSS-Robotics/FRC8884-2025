package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private static TalonFX lMotor = new TalonFX(
        Constants.ClimberConstants.rMotorID
    );

    private static Servo lServo = new Servo(
        Constants.ClimberConstants.rChannel
    );

    // private static Follower leader = new Follower(
    //     Constants.ClimberConstants.lMotorID,
    //     true
    // );
    private static Slot0Configs upPID = new Slot0Configs();
    private static Slot1Configs downPID = new Slot1Configs();
    private static PositionVoltage positionPID;
    private static Timer timer = new Timer();
    public boolean latchEngaged = true;

    public Climber() {
        lMotor.clearStickyFaults();
        resetEncoders();
        // Put these in Brake mode when running for real.
        lMotor.setNeutralMode(NeutralModeValue.Brake);
        // rMotor.setControl(leader);

        upPID.kP = Constants.ClimberConstants.upkP;
        upPID.kI = Constants.ClimberConstants.upkI;
        upPID.kD = Constants.ClimberConstants.upkD;
        downPID.kP = Constants.ClimberConstants.downkP;
        downPID.kI = Constants.ClimberConstants.downkI;
        downPID.kD = Constants.ClimberConstants.downkD;

        lMotor.getConfigurator().apply(upPID);
        lMotor.getConfigurator().apply(downPID);
        // rMotor.getConfigurator().apply(upPID);
        // rMotor.getConfigurator().apply(downPID);

        engageLatch();
    }

    public void resetEncoders() {
        lMotor.setPosition(0);
        // rMotor.setPosition(0);
    }

    public void setClimber(double position, int slot) {
        position = Math.max(
            0,
            Math.min(Constants.ClimberConstants.maxRotations, position)
        );
        SmartDashboard.putNumber("Climber target pos", position);
        positionPID = new PositionVoltage(position).withSlot(slot);
        lMotor.setControl(positionPID);
    }

    public void stopClimber() {
        lMotor.stopMotor();
        // rMotor.stopMotor();
    }

    public double getPosition() {
        return lMotor.getPosition().getValueAsDouble();
    }

    public void engageLatch() {
        lServo.setAngle(0);
        // rServo.set(0.5);
        latchEngaged = true;
    }

    public void disengageLatch() {
        lServo.setAngle(90);
        // rServo.set(0.6);
        latchEngaged = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("lServo", lServo.getPosition());
        // SmartDashboard.putNumber("rServo", rServo.getPosition());
        SmartDashboard.putNumber(
            "L Climber Pos",
            lMotor.getPosition().getValueAsDouble()
        );
        // SmartDashboard.putNumber(
        //     "R Climber Pos",
        //     rMotor.getPosition().getValueAsDouble()
        // );
        SmartDashboard.putBoolean("Latch Engaged", latchEngaged);
    }
}
