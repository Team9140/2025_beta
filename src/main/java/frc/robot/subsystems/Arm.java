package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

/**
 * The moving arm of the robot that is used to aim the launcher
 **/
public class Arm extends SubsystemBase {
    /**
     * Instance of the arm
     **/
    private static Arm instance;

    /**
     * Kraken motor that rotates the arm
     **/
    private final TalonFX motor = new TalonFX(Constants.Ports.ARM_MOTOR, Constants.Ports.CTRE_CANBUS);

    /**
     * A MagicMotion instance that outputs with a voltage. This is used to simplify
     * PID control
     **/
    // private final MotionMagicVoltage motionMagic;

    private final MotionMagicTorqueCurrentFOC motionMagic;

    // Set PID and SVA values
    Slot0Configs launcherGains = new Slot0Configs()
            .withKP(Constants.Arm.P)
            .withKI(Constants.Arm.I)
            .withKD(Constants.Arm.D)
            .withKS(Constants.Arm.S)
            .withKV(Constants.Arm.V)
            .withKA(Constants.Arm.A);

    Slot0Configs tcGains = new Slot0Configs()
            .withKP(2600)
            .withKD(384)
            .withKA(8.0)
            .withKV(0.03)
            .withKS(3.95)
            .withKG(28.0)
            .withGravityType(GravityTypeValue.Arm_Cosine);

    // Ensure that the motor doesn't die with current limits
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Arm.MAX_CURRENT)
            .withStatorCurrentLimitEnable(true);

    // Sets units to radians of the physical arm
    FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(Constants.Arm.SENSOR_TO_MECHANISM_RATIO);

    // Set max speed / acceleration limits
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.Arm.CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Constants.Arm.ACCELERATION);

    MotionMagicConfigs motionMagicConfigSlow = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.Arm.CRUISE_VELOCITY / 4.0)
            .withMotionMagicAcceleration(Constants.Arm.ACCELERATION / 4.0);

    // Set motor direction
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);

    SoftwareLimitSwitchConfigs softLimit = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(3.5)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(-1.7)
            .withReverseSoftLimitEnable(true);

    // Apply other configs
    TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withSlot0(tcGains)
            .withCurrentLimits(currentLimitsConfigs)
            .withFeedback(feedbackConfigs)
            .withMotionMagic(motionMagicConfigs)
            .withMotorOutput(motorOutputConfigs)
            .withSoftwareLimitSwitch(softLimit);

    private Arm() {

        this.motor.getConfigurator().apply(motorConfig);
        this.motor.setNeutralMode(NeutralModeValue.Brake);

        // Initialize Motion Magic
        // this.motionMagic = new MotionMagicVoltage(Constants.Arm.Positions.INTAKE)
        // .withEnableFOC(true)
        // .withSlot(0)
        // .withFeedForward(Constants.Arm.FEED_FORWARD);

        this.motor.getConfigurator()
                .apply(new TorqueCurrentConfigs()
                        .withPeakForwardTorqueCurrent(Constants.Arm.MAX_CURRENT)
                        .withPeakReverseTorqueCurrent(-Constants.Arm.MAX_CURRENT));

        this.motionMagic = new MotionMagicTorqueCurrentFOC(Constants.Arm.Positions.INTAKE)
                .withSlot(0);


        // Set arm encoder position as starting if it is the first time booting the
        // kraken
        if (Math.abs(this.getAngle()) < Constants.Arm.INITIAL_VARIANCE)
            this.motor.setPosition(-0.25);
    }

    /**
     * Returns an initialized class of Arm if one exists, or create a new one if it
     * doesn't (and return it).
     *
     * @return The arm
     **/
    public static Arm getInstance() {
        return instance == null ? Arm.instance = new Arm() : Arm.instance;
    }

    /**
     * Routinely puts the arm position on the dashboard and enables motion magic
     * motor control
     **/
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position (rotations)", this.getAngle());
        SmartDashboard.putNumber("Arm Position (rads)", this.getAngle() * Math.PI * 2.0);
        SmartDashboard.putNumber("Arm Current", this.motor.getTorqueCurrent().getValueAsDouble());
        if (this.motor.getAppliedControl() instanceof MotionMagicTorqueCurrentFOC) {
            SmartDashboard.putNumber("arm motion profile thingy", ((MotionMagicTorqueCurrentFOC) this.motor.getAppliedControl()).Position);
        }

    }

    public boolean atPosition() {
        return Math.abs(this.getAngle() - this.motionMagic.Position) <= Math.toRadians(2.0);
    }

    /**
     * Checks if the arm is at the requested position
     *
     * @return true if the arm is at the requested position, false otherwise
     **/
    // public boolean isReady() {
    // return Math.abs(this.getAngle() - this.motionMagic.Position) <
    // Constants.Arm.AIM_ERROR;
    // }

    /**
     * Get the arm position
     *
     * @return The arm's angle, in radians
     **/
    public double getAngle() {
        return this.motor.getPosition().getValueAsDouble();
    }

    /**
     * Arm goes to desired angle in radians using motionMagic
     *
     * @return a command that sets the desired angle of the arm
     **/
    public Command setAngle(double position) {
        return this.runOnce(() -> this.motor.setControl(this.motionMagic.withPosition(position)));
    }

    /**
     * Moves arm to stowed position (which is the same as intake)
     *
     * @return a command that sets the desired angle to the correct angle for
     *         intaking, aka the stow position
     **/
    public Command setStow() {
        return this.setIntake();
    }

    public Command setStowSlow() {
        return this.runOnce(() -> this.motor.getConfigurator().apply(motionMagicConfigSlow))
                .andThen(this.setStow())
                .andThen(new WaitUntilCommand(this::atPosition))
                .andThen(this.runOnce(() -> this.motor.getConfigurator().apply(motionMagicConfigs)));
    }

    /**
     * Moves arm to intake position
     *
     * @return a command that sets the desired angle to the angle required for
     *         intaking
     **/
    public Command setIntake() {
        return this.setAngle(Constants.Arm.Positions.INTAKE);
    }

    /**
     * Moves arm to overhand throwing position
     *
     * @return a command that sets the desired angle to the angle required for
     *         overhead launching into the speaker
     **/
    public Command setOverhand() {
        return this.setAngle(Constants.Arm.Positions.OVERHAND);
    }

    /**
     * Moves arm to underhand throwing position
     *
     * @return a command that sets the desired angle to the angle required for
     *         underhand launching into the speaker
     **/
    public Command setUnderhand() {
        return this.setAngle(Constants.Arm.Positions.UNDERHAND);
    }

    public Command setTuned() {
        return this.run(() -> this.motor.setControl(this.motionMagic.withPosition(SmartDashboard.getNumber("underhand", 0.0))));
    }

    public Command trackAngle(DoubleSupplier angleSupplier) {
        return this.run(() -> {
            // double roundedAngle = Math.toRadians(((int)
            // (Math.toDegrees(angleSupplier.getAsDouble()) * 10.0)) / 10.0);
            // this.motor.setControl(this.motionMagic.withPosition(roundedAngle));
            if (Math.abs(angleSupplier.getAsDouble() - this.motionMagic.Position) >= (Math.toRadians(0.5) / 2.0 * Math.PI)) {
                this.motor.setControl(this.motionMagic.withPosition(angleSupplier.getAsDouble()));
            }
        });
    }

    /**
     * Moves arm to throwing position for Amp
     *
     * @return a command that sets the desired angle to the angle required for
     *         launching into the amp
     **/
    public Command setAmp() {
        return this.setAngle(Constants.Arm.Positions.AMP);
    }

    public boolean isAmping() {
        return this.motionMagic.Position == Constants.Arm.Positions.AMP;
    }

    public void setAmpsSysID(double amps) {
        this.motor.setControl(new TorqueCurrentFOC(amps));
    }
}