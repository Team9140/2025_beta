package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Seconds;

public class Yeeter extends SubsystemBase {
    private static Yeeter instance;

    public static Yeeter getInstance() {
        return instance == null ? instance = new Yeeter() : instance;
    }

    private final WPI_TalonSRX feeder = new WPI_TalonSRX(Constants.Ports.THROWER_FEEDER);

    private double feederVolts;

    private final TalonFX leftRollers = new TalonFX(6, Constants.Ports.CTRE_CANBUS);
    private final TalonFX rightRollers = new TalonFX(5, Constants.Ports.CTRE_CANBUS);

    private final VelocityVoltage leftSpeed;
    private final VelocityVoltage rightSpeed;

    private final SlewRateLimiter leftSlew;
    private final SlewRateLimiter rightSlew;

    private double leftSpeedTarget;
    private double rightSpeedTarget;

    private final VoltageOut SysIDController = new VoltageOut(0.0).withEnableFOC(true);

    public SysIdRoutine YeeterRoutine = new SysIdRoutine(new SysIdRoutine.Config(
            null, Units.Volts.of(4.0), null, (state) -> SignalLogger.writeString("yeeter-state", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> this.SysIDController.withOutput(volts.magnitude()), null, this));

    private Yeeter() {

        CurrentLimitsConfigs launcherCurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Thrower.Launcher.MAX_CURRENT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLowerTime(Seconds.of(1.0))
                .withSupplyCurrentLimit(40.0)
                .withSupplyCurrentLimitEnable(true);

        Slot0Configs configs = new Slot0Configs()
                .withKP(0.2)
                .withKV(0.13)
                .withKS(0.2)
                .withKA(0.00636);

        TalonFXConfiguration launcherConfiguration = new TalonFXConfiguration()
                .withCurrentLimits(launcherCurrentLimits)
                .withSlot0(configs);

        this.leftRollers.getConfigurator().apply(launcherConfiguration
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)));
        this.rightRollers.getConfigurator().apply(launcherConfiguration
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));

        this.leftSpeed = new VelocityVoltage(0.0).withEnableFOC(true);
        this.rightSpeed = new VelocityVoltage(0.0).withEnableFOC(true);

        double slewRate = 12000.0 / 60.0 / 0.6; // zero to full speed in half second
        this.leftSlew = new SlewRateLimiter(slewRate);
        this.rightSlew = new SlewRateLimiter(slewRate);

        this.feeder.setInverted(true);
        this.feeder.configContinuousCurrentLimit(Constants.Thrower.Feeder.MAX_CURRENT);
        this.feeder.configPeakCurrentLimit(30);
        this.feeder.configPeakCurrentDuration(500);
        this.feeder.enableCurrentLimit(true);
    }

    @Override
    public void periodic() {
        // this.leftRollers.setControl(SysIDController);
        // this.rightRollers.setControl(SysIDController);
        if (this.leftSpeedTarget != 0.0) {
            this.leftRollers.setControl(this.leftSpeed.withVelocity(this.leftSlew.calculate(this.leftSpeedTarget)));
        } else {
            this.leftRollers.setControl(new VoltageOut(0.0));
        }

        if (this.rightSpeedTarget != 0.0) {
            this.rightRollers.setControl(this.rightSpeed.withVelocity(this.rightSlew.calculate(this.rightSpeedTarget)));
        } else {
            this.rightRollers.setControl(new VoltageOut(0.0));
        }
        
        
        this.feeder.setVoltage(this.feederVolts);

        // SmartDashboard.putNumber("left roller RPS", this.leftRollers.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("right roller RPS", this.rightRollers.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("left roller RPS target", this.leftSpeedTarget);
        // SmartDashboard.putNumber("right roller RPS target", this.rightSpeedTarget);
        // SmartDashboard.putNumber("left roller amps", this.leftRollers.getTorqueCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("right roller amps", this.rightRollers.getTorqueCurrent().getValueAsDouble());
    }

    public double getFeederCurrent() {
        return this.feeder.getSupplyCurrent();
    }

    public Command setIntake() {
        return this.runOnce(() -> {
            this.feederVolts = Constants.Thrower.Feeder.INTAKE_VOLTAGE;
            this.leftSpeedTarget = Constants.Yeeter.INTAKE_RPS;
            this.rightSpeedTarget = Constants.Yeeter.INTAKE_RPS;
            // this.leftSpeed.withVelocity(Constants.Yeeter.INTAKE_RPS);
            // this.rightSpeed.withVelocity(Constants.Yeeter.INTAKE_RPS);
        });
    }

    public Command hold() {
        return this.runOnce(() -> {
            this.feederVolts = Constants.Thrower.Feeder.PREPARE_VOLTAGE;
            this.leftSpeedTarget = 0.0;
            this.rightSpeedTarget = 0.0;
            // this.leftSpeed.withVelocity(0.0);
            // this.rightSpeed.withVelocity(0.0);
        });
    }

    public Command prepareSpeaker() {
        return this.runOnce(() -> {
            this.feederVolts = Constants.Thrower.Feeder.PREPARE_VOLTAGE;
            this.leftSpeedTarget = Constants.Yeeter.LEFT_SPEAKER_RPS;
            this.rightSpeedTarget = Constants.Yeeter.RIGHT_SPEAKER_RPS;
            // this.leftSpeed.withVelocity(Constants.Yeeter.LEFT_SPEAKER_RPS);
            // this.rightSpeed.withVelocity(Constants.Yeeter.RIGHT_SPEAKER_RPS);
        });
    }

    public Command prepareAmp() {
        return this.runOnce(() -> {
            this.feederVolts = Constants.Thrower.Feeder.PREPARE_VOLTAGE;
            this.leftSpeedTarget = Constants.Yeeter.AMP_RPS;
            this.rightSpeedTarget = Constants.Yeeter.AMP_RPS;
            // this.leftSpeed.withVelocity(Constants.Yeeter.AMP_RPS);
            // this.rightSpeed.withVelocity(Constants.Yeeter.AMP_RPS);
        });
    }

    public Command launch() {
        return this.runOnce(() -> {
            this.feederVolts = Constants.Thrower.Feeder.LAUNCH_VOLTAGE;
        });
    }

    public Command off() {
        return this.runOnce(() -> {
            this.feederVolts = 0.0;
            this.leftSpeedTarget = 0.0;
            this.rightSpeedTarget = 0.0;
        });
    }

    public Command eject() {
        return this.runOnce(() -> {
            this.feederVolts = 12.0;
            this.leftSpeedTarget = Constants.Yeeter.AMP_RPS;
            this.rightSpeedTarget = Constants.Yeeter.AMP_RPS;
        });
    }

    private final Debouncer noty = new Debouncer(0.25);

    public final Trigger hasNote = new Trigger(
            () -> noty.calculate(Math.abs(this.feeder.getStatorCurrent()) > Constants.INTAKE_NOTIFY_CURRENT)
                    && this.feederVolts == Constants.Thrower.Feeder.INTAKE_VOLTAGE);

    private static final double THRESHOLD = 5.0;
    public final Trigger ready = new Trigger(
            () -> Math.abs(leftRollers.getClosedLoopError().getValueAsDouble()) <= THRESHOLD
                    && Math.abs(rightRollers.getClosedLoopError().getValueAsDouble()) <= THRESHOLD
                    && leftSpeedTarget >= Constants.Yeeter.AMP_RPS
                    && rightSpeedTarget >= Constants.Yeeter.AMP_RPS);
}
