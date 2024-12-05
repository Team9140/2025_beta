package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Cantdle extends SubsystemBase {

    public static final Color8Bit RED = new Color8Bit(255, 0, 0);
    public static final Color8Bit GREEN = new Color8Bit(0, 255, 0);
    public static final Color8Bit BLUE = new Color8Bit(0, 0, 255);
    public static final Color8Bit ORANGE = new Color8Bit(255, 157, 0);
    public static final Color8Bit PURPLE = new Color8Bit(151, 0, 180);

    private CANdle candle;
    private static Cantdle instance;

    private Cantdle() {
        this.candle = new CANdle(Constants.Ports.CANDLE, "jama");
        this.candle.setLEDs(0, 0, 0);
    }

    public static Cantdle getInstance() {
        return instance == null ? instance = new Cantdle() : instance;
    }

    public Command solidAllianceColor() {
        return new ConditionalCommand(this.setColor(RED), this.setColor(BLUE),
                () -> DriverStation.getAlliance().isPresent()
                        && Alliance.Red.equals(DriverStation.getAlliance().get()));
    }

    public Command setColor(Color8Bit c) {
        return this.runOnce(() -> this.candle.setLEDs(c.red, c.green, c.blue));
    }

    public Command off() {
        return this.setColor(new Color8Bit(0, 0, 0));
    }

    public Command flashColor(Color8Bit c, double seconds) {
        return this.setColor(c).andThen(Commands.waitSeconds(seconds)).andThen(this.solidAllianceColor());
    }

    public Command blinkColor(Color8Bit c, double seconds) {
        return this.setColor(c).andThen(Commands.waitSeconds(0.1)).andThen(this.off())
                .andThen(Commands.waitSeconds(0.1)).repeatedly().withTimeout(seconds);
    }
}