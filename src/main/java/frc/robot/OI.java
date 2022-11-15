package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Drivebase;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class OI {
    public static Joystick gamepad = new Joystick(Constants.GAMEPAD);
    public static Joystick joystick = new Joystick(Constants.JOYSTICK);

    public static TalonFX testMotor = new TalonFX(Constants.TESTMOTOR);

    public OI(){
        Drivebase drivebase = new Drivebase();
    }
}