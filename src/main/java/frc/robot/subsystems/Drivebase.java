package frc.robot.subsystems;

import frc.robot.OI;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

//Subsystem imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Import arrays to list motors b
import java.util.ArrayList;

public class Drivebase extends SubsystemBase {
    public OI oi;
    ArrayList<TalonFX> motorList;
    ArrayList<TalonFX> driverMotors;

    public void DriveBase(){
        //Add motors
        addMotors();
    }

    protected void addMotors(){
        //For Falcon motors
        driverMotors = new ArrayList<>();
        driverMotors.add(OI.testMotor);
    }

    @Override
    public void periodic(){
    }

    public static double maxSpeed(){

        double accelCurveValue = 0.05;
        double accelCurveStart = 0.05;
        double accelCurveFactor = 40;

        while(accelCurveValue <= 0.4){
            accelCurveValue = accelCurveValue * accelCurveStart * accelCurveFactor;
        } return accelCurveValue;
    }
}
