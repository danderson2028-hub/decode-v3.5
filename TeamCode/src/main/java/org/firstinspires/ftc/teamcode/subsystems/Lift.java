package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.FeedbackCRServoEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Lift implements Subsystem {
    public static final Lift INSTANCE = new Lift();
    private Lift(){}

    private FeedbackCRServoEx liftLeft = new FeedbackCRServoEx(0.01,() -> ActiveOpMode.hardwareMap().analogInput.get("lift1"), () -> ActiveOpMode.hardwareMap().crservo.get("liftL"));

    private CRServoEx liftRight = new CRServoEx("liftR");
    private double liftTarget = -38.5;
    double totalAngle = 0.0; // This is your angle of the servo
    double previousAngle = 0.0;



    private ControlSystem controller = ControlSystem.builder()
            .posPid(0)
            .elevatorFF()
            .build();

    @Override
    public void initialize(){
        totalAngle=0.0;
        previousAngle= 0.0;
    }

    public Command holdPlate(){
        return new ParallelGroup(
                new SetPower(liftLeft, 0.0),
                new SetPower(liftRight, -0.0)
        );
    }
    public Command holdLift(){
        return new ParallelGroup(
                new SetPower(liftLeft, -0.175),
                new SetPower(liftRight, 0.175)
        );
    }
    public Command lift(){
        return new ParallelGroup(
                new SetPower(liftLeft, -0.9),
                new SetPower(liftRight, 0.9)
        );
    }
    public Command retract(){
        return new ParallelGroup(
                new SetPower(liftLeft, .05),
                new SetPower(liftRight, -.05)
        );
    }


    @Override
    public void periodic(){
        double currentAngle = liftLeft.getCurrentPosition();
        double deltaAngle = currentAngle - previousAngle;

        if (deltaAngle > Math.PI) deltaAngle -= 2 * Math.PI;
        else if (deltaAngle < -Math.PI) deltaAngle += 2 * Math.PI;

        totalAngle += deltaAngle;
        previousAngle = currentAngle;
        ActiveOpMode.telemetry().addLine("lift left power"+ liftLeft.getPower()+"lift right power"+ liftRight.getPower());
        ActiveOpMode.telemetry().addData("angle",totalAngle);
    }
}
