package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Double.POSITIVE_INFINITY;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.utils.Calculations;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter(){}

    private MotorEx shooter = new MotorEx("shooter");
    private ServoEx light = new ServoEx("light");

    public static boolean atTargetVelocity = false;


    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(0.02, 0, 0.0)
            .basicFF(0.00056,0.0,0.0)
            .build();
    public Command runFlywheelClose = new RunToVelocity(controlSystem, 1080,15).requires(this);
    public Command runFlywheelFar = new RunToVelocity(controlSystem, 1400).requires(this);

    public Command stopFlywheel = new RunToVelocity(controlSystem, 0).requires(this);
    public Command calculateFlywheel(double distance){
        return new RunToVelocity(controlSystem, Calculations.getShooterRPM(distance),15);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void periodic(){
        shooter.setPower(controlSystem.calculate(new KineticState(shooter.getCurrentPosition(),shooter.getVelocity())));
        atTargetVelocity = controlSystem.isWithinTolerance(new KineticState(POSITIVE_INFINITY, 25,POSITIVE_INFINITY));
        if(atTargetVelocity) light.setPosition(0.5);
        else light.setPosition(.611);
        ActiveOpMode.telemetry().addData("Flywheel Velocity", shooter.getVelocity());
        ActiveOpMode.telemetry().addData("RPM", shooter.getVelocity() * 60 / 28);
        ActiveOpMode.telemetry().addData("Shooter power", controlSystem.calculate(new KineticState(shooter.getCurrentPosition(),shooter.getVelocity())));

    }
}
