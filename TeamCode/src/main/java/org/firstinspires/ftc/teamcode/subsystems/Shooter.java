package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.commands.Calculations;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
@Configurable
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter(){}

    private MotorEx shooter = new MotorEx("shooter");




    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(0.02, 0, 0.0)
            .basicFF(0.00056,0.0,0.0)
            .build();
    public Command runFlywheel = new RunToVelocity(controlSystem, 1100).requires(this);
    public Command runFlywheelFar = new RunToVelocity(controlSystem, 1500).requires(this);

    public Command stopFlywheel = new RunToVelocity(controlSystem, 0).requires(this);
    public Command calculateFlywheel(double distance){
        return new RunToVelocity(controlSystem, Calculations.getShooterRPM(distance));
    }

    @Override
    public void initialize(){

    }

    @Override
    public void periodic(){
        shooter.setPower(controlSystem.calculate(new KineticState(shooter.getCurrentPosition(),shooter.getVelocity())));


        ActiveOpMode.telemetry().addData("Flywheel Velocity", shooter.getVelocity());
        ActiveOpMode.telemetry().addData("RPM", shooter.getVelocity() * 60 / 28);
        ActiveOpMode.telemetry().addData("Shooter power", controlSystem.calculate(new KineticState(shooter.getCurrentPosition(),shooter.getVelocity())));

    }
}
