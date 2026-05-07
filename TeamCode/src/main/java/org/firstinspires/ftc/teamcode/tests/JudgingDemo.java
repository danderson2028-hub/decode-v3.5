package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.graph.GraphManager;
//import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.Calculations;

import java.util.List;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.positionable.SetPositions;

@Configurable
@TeleOp
public class JudgingDemo extends NextFTCOpMode {
    public JudgingDemo(){
        addComponents(
                new SubsystemComponent(Lift.INSTANCE, Turret.INSTANCE, Limelight.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public Limelight3A limelight;
    public static double targetVelocity = 0.0; // ticks per second


    // ---- Hardware ----
    private MotorEx shooter;

    private MotorEx transfer;
    private MotorEx intake;


    public static boolean runTransfer = false;
    public static double transferPower = 0.5;
    public static double hoodOffset = 0.0;

    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(0.02, 0, 0.0)
            .basicFF(0.00056,0.0,0.0)
            .build();



    private ServoEx hood = new ServoEx("hood");


    @Override
    public void onInit() {
        panelsTelemetry.addLine("Data Collection opmode");
        panelsTelemetry.update();
        Turret.alignment=true;
        // Initialize motor
        shooter = new MotorEx("shooter");
        transfer = new MotorEx("transfer");
        intake = new MotorEx("intake");

        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(30);
        limelight.pipelineSwitch(1);
        limelight.start();

        // Build controller with velocity PID using live-updating coefficients


    }

    @Override
    public void onStartButtonPressed() {
        Gamepads.gamepad1().circle().whenBecomesTrue(Lift.INSTANCE.lift()).whenBecomesFalse(Lift.INSTANCE.holdLift());
        Gamepads.gamepad1().triangle().whenBecomesTrue(Lift.INSTANCE.retract()).whenBecomesFalse(Lift.INSTANCE.holdLift());

        panelsTelemetry.addLine("Shooter PID Tuner Started");
        panelsTelemetry.update();
    }

    @Override
    public void onUpdate() {
        LLResult result = limelight.getLatestResult();


        shooter.setPower(controlSystem.calculate(new KineticState(shooter.getCurrentPosition(),shooter.getVelocity())));

        if(result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                double ty = fr.getTargetYDegrees();
                double distance = 16 / (Math.tan(Math.toRadians(7.6026 + ty)));

                ActiveOpMode.telemetry().addData("ty", ty);
                ActiveOpMode.telemetry().addData("dist", distance);
                ActiveOpMode.telemetry().addData("target rpm", Calculations.getShooterRPM(distance));
                ActiveOpMode.telemetry().addData("hood offset", .63-Calculations.getHoodAngle(distance));

                controlSystem.setGoal(new KineticState(shooter.getCurrentPosition(),Calculations.getShooterRPM(distance)));

                Hood.INSTANCE.calculateAngle(distance).schedule();
                hood.to(Calculations.getHoodAngle(distance));

                //transferPower = Calculations.getTransferSpeed(distance);

                //ActiveOpMode.telemetry().addData("transfer power", transferPower);


            }
        }




        // Optional: only rebuild if values changed, or rebuild every loop for simpl

        //graphManager.update();
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void onStop() {
        shooter.setPower(0);
        panelsTelemetry.addLine("Shooter PID Tuner Stopped");
        panelsTelemetry.update();
    }
}