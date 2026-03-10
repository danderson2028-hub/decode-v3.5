package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.Calculations;
import org.firstinspires.ftc.teamcode.utils.Data;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp (name = "SOTM Test")
public class SOTMTest extends NextFTCOpMode {
    public SOTMTest(){
        addComponents(
                new SubsystemComponent(
                        Shooter.INSTANCE,
                        Intake.INSTANCE,
                        Transfer.INSTANCE,
                        Turret.INSTANCE,
                        Hood.INSTANCE,
                        Limelight.INSTANCE,
                        Lift.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );

    }

    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;

    private boolean runShooter = false;
    public double transferPower = 0.5;


    public static Follower getFollower(){
        return PedroComponent.follower();
    }
    @Override
    public void onStartButtonPressed() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().rightStickY().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                Gamepads.gamepad1().leftStickX().map(value -> 0.5*value).negate()
        );
        Turret.alignment=true;
        driverControlled.schedule();
        Lift.INSTANCE.holdPlate().schedule();
        Gamepads.gamepad1().circle().whenBecomesTrue(Intake.INSTANCE.runIntake);
        Gamepads.gamepad1().square().whenBecomesTrue(Intake.INSTANCE.runIntakeReverse);
        Gamepads.gamepad1().dpadRight().whenBecomesTrue(Intake.INSTANCE.stopIntake);
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(Transfer.INSTANCE.runTransfer(0.4)).whenBecomesFalse(Transfer.INSTANCE.runTransfer(0.0));

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(Transfer.INSTANCE.runTransfer(0.7)).whenBecomesFalse(Transfer.INSTANCE.runTransfer(0.0));
        //Gamepads.gamepad1().dpadUp().whenBecomesTrue(Shooter.INSTANCE.runFlywheelClose.and(Hood.INSTANCE.close));
        //Gamepads.gamepad1().dpadDown().whenBecomesTrue(Shooter.INSTANCE.runFlywheelFar.and(Hood.INSTANCE.up));
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(Transfer.INSTANCE.runTransfer(1.0)).whenBecomesFalse(Transfer.INSTANCE.runTransfer(0.0));
        Gamepads.gamepad2().circle().whenBecomesTrue(Lift.INSTANCE.lift()).whenBecomesFalse(Lift.INSTANCE.holdLift());

    }
    @Override
    public void onInit(){
        CommandManager.INSTANCE.cancelAll();

        frontLeftMotor = new MotorEx("FL").reversed();
        frontRightMotor = new MotorEx("FR");
        backLeftMotor = new MotorEx("BL").reversed();
        backRightMotor = new MotorEx("BR");

        Calculations.goalPose=Calculations.blueGoalPose;
        if (Data.endPose != null) {
            PedroComponent.follower().setStartingPose(Data.endPose);
            telemetry.addData("X", Data.endPose.getX());
            telemetry.addData("Y", Data.endPose.getY());
            telemetry.addData("heading", Data.endPose.getHeading());
        } else {
            PedroComponent.follower().setStartingPose(new Pose(72,72,Math.toRadians(90)));
        }

        telemetry.update();
        //Limelight.INSTANCE.limelight.pipelineSwitch(1);
    }
    @Override
    public void onUpdate(){
        LLResult result = Limelight.INSTANCE.getLatestResult();
        if(gamepad1.dpad_up) {
            runShooter = true;
            Turret.alignment = true;
        }
        if(gamepad1.dpad_down) {
            runShooter = false;
            Turret.alignment = false;
        }
        //if(gamepad2.cross){
        //    PedroComponent.follower().setPose(new Pose(135,9,Math.toRadians(90)));
        //}
        //if(gamepad1.right_bumper) Transfer.INSTANCE.runTransfer(transferPower).schedule();
        //else Transfer.INSTANCE.runTransfer(0.0).schedule();


        if(result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                double ty = fr.getTargetYDegrees();
                double distance = 16 / (Math.tan(Math.toRadians(7.6026 + ty)));

                Pose robotPose = PedroComponent.follower().getPose();
                double fieldVx = PedroComponent.follower().getVelocity().getXComponent();
                double fieldVy = PedroComponent.follower().getVelocity().getYComponent();
                double speed   = Math.hypot(fieldVx, fieldVy);

                double adjustedDistance = distance;
                Turret.sotmAngularOffset = 0.0; // reset each loop

                if (speed > 0.1) {
                    // --- Unit vector from robot toward goal ---
                    double dx   = Calculations.goalPose.getX() - robotPose.getX();
                    double dy   = Calculations.goalPose.getY() - robotPose.getY();
                    double norm = Math.hypot(dx, dy);
                    double toGoalX = dx / norm;
                    double toGoalY = dy / norm;

                    // --- Decompose velocity ---
                    double radialVel  =  fieldVx * toGoalX   + fieldVy * toGoalY;
                    double lateralVel =  fieldVx * (-toGoalY) + fieldVy * toGoalX;

                    // --- Time of flight ---
                    double tof = Calculations.getFlightTime(distance);

                    // --- Adjusted inputs ---
                    adjustedDistance = distance + radialVel * tof;
                    Turret.sotmAngularOffset = Math.max(-10, Math.min(10, 2*Math.toDegrees(Math.atan2(lateralVel * tof, distance)))); // 15 degree max offset at max speed
                }

                // --- Telemetry ---
                ActiveOpMode.telemetry().addData("ty", ty);
                ActiveOpMode.telemetry().addData("dist (raw)", distance);
                ActiveOpMode.telemetry().addData("dist (adjusted)", adjustedDistance);
                ActiveOpMode.telemetry().addData("sotm angle offset", Turret.sotmAngularOffset);
                ActiveOpMode.telemetry().addData("target rpm", Calculations.getShooterRPM(adjustedDistance));
                ActiveOpMode.telemetry().addData("hood offset", 0.63 - Calculations.getHoodAngle(adjustedDistance));


                if(runShooter){
                    Shooter.INSTANCE.calculateFlywheel(adjustedDistance).schedule();
                    Hood.INSTANCE.calculateAngle(adjustedDistance).schedule();
                }
                else{
                    Shooter.INSTANCE.stopFlywheel.schedule();
                }


                //transferPower = Calculations.getTransferSpeed(distance);

                //ActiveOpMode.telemetry().addData("transfer power", transferPower);


            }
        }

        //telemetry.addData("distance", Math.hypot(Calculations.redGoalPose.getY()-PedroComponent.follower().getPose().getY(),Calculations.redGoalPose.getX()-PedroComponent.follower().getPose().getX()));
        telemetry.addData("x y heading", PedroComponent.follower().getPose().getX() + ", "+ PedroComponent.follower().getPose().getY()+ ", "+ PedroComponent.follower().getPose().getHeading());
        telemetry.addData("Commands", CommandManager.INSTANCE.snapshot());
        telemetry.update();
    }
    @Override
    public void onStop(){
        Turret.alignment=false;
        CommandManager.INSTANCE.cancelAll();
        Shooter.INSTANCE.stopFlywheel.schedule();
        //Poses.endPose=PedroComponent.follower().getPose();
    }
}
