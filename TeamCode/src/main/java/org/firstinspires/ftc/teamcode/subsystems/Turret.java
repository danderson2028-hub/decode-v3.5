package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utils.Calculations.getTurretAngle;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.opmodes.Teleop;
import org.firstinspires.ftc.teamcode.utils.Calculations;

import java.util.List;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.FeedbackCRServoEx;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();

    private Turret() {}

    public FeedbackCRServoEx turretOne = new FeedbackCRServoEx(
            0.02,
            () -> ActiveOpMode.hardwareMap().analogInput.get("analog"),
            () -> ActiveOpMode.hardwareMap().crservo.get("turret")
    );

    public CRServoEx turretTwo = new CRServoEx("turret");

    public static boolean alignment = false;
    public static double sotmAngularOffset = 0.0;

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.03, 0.0, 0.0)
            .basicFF(0, 0, 0.05)
            .build();

    private enum AlignmentMode { Limelight, Odometry, OFF }
    public static AlignmentMode state = AlignmentMode.Odometry;

    // ---- Encoder tracking (all in SERVO degrees internally) ----
    private double servoPosRadians = 0.0;
    private double previousAngle = 0.0;

    // Servo degrees per turret degree
    private static final double SERVO_TO_TURRET = 2.85;
    // Max turret rotation in servo degrees (±94.7 real turret degrees for now)
    private static final double SERVO_DEG_LIMIT = 270.0;

    @Override
    public void initialize() {
        resetEncoder();
    }

    public void resetEncoder() {
        previousAngle = turretOne.getCurrentPosition();
        servoPosRadians = 0.0;
    }

    public void updatePosition() {
        double currentAngle = turretOne.getCurrentPosition();
        double deltaAngle = currentAngle - previousAngle;

        if (deltaAngle > Math.PI) deltaAngle -= 2 * Math.PI;
        else if (deltaAngle < -Math.PI) deltaAngle += 2 * Math.PI;

        servoPosRadians += deltaAngle;
        previousAngle = currentAngle;
    }

    /** Raw servo position in degrees. Use this for PID feedback. */
    public double getServoPosDeg() {
        return Math.toDegrees(servoPosRadians);
    }

    /** Actual turret angle in real-world degrees. Use this for telemetry only. */
    public double getTurretAngleDeg() {
        return getServoPosDeg() / SERVO_TO_TURRET;
    }

    public void setCurrentPosition(double turretDeg) {
        // Accept real turret degrees, store as radians of servo position
        servoPosRadians = Math.toRadians(turretDeg * SERVO_TO_TURRET);
        previousAngle = turretOne.getCurrentPosition();
    }

    /** Takes a position in real turret degrees */
    public Command runTurretToPosition(double turretDeg) {
        double clampedServoDeg = MathFunctions.clamp(
                turretDeg * SERVO_TO_TURRET,
                -SERVO_DEG_LIMIT,
                SERVO_DEG_LIMIT
        );
        return new RunToPosition(controlSystem, clampedServoDeg, 30).requires(this);
    }

    // Telemetry targets for logging
    private double llTargetServoDeg = 0.0;
    private double odomTargetServoDeg = 0.0;

    @Override
    public void periodic() {
        updatePosition();

        double currentServoDeg = getServoPosDeg(); // PID feedback unit

        LLResult result = Limelight.INSTANCE.getLatestResult();

        // Determine alignment mode
        if (!alignment) {
            state = AlignmentMode.OFF;
        } else if (result != null && result.isValid()
                && Math.abs(result.getFiducialResults().get(0).getTargetXDegrees()) < 10) {
            state = AlignmentMode.Limelight;
        } else {
            state = AlignmentMode.Odometry;
        }

        switch (state) {
            case Limelight:
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        // adjustedTx is in real degrees, convert to servo degrees before adding
                        double txServoDeg = fr.getTargetXDegrees() * SERVO_TO_TURRET;
                        llTargetServoDeg = MathFunctions.clamp(
                                currentServoDeg + txServoDeg,
                                -SERVO_DEG_LIMIT,
                                SERVO_DEG_LIMIT
                        );
                        if (Math.abs(fr.getTargetXDegrees()) > 2) {
                            controlSystem.setGoal(new KineticState(
                                    llTargetServoDeg,
                                    Math.signum(llTargetServoDeg - currentServoDeg)
                            ));
                        }
                    }
                } else {
                    // Hold position
                    controlSystem.setGoal(new KineticState(currentServoDeg, 0));
                }
                break;

            case Odometry:
                // getTurretAngle() already returns servo degrees
                odomTargetServoDeg = getTurretAngle(Teleop.getFollower().getPose());
                controlSystem.setGoal(new KineticState(
                        odomTargetServoDeg,
                        Math.signum(odomTargetServoDeg - currentServoDeg)
                ));
                break;

            case OFF:
                break;
        }

        // Telemetry
        ActiveOpMode.telemetry().addData("Align mode", state);
        ActiveOpMode.telemetry().addData("Auto Align", alignment);
        ActiveOpMode.telemetry().addData("LL target (servo deg)", llTargetServoDeg);
        ActiveOpMode.telemetry().addData("Odom target (servo deg)", odomTargetServoDeg);
        ActiveOpMode.telemetry().addData("Current (servo deg)", currentServoDeg);
        ActiveOpMode.telemetry().addData("Current (turret deg)", getTurretAngleDeg());

        // PID feedback in servo degrees, matching goals
        double power = controlSystem.calculate(
                new KineticState(currentServoDeg, turretOne.getVelocity())
        );
        turretOne.setPower(power);
        turretTwo.setPower(-power);
    }
}