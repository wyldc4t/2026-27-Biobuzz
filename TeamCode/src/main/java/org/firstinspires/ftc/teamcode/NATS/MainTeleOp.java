package org.firstinspires.ftc.teamcode.NATS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//6/11/26  11:04AM
;
@Config
@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends OpMode {

    private Turret turret;
    private Shooter shooter;
    private ShooterRegression regression;
    private DcMotorEx intake1, intake2;
    private Servo stopper, flower, pivotLeft, pivotRight;

    private boolean feedLatched = false;
    private double shootHoldUntil = 0;   // time (s) until which both intake motors keep running after a shot
    private double lastDistance = 0;
    private double lastX = 0, lastY = 0;
    private double rpmTrim = 0;
    private double commandedRpm = 0;
    private boolean flowerIsDown = false;
    private double flowerPos = RobotConfig.FLOWER_UP;
    private boolean prevUp, prevDown, prevLeft, prevL3;
    private boolean prevUp2, prevDown2;
    private double lastLoopTime = -1;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize mechanisms
        turret = new Turret();
        turret.hardwareMap = hardwareMap;
        turret.gamepad1 = gamepad1;
        turret.gamepad2 = gamepad2;
        turret.telemetry = telemetry;
        turret.updateTelemetry = false;
        turret.init();

        shooter = new Shooter();
        shooter.init(hardwareMap);
        regression = new ShooterRegression(RobotConfig.REGRESSION_DEGREE);

        intake1 = hardwareMap.get(DcMotorEx.class, RobotConfig.INTAKE_MOTOR_1);
        intake2 = hardwareMap.get(DcMotorEx.class, RobotConfig.INTAKE_MOTOR_2);
        intake1.setDirection(RobotConfig.INTAKE_MOTOR_1_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(RobotConfig.INTAKE_MOTOR_2_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setPower(0);
        intake2.setPower(0);

        stopper = hardwareMap.get(Servo.class, RobotConfig.STOP_SERVO);
        flower = hardwareMap.get(Servo.class, RobotConfig.FLOWER_SERVO);
        stopper.setPosition(RobotConfig.STOP_BLOCKING);
        flower.setPosition(RobotConfig.FLOWER_UP);
        pivotLeft = hardwareMap.tryGet(Servo.class, RobotConfig.PIVOT_SERVO_LEFT);
        pivotRight = hardwareMap.tryGet(Servo.class, RobotConfig.PIVOT_SERVO_RIGHT);
        setPivot(RobotConfig.PIVOT_UP);

        telemetry.addLine("Main TeleOp Initialized.");
        telemetry.update();
    }

    @Override
    public void start() {
        turret.start();
        lastLoopTime = System.nanoTime() / 1e9;
    }

    @Override
    public void loop() {
        double now = System.nanoTime() / 1e9;
        double dt = now - lastLoopTime;
        lastLoopTime = now;

        // ---- Turret & Drive Subsystem Loop ----
        turret.loop();

        // Distance to the goal the turret is tracking (BACK / right stick button), used by the RPM regression.
        if (turret.follower != null) {
            Pose pose = turret.follower.pose();
            lastDistance = Math.hypot(pose.x() - turret.getGoalX(), pose.y() - turret.getGoalY());
            lastX = pose.x();
            lastY = pose.y();
        }

        // ---- Shooter Control (always spinning) ----
        boolean autoRpm = RobotConfig.AUTO_RPM_FROM_DISTANCE;
        boolean up2 = gamepad2.dpad_up, down2 = gamepad2.dpad_down;
        if (up2 && !prevUp2) {
            if (autoRpm) rpmTrim += RobotConfig.RPM_STEP; else RobotConfig.SHOOTER_TARGET_RPM += RobotConfig.RPM_STEP;
        }
        if (down2 && !prevDown2) {
            if (autoRpm) rpmTrim -= RobotConfig.RPM_STEP; else RobotConfig.SHOOTER_TARGET_RPM = Math.max(0, RobotConfig.SHOOTER_TARGET_RPM - RobotConfig.RPM_STEP);
        }
        prevUp2 = up2; prevDown2 = down2;
        if (gamepad2.y) rpmTrim = 0;
        if (gamepad1.dpad_up && !prevUp) {
            if (autoRpm) rpmTrim += RobotConfig.RPM_STEP; else RobotConfig.SHOOTER_TARGET_RPM += RobotConfig.RPM_STEP;
        }
        if (gamepad1.dpad_down && !prevDown) {
            if (autoRpm) rpmTrim -= RobotConfig.RPM_STEP; else RobotConfig.SHOOTER_TARGET_RPM = Math.max(0, RobotConfig.SHOOTER_TARGET_RPM - RobotConfig.RPM_STEP);
        }
        if (regression.getRequestedDegree() != RobotConfig.REGRESSION_DEGREE) regression.refit(RobotConfig.REGRESSION_DEGREE);
        if (autoRpm) {
            commandedRpm = regression.predict(lastX, lastY, turret.getGoalX(), turret.getGoalY(), !turret.isRedAlliance(), shooter.getVoltage(), RobotConfig.SHOOTER_MIN_RPM, Double.MAX_VALUE) + rpmTrim;
            commandedRpm = Math.max(0, commandedRpm);
        } else {
            commandedRpm = RobotConfig.SHOOTER_TARGET_RPM;
        }
        // Controller 2 left stick press records the shot into the table
        if (gamepad2.left_stick_button && !prevL3) { regression.addSample(lastX, lastY, turret.getGoalX(), turret.getGoalY(), !turret.isRedAlliance(), commandedRpm, shooter.getVoltage()); }
        shooter.setTargetRpm(commandedRpm);
        shooter.update();

        // ---- Intake Control ----
        double nowShoot = System.nanoTime() / 1e9;

        // Gate the shot once per bumper press
        if (!gamepad1.right_bumper) feedLatched = false;
        else if (!feedLatched && (!RobotConfig.REQUIRE_AT_SPEED || shooter.atSpeed())) feedLatched = true;
        boolean shooting = gamepad1.right_bumper && feedLatched;
        shooter.setFeeding(shooting);

        if (shooting) shootHoldUntil = nowShoot + RobotConfig.SHOOT_STOP_DELAY_SEC;
        boolean shootTail = !shooting && nowShoot < shootHoldUntil;

        // Intake state machine
        double intakePower;
        if (gamepad1.a) {
            intakePower = RobotConfig.INTAKE_POWER;
        } else if (gamepad1.left_bumper) {
            intakePower = RobotConfig.OUTTAKE_POWER;
            shootHoldUntil = 0;
        } else if (gamepad1.right_trigger > 0.05 || gamepad1.left_trigger > 0.05) {
            intakePower = (gamepad1.right_trigger - gamepad1.left_trigger) * RobotConfig.INTAKE_POWER;
            if (!shooting) shootHoldUntil = 0;
        } else if (shooting || shootTail) {
            intakePower = RobotConfig.INTAKE_POWER;
        } else {
            intakePower = 0;
        }
        intake1.setPower(intakePower);
        intake2.setPower(intakePower);

        // ---- Stop Servo: open only while shooting ----
        stopper.setPosition(shooting || shootTail ? RobotConfig.STOP_DOWN : RobotConfig.STOP_BLOCKING);

        // ---- Flower Servo: controller 1 dpad-left toggles up <-> down ----
        if (gamepad1.dpad_left && !prevLeft) {
            flowerIsDown = !flowerIsDown;
            flowerPos = flowerIsDown ? RobotConfig.FLOWER_DOWN : RobotConfig.FLOWER_UP;
            flower.setPosition(flowerPos);
        }
        // Controller 2 bumpers fine-adjust the flower
        double flowerDir = turret.isCornerMode() ? 0 : (gamepad2.right_bumper ? 1 : 0) - (gamepad2.left_bumper ? 1 : 0);
        if (flowerDir != 0) {
            double lo = Math.min(RobotConfig.FLOWER_UP, RobotConfig.FLOWER_DOWN);
            double hi = Math.max(RobotConfig.FLOWER_UP, RobotConfig.FLOWER_DOWN);
            double upDirection = Math.signum(RobotConfig.FLOWER_UP - RobotConfig.FLOWER_DOWN);
            flowerPos = Math.max(lo, Math.min(hi, flowerPos + flowerDir * upDirection * RobotConfig.FLOWER_ADJUST_PER_SEC * dt));
            flower.setPosition(flowerPos);
        }

        // ---- Pivot: auto-managed by intake state (optional - can add button control) ----
        // Pivot is typically controlled in auto; in TeleOp it stays up

        prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down;
        prevLeft = gamepad1.dpad_left;
        prevL3 = gamepad2.left_stick_button;

        // ---- Additional Telemetry ----
        telemetry.addData("Shot X / Y / dist to goal (in)", "%.1f / %.1f / %.1f", lastX, lastY, lastDistance);
        telemetry.addData("RPM Fit", "deg %d, R2 %.3f, %s",
                regression.getDegreeUsed(), regression.getRSquared(), regression.describeSamples());
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.getRpm(), shooter.getTargetRpm());
        telemetry.addData("Battery (V)", "%.2f", shooter.getVoltage());
        telemetry.addData("Voltage effect", regression.describeVoltageEffect());
        telemetry.addData("Shooter At Speed", shooter.atSpeed());
        telemetry.addData("Dip boost", shooter.isRecovering());
        telemetry.addData("Intake motors", "%.2f / %.2f", intake1.getPower(), intake2.getPower());
        telemetry.addData("Stop / flower", "%.2f / %.2f", stopper.getPosition(), flower.getPosition());
        telemetry.addData("Loop MS", "%.1f", dt * 1000);
        telemetry.update();
    }

    private void setPivot(double position) {
        if (pivotLeft != null) pivotLeft.setPosition(position);
        if (pivotRight != null) {
            pivotRight.setPosition(RobotConfig.PIVOT_RIGHT_REVERSED ? 1.0 - position : position);
        }
    }

    @Override
    public void stop() {
        turret.stop();
        shooter.stop();
        intake1.setPower(0);
        intake2.setPower(0);
        stopper.setPosition(RobotConfig.STOP_BLOCKING);
        if (pivotLeft != null) pivotLeft.setPosition(RobotConfig.PIVOT_UP);
        if (pivotRight != null) pivotRight.setPosition(RobotConfig.PIVOT_RIGHT_REVERSED ? 1.0 - RobotConfig.PIVOT_UP : RobotConfig.PIVOT_UP);
    }
}
