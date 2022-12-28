package org.firstinspires.ftc.teamcode.drive.TeleOp;
import android.os.Debug;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "Driving")
public class AndreiTeleOp extends LinearOpMode {
        int PlatePos = 0, LiftPos1 = 0, LiftPos2 = 0;
        double LiftPower = 1f;
        boolean StartControl = false, lastPressedStart = false;
        SampleMecanumDrive mecanumDrive;
<<<<<<< Updated upstream
//        org.firstinspires.ftc.teamcode.autonom.AutoUtil util;
        DcMotorEx liftMotor1,liftMotor2, plateMotor;
=======
        DcMotorEx liftMotor1, liftMotor2, plateMotor;
>>>>>>> Stashed changes
        Servo catcher;

        private void initialization() {
            liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
            liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
            plateMotor = hardwareMap.get(DcMotorEx.class, "plateMotor");
            catcher = hardwareMap.get(Servo.class, "catcherServo");
            catcher.setPosition(0);

            liftMotor1.setDirection(DcMotor.Direction.REVERSE);
            liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            liftMotor2.setDirection(DcMotor.Direction.REVERSE);
            liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            plateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            plateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            mecanumDrive = new SampleMecanumDrive(hardwareMap);
            mecanumDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
            mecanumDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        private void setPlateLevel() {
            if (gamepad2.left_bumper) {
                PlatePos = -1000;
                plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                plateMotor.setTargetPosition(PlatePos);
            } else if (gamepad2.right_bumper) {
                PlatePos = 1000;
                plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                plateMotor.setTargetPosition(PlatePos);
            } else if (gamepad2.dpad_up) {
                PlatePos = 0;
                plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                plateMotor.setTargetPosition(PlatePos);
            }
        }
        private void controlWheels() {
            Pose2d poseEstimate = mecanumDrive.getPoseEstimate();
            boolean pressedStart = gamepad1.start;
            if (pressedStart && !lastPressedStart) StartControl = !StartControl;
            lastPressedStart = pressedStart;
            if (StartControl == true) {
                Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).rotated(-poseEstimate.getHeading());
                mecanumDrive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -(gamepad1.right_trigger - gamepad1.left_trigger)));
            } else {
                Vector2d input = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x).rotated(-poseEstimate.getHeading());
                mecanumDrive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -(gamepad1.right_trigger - gamepad1.left_trigger)));
            }

        }
        private void controlArm() {
            if (gamepad2.left_stick_y != 0) {
                LiftPos1 = liftMotor1.getCurrentPosition();
                LiftPos2 = liftMotor2.getCurrentPosition();

                liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotor1.setPower(1f * gamepad2.left_stick_y);

                liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotor2.setPower(1f * gamepad2.left_stick_y);
            } else {
                if (LiftPos1 < -10 || LiftPos2 < -10)
                    LiftPower = 1f;
                else
                    LiftPower = 0f;

                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor1.setTargetPosition(LiftPos1);
                liftMotor1.setPower(LiftPower);

                liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor2.setTargetPosition(LiftPos2);
                liftMotor2.setPower(LiftPower);
            }
            if (gamepad2.right_trigger != 0 || gamepad2.left_trigger != 0) {
                PlatePos = plateMotor.getCurrentPosition();
                plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                plateMotor.setPower (1f * (gamepad2.right_trigger - gamepad2.left_trigger));
            } else {
                plateMotor.setTargetPosition(PlatePos);
                plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                plateMotor.setPower(1f);
            }
        }
        private void controlCatcher() {
            if (gamepad1.dpad_left) catcher.setPosition(0);
            else if (gamepad1.dpad_right) catcher.setPosition(.4f);
        }
        private void debugTelemetry() {
            telemetry.addData("LiftPos1: ", liftMotor1.getCurrentPosition());
            telemetry.addData("LiftPos2: ", liftMotor2.getCurrentPosition());
            telemetry.addData("PlatePos: ", plateMotor.getCurrentPosition());
            telemetry.addData("Claw: ", catcher.getPosition());
            telemetry.update();
        }

        @Override
        public void runOpMode() throws InterruptedException {
            initialization();
            waitForStart();
            while (opModeIsActive() && !isStopRequested()) {
                setPlateLevel();
                controlWheels();
                controlArm();
                controlCatcher();
                CustomLift();
                debugTelemetry();
            }
        }

        private void CustomLift() { // ! ground junction
            if (gamepad2.y) { // high junction
                LiftPos1 = -1765;
                LiftPos2 = -1755;
                LiftPower = 1f;
            } else if (gamepad2.x) { // mid junction
                LiftPos1 = -1255;
                LiftPos2 = -1245;
                LiftPower = 1f;
            } else if (gamepad2.b) { // low junction
                LiftPos1 = -745;
                LiftPos2 = -735;
                LiftPower = 1f;
            } else if (gamepad2.a) { // off
                PlatePos = 0;
                plateMotor.setTargetPosition(0);
                plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                LiftPos1 = 5;
                LiftPos2 = 5;
                LiftPower = 0f;
            }

            liftMotor1.setTargetPosition(LiftPos1);
            liftMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor1.setPower(LiftPower);

            liftMotor2.setTargetPosition(LiftPos2);
            liftMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor2.setPower(LiftPower);
        }
}

//        private void controlArm(){
//            if(gamepad2.left_stick_y!=0){
//                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                cp = liftMotor.getCurrentPosition();
//                liftMotor.setPower( 1f * gamepad2.left_stick_y);
//            }
//            else{
//                liftMotor.setTargetPosition(cp);
//                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftMotor.setPower(1f);
//            }
//            if (gamepad2.right_trigger!=0 || gamepad2.left_trigger!=0){
//                plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                pp = plateMotor.getCurrentPosition();
//                plateMotor.setPower ( 1f * (gamepad2.right_trigger-gamepad2.left_trigger));
//            }
//            else {
//                plateMotor.setTargetPosition(pp);
//                plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                plateMotor.setPower(1f);
//            }
//        }
//        private void setLiftLevel(){
//            if (gamepad2.a) {
//                pp = 0;
//                plateMotor.setTargetPosition(0);
//                plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                cp1 = 5;
//                liftMotor1.setTargetPosition(cp1);
//                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftMotor1.setPower(1f);
//                cp2 = 5;
//                liftMotor2.setTargetPosition(cp2);
//                liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftMotor2.setPower(1f);
//            } else if (gamepad2.b) {
//                cp1 = -732;
//                liftMotor1.setTargetPosition(cp1);
//                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftMotor1.setPower(1f);
//                cp2 = -732;
//                liftMotor2.setTargetPosition(cp2);
//                liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftMotor2.setPower(1f);
//            } else if (gamepad2.x) {
//                cp1 = -1235;
//                liftMotor1.setTargetPosition(cp1);
//                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftMotor1.setPower(1f);
//                cp2 = -1234;
//                liftMotor2.setTargetPosition(cp2);
//                liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftMotor2.setPower(1f);
//            } else if (gamepad2.y) {
//                cp1 = -1740;
//                liftMotor1.setTargetPosition(cp1);
//                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftMotor1.setPower(1f);
//                cp2 = -1737;
//                liftMotor2.setTargetPosition(cp2);
//                liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftMotor2.setPower(1f);
//            }
//        }
//        private void suppressWheels() {
//            if(gamepad1.right_bumper)
//                suppress1 = 0.5f;
//            else
//                suppress1 = 1f;
//        }
//        int color;
//        private void colorDet() {
//            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
//            colors.red   /= max;
//            colors.green /= max;
//            colors.blue  /= max;
//            color = colors.toColor();
//        }