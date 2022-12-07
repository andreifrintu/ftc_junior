package org.firstinspires.ftc.teamcode.drive.TeleOp;
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

        SampleMecanumDrive mecanumDrive;
        // AutoUtil util;
        DcMotorEx liftMotor1,liftMotor2, plateMotor;
        Servo catcher;
        NormalizedColorSensor sensor;
        double suppress1;
        NormalizedRGBA colors;
        boolean lr = false;

        @Override
        public void runOpMode() throws InterruptedException {
            initialization();
            waitForStart();
            while (opModeIsActive() && !isStopRequested()) {
                run();
            }
        }
        private void initialization(){
            liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
            liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
            plateMotor = hardwareMap.get(DcMotorEx.class, "plateMotor");
            catcher = hardwareMap.get(Servo.class, "catcherServo");
            sensor = hardwareMap.get(NormalizedColorSensor.class, "sensor");
            colors = sensor.getNormalizedColors();
            catcher.setPosition(0);
            liftMotor1.setDirection(DcMotor.Direction.REVERSE);
            liftMotor2.setDirection(DcMotor.Direction.REVERSE);
            liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            plateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            plateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            mecanumDrive = new SampleMecanumDrive(hardwareMap);
            mecanumDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
            mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mecanumDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }


        boolean lastPressedStart = false;
        private void controlWheels(){
            Pose2d poseEstimate = mecanumDrive.getPoseEstimate();
            boolean pressedStart = gamepad1.start;
            if (pressedStart&&!lastPressedStart) lr = !lr;
            lastPressedStart = pressedStart;
            if (lr) {
                Vector2d input = new Vector2d(
                        -gamepad1.left_stick_y*suppress1,
                        -gamepad1.left_stick_x*suppress1
                ).rotated(-poseEstimate.getHeading());
                mecanumDrive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -(gamepad1.right_trigger-gamepad1.left_trigger)*suppress1
                        )
                );
            }
            else {
                Vector2d input = new Vector2d(
                        -gamepad1.right_stick_y*suppress1,
                        -gamepad1.right_stick_x*suppress1
                ).rotated(-poseEstimate.getHeading());
                mecanumDrive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -(gamepad1.right_trigger-gamepad1.left_trigger)*suppress1
                        )
                );
            }

        }

        int cp1 = 0;
        int cp2 = 0;
        int pp = 0;

        boolean stop = false;

        /*private void controlArm(){
            if(gamepad2.left_stick_y!=0){
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                cp = liftMotor.getCurrentPosition();
                liftMotor.setPower( 1f * gamepad2.left_stick_y);
            }
            else{
                liftMotor.setTargetPosition(cp);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor.setPower(1f);
            }
            if (gamepad2.right_trigger!=0 || gamepad2.left_trigger!=0){
                plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pp = plateMotor.getCurrentPosition();
                plateMotor.setPower ( 1f * (gamepad2.right_trigger-gamepad2.left_trigger));
            }
            else {
                plateMotor.setTargetPosition(pp);
                plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                plateMotor.setPower(1f);
            }
        }*/
        private void controlArm(){
            if (gamepad2.left_stick_y != 0) {
                liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                cp1 = liftMotor1.getCurrentPosition();
                cp2 = liftMotor2.getCurrentPosition();
                liftMotor1.setPower(1f * gamepad2.left_stick_y);
                liftMotor2.setPower(1f * gamepad2.left_stick_y);
            } else {
                liftMotor1.setTargetPosition(cp1);
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor1.setPower(1f);/// de facut cp1>-10?0f:.5f
                liftMotor2.setTargetPosition(cp2);
                liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor2.setPower(1f);/// de facutcp2>-10?0f:.5f
            }
            if (gamepad2.right_trigger!=0 || gamepad2.left_trigger!=0){
                plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pp = plateMotor.getCurrentPosition();
                plateMotor.setPower ( 1f * (gamepad2.right_trigger-gamepad2.left_trigger));
            }
            else {
                plateMotor.setTargetPosition(pp);
                plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                plateMotor.setPower(1f);
            }
        }

        private void controlCatcher(){
            if(gamepad1.dpad_left)
                catcher.setPosition(0);
            else if(gamepad1.dpad_right)
                catcher.setPosition(.4f);
        }
        private void setPlateLevel(){
            if(gamepad2.left_bumper){
                pp = - 1000;
                plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                plateMotor.setTargetPosition(-1000);
            }
            else if (gamepad2.right_bumper){
                pp = 1000;
                plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                plateMotor.setTargetPosition(1000);
            }
            else if(gamepad2.dpad_up) {
                pp = 0;
                plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                plateMotor.setTargetPosition(0);
            }
        }



        private void setLiftLevel(){
            if (gamepad2.a) {
                pp = 0;
                plateMotor.setTargetPosition(0);
                plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                cp1 = 5;
                liftMotor1.setTargetPosition(cp1);
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor1.setPower(1f);
                cp2 = 5;
                liftMotor2.setTargetPosition(cp2);
                liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor2.setPower(1f);
            } else if (gamepad2.b) {
                cp1 = -732;
                liftMotor1.setTargetPosition(cp1);
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor1.setPower(1f);
                cp2 = -732;
                liftMotor2.setTargetPosition(cp2);
                liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor2.setPower(1f);
            } else if (gamepad2.x) {
                cp1 = -1235;
                liftMotor1.setTargetPosition(cp1);
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor1.setPower(1f);
                cp2 = -1234;
                liftMotor2.setTargetPosition(cp2);
                liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor2.setPower(1f);
            } else if (gamepad2.y) {
                cp1 = -1740;
                liftMotor1.setTargetPosition(cp1);
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor1.setPower(1f);
                cp2 = -1737;
                liftMotor2.setTargetPosition(cp2);
                liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor2.setPower(1f);
            }
        }

        private void suppressWheels() {
            if(gamepad1.right_bumper)
                suppress1 = 0.5f;
            else
                suppress1 = 1f;
        }
        int color;
        private void colorDet() {
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red   /= max;
            colors.green /= max;
            colors.blue  /= max;
            color = colors.toColor();
        }

        private void run(){
            suppressWheels();
            setPlateLevel();
            controlWheels();
            setLiftLevel();
            controlArm();
            controlCatcher();
            debugTelemetry();
            colorDet();
        }

        private void debugTelemetry(){
            telemetry.addData("lift1", liftMotor1.getCurrentPosition());
            telemetry.addData("lift2", liftMotor2.getCurrentPosition());
            telemetry.addData("plate", plateMotor.getCurrentPosition());
            telemetry.addData("claw",catcher.getPosition());
            telemetry.update();
        }

    }