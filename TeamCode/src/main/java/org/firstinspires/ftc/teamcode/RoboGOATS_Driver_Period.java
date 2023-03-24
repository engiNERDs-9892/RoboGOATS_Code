package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="RoboGOATS Driver Period", group="Linear Opmode")
//@Disabled
public class RoboGOATS_Driver_Period extends LinearOpMode {



    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;
    private DcMotor motorLS;

    // The Servo ____; are used to identify a servo that can be used throughout the code.

    Servo servoCL;
    Servo servoCR;

    TouchSensor Touch;


    @Override
    public void runOpMode() {


        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorLS = hardwareMap.dcMotor.get("motorLS");


        Touch = hardwareMap.get(TouchSensor.class, "Touchsensor");
        servoCL = hardwareMap.servo.get("servoCL");
        servoCR = hardwareMap.servo.get("servoCR");


        motorLS.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorLS.setPower(0);

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorLS.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Variable used for Regular speed (To find the direction that the stick needs to be in)
            double max;

            // Variable used for Fast speed (To find the direction that the stick needs to be in)
            double fmax;

            // Variable used for Slow speed (To find the direction that the stick needs to be in)
            double smax;


            double ls = gamepad2.left_stick_y; // Slide up or down


            double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value

            // The code below talks about the X-axis (Left and Right)

            double lateral = -gamepad1.left_stick_x; // The bottom two are inverted because motor direction is changed

            // The code below talks about Z-Axis (Spinning around)

            double yaw = -gamepad1.right_stick_x;

            double xrs = .5;


            // Combine the joystick requests for each axis-motion to determine each wheel's power
            // And direction for Regular speed
            double leftFrontPower = .6 * (axial + lateral + yaw);
            double rightFrontPower = .6 * (axial - lateral - yaw);
            double leftBackPower = .6 * (axial - lateral + yaw);
            double rightBackPower = .6 * (axial + lateral - yaw);

            // Combine the joystick requests for each axis-motion to determine each wheel's power
            // And direction for Fast speed
            double FleftFrontPower = (axial + lateral + yaw);
            double FrightFrontPower = (axial - lateral - yaw);
            double FleftBackPower = (axial - lateral + yaw);
            double FrightBackPower = (axial + lateral - yaw);

            // Combine the joystick requests for each axis-motion to determine each wheel's power
            // And direction for Slow speed
            double SleftFrontPower = .25 * (axial + lateral + yaw);
            double SrightFrontPower = .25 * (axial - lateral - yaw);
            double SleftBackPower = .25 * (axial - lateral + yaw);
            double SrightBackPower = .25 * (axial + lateral - yaw);



            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            fmax = Math.max(Math.abs(FleftFrontPower), Math.abs(FrightFrontPower));
            fmax = Math.max(fmax, Math.abs(FleftBackPower));
            fmax = Math.max(fmax, Math.abs(SrightBackPower));

            smax = Math.max(Math.abs(SleftFrontPower), Math.abs(SrightFrontPower));
            smax = Math.max(smax, Math.abs(SleftBackPower));
            smax = Math.max(smax, Math.abs(SrightBackPower));

            if (fmax > 1.0) {

                FleftFrontPower = fmax;
                FrightFrontPower = fmax;
                FleftBackPower = fmax;
                FrightBackPower = fmax;
            }

            if (smax > 1.0) {
                SleftFrontPower = smax;
                SrightFrontPower = smax;
                SleftBackPower = smax;
                SrightBackPower = smax;
            }

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }


            // Setting the power for Slow Speed
            if (gamepad1.left_trigger != 0) {
                motorFL.setPower(SleftFrontPower);
                motorBL.setPower(SleftBackPower);
                motorBR.setPower(SrightBackPower);
                motorFR.setPower(SrightFrontPower);

            }

            // Setting the power for Fast Speed
            else if (gamepad1.right_trigger != 0) {

                motorFL.setPower(FleftFrontPower);
                motorBL.setPower(FleftBackPower);
                motorBR.setPower(FrightBackPower);
                motorFR.setPower(FrightFrontPower);
            }

            // Setting the power for Regular Speed
            else {
                motorFL.setPower(leftFrontPower);
                motorBL.setPower(leftBackPower);
                motorFR.setPower(rightFrontPower);
                motorBR.setPower(rightBackPower);
            }


            // close arms
            if (gamepad2.a) {
                servoCL.setPosition(0);
                servoCR.setDirection(Servo.Direction.REVERSE);
                servoCR.setPosition(0);
            }

            // Open arms
            

            if (gamepad2.b) {
                servoCL.setPosition(.12);
                servoCR.setDirection(Servo.Direction.REVERSE);
                servoCR.setPosition(.12);
            }

            if (ls < 0 ){
                motorLS.setPower(ls);
            }

            if (ls > 0 && Touch.isPressed() ) {
                    motorLS.setPower(0);
                }

            if (ls >0 && !Touch.isPressed()){
                motorLS.setPower(ls*.8);
                }

            if (ls == 0) {
                motorLS.setPower(0);
            }

        }

    }
}

