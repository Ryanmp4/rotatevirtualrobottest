package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name = "AUTOTEST3", group = "FRC 4557")

public class AUTOTEST3 extends LinearOpMode {

    DcMotor m1, m2, m3, m4;
    BNO055IMU imu;
    DistanceSensor frontDistance, leftDistance, rightDistance, backDistance;
    ColorSensor colorSensor;

    public void runOpMode(){

        m1 = hardwareMap.dcMotor.get("back_left_motor");
        m2 = hardwareMap.dcMotor.get("front_left_motor");
        m3 = hardwareMap.dcMotor.get("front_right_motor");
        m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";
        imu.initialize(parameters);
        colorSensor = hardwareMap.colorSensor.get("color_sensor");

        //commands to run in auto starts

        encDriveLess(0,1,0,2200);

        encDriveGreater(0,0,0.5,130);

        encDriveLess(0,1,0,2400);

        sleep(2000);

        encDriveGreater(0,-1,0,130);

        encDriveLess(0,0,-0.5,2200);

       encDriveLess(0,1,0,10500);

        encDriveGreater(0,0,0.5,8470);

        encDriveLess(0,1,0,10750);

        sleep(2000);

        encDriveGreater(0,-1,0,8470);

       encDriveGreater(0,0,0.5,6380);

        encDriveLess(0,1,0,17000);

        //commands to run in auto ends

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);

    }

    void setPower(double px, double py, double pa) {
        double p1 = -px + py - pa;
        double p2 = px + py + -pa;
        double p3 = -px + py + pa;
        double p4 = px + py + pa;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        m1.setPower(p1);
        m2.setPower(p2);
        m3.setPower(p3);
        m4.setPower(p4);
    }

    void runEnc() {
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void resetEnc() {
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void encDriveLess (double strafe, double forward, double rotate, double value) {
        setPower(strafe, forward, rotate);
        while (m1.getCurrentPosition() < value) {
        }
        setPower(0, 0, 0);
    }

    void encDriveGreater (double strafe, double forward, double rotate, double value) {
        setPower(strafe, forward, rotate);
        while (m1.getCurrentPosition() > value) {
        }
        setPower(0, 0, 0);
    }



}

