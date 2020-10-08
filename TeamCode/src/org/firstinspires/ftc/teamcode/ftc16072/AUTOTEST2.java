package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.EncBot;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "AUTOTEST2", group = "FRC 4557")

public class AUTOTEST2 extends LinearOpMode {

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

        telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
        //telemetry.addData("Heading"," %.1f", gyro.getHeading());
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);
        telemetry.addData("Front Distance", " %.1f", frontDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(), m3.getCurrentPosition(), m4.getCurrentPosition());
        telemetry.update();

        //commands to run in auto starts

        setPower(0, 1, 0);
        while (m1.getCurrentPosition() < 2200) {
        }
        setPower(0, 0, 0);

        setPower(0, 0, 0.5);
        while (m1.getCurrentPosition() > 130) {
        }
        setPower(0, 0, 0);

        setPower(0, 1, 0);
        while (m1.getCurrentPosition() < 2400) {
        }
        setPower(0, 0, 0);

        sleep(2000);

        setPower(0, -1, 0);
        while (m1.getCurrentPosition() > 130) {
        }
        setPower(0, 0, 0);

        setPower(0, 0, -0.5);
        while (m1.getCurrentPosition() < 2200) {
        }
        setPower(0, 0, 0);

        setPower(0, 1, 0);
        while (m1.getCurrentPosition() < 10500) {
        }
        setPower(0, 0, 0);

        setPower(0, 0, 0.5);
        while (m1.getCurrentPosition() > 8470) {
        }
        setPower(0, 0, 0);

        setPower(0, 1, 0);
        while (m1.getCurrentPosition() < 10750) {
        }
        setPower(0, 0, 0);

        sleep(2000);

        setPower(0, -1, 0);
        while (m1.getCurrentPosition() > 8470) {
        }
        setPower(0, 0, 0);

        setPower(0, 0, 0.5);
        while (m1.getCurrentPosition() > 6380) {
        }
        setPower(0, 0, 0);

        setPower(0, 1, 0);
        while (m1.getCurrentPosition() < 17000) {
        }
        setPower(0, 0, 0);

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

    void encDriveLess (double forward, double strafe, double rotate, double value) {
        setPower(strafe, forward, rotate);
        while (m1.getCurrentPosition() < value) {
        }
        setPower(0, 0, 0);
    }

    void encDriveGreater (double forward, double strafe, double rotate, double value) {
        setPower(strafe, forward, rotate);
        while (m1.getCurrentPosition() > value) {
        }
        setPower(0, 0, 0);
    }

}

