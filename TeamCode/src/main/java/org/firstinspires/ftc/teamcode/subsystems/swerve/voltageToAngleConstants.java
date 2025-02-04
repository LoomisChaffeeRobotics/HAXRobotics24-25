package org.firstinspires.ftc.teamcode.subsystems.swerve;

import android.os.Environment;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.apache.commons.lang3.ArrayUtils;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

public class voltageToAngleConstants {
    double smallToBigPulley = (double) 15 / 38; // small:big --> small deg to big deg
    int[] rotations; // full small pulley rotations, added to how much degrees of current rotation
    ArrayList<AnalogInput> encoders = new ArrayList<>(); // list of objects, comes from user getting hardware map inputs
    public String logFilePath = String.format("%s/FIRST/wheelAngles.txt", Environment.getExternalStorageDirectory().getAbsolutePath());
    public File dataLog = AppUtil.getInstance().getSettingsFile(logFilePath);
    double[] voltages; // these are from the analog inputs
    public double[] sm; // small pulley angle values NO ROTATIONS
    double[] lastSm; //last small pulley angle for counting up/down rotations
    double[] angle; // big pulley angle values
    double[] differenceMs;
    public double[] offsets; // the offsets added to it to tell it where's zero, depends every reset, fix it.
    String[] offsetStrings;
    double x1class;
    String[] lastAngStringsWriting; // last angle, but as strings
    String[] smallAngString;
    String[] smallRotString;
    String[] valuesReading = null; // used for reading at the beginning of opmodes
    String lastLineValue = null;
    String fileDataRaw;
    String finalString;
    double tempAngFL, tempAngFR, tempAngBL, tempAngBR;
    double degreesRawFL, degreesRawFR, degreesRawBL, degreesRawBR;
    OpMode opMode; // for telemetry when done reading
    public voltageToAngleConstants(OpMode opMode, HardwareMap hw, String[] encoderNames) {
        this.opMode = opMode;
        for (String encoderName : encoderNames) {
            encoders.add(hw.get(AnalogInput.class, encoderName));
        }
        voltages = new double[encoderNames.length];
        lastSm = new double[encoderNames.length];
        differenceMs = new double[encoderNames.length];
        offsets = new double[encoderNames.length];

        lastAngStringsWriting = new String[encoderNames.length];
        smallAngString = new String[encoderNames.length];
        smallRotString = new String[encoderNames.length];
        offsetStrings = new String[encoderNames.length];

        fileDataRaw = ReadWriteFile.readFile(dataLog);
    }
    public void init_loop() {
        lastLineValue = fileDataRaw;
        lastLineValue = lastLineValue.replace("[", "").replace("]", "");
        valuesReading = lastLineValue.split(",");
        angle = Arrays.stream(Arrays.copyOfRange(valuesReading, 0, 4)).mapToDouble(Double::parseDouble).toArray();
        rotations = Arrays.stream(Arrays.copyOfRange(valuesReading, 4, 8)).mapToInt(Integer::parseInt).toArray();
        sm = Arrays.stream(Arrays.copyOfRange(valuesReading, 8, 12)).mapToDouble(Double::parseDouble).toArray();
        offsets = Arrays.stream(Arrays.copyOfRange(valuesReading, 12, 16)).mapToDouble(Double::parseDouble).toArray();
        // These offsets get read once, but don't get rewritten unless reset in resetStorage
        // This is why there's another array needed but not in loop()
        System.arraycopy(valuesReading, 12, offsetStrings, 0, offsets.length);
        System.arraycopy(sm, 0, lastSm, 0, 4);
        opMode.telemetry.addLine("Done Reading");
        opMode.telemetry.addData("Big Angles", Arrays.toString(angle));
        opMode.telemetry.addData("Small rotations", Arrays.toString(rotations));
        opMode.telemetry.addData("Small Angles", Arrays.toString(sm));
        opMode.telemetry.update();

    }
    Map<Double,Double> bl = new LinkedHashMap<Double, Double>() {{ // Voltage up, degrees down
        put(0.0, 114.0);
        put(0.220, 90.0);
        put(0.634, 45.0);
        put(1.046, 0.0);
        put(1.047, 360.0); //synthetic
        put(1.461, 315.0);
        put(1.874, 270.0);
        put(2.287, 225.0);
        put(2.701, 180.0);
        put(3.114, 135.0);
        put(3.307, 114.0);
    }};
    Map<Double,Double> fr = new LinkedHashMap<Double, Double>() {{ // Voltage up, degrees not up
        put(0.0, 18.0);
        put(0.165, 0.0); //synthetic
        put(0.166, 360.0);
        put(0.579, 315.0);
        put(0.992, 270.0);
        put(1.405, 225.0);
        put(1.819, 180.0);
        put(2.232, 135.0);
        put(2.646, 90.0);
        put(3.059, 45.0);
        put(3.307, 18.0);
    }};
    Map<Double,Double> fl = new LinkedHashMap<Double, Double>() {{ // Voltage up, degrees down.
        put(0.0, 90.0);
        put(0.413, 45.0);
        put(0.827, 0.0);
        put(0.828, 360.0); // synthetic
        put(1.240, 315.0);
        put(1.654, 270.0);
        put(2.067, 225.0);
        put(2.480, 180.0);
        put(2.894, 131.0);
        put(3.307, 90.0);
    }};
    Map<Double,Double> br = new LinkedHashMap<Double, Double>() {{ // voltage up deg down (not side dependent)
        put(0.0, 55.0);
        put(0.092, 45.0);
        put(0.46, 0.0);
        put(0.461, 360.0); // synthetic
        put(0.919, 315.0);
        put(1.341, 270.0);
        put(1.745, 225.0);
        put(2.136, 180.0);
        put(2.572, 135.0);
        put(3.019, 90.0);
        put(3.307, 55.0);
    }};
    List<Map<Double, Double>> modulesTable = new ArrayList<Map<Double, Double>>() {{
        add(0, fl);
        add(1, fr);
        add(2, bl);
        add(3, br);
    }};
    public void getTelemetry(Telemetry t) {
        t.addData("smBR", sm[3]);
        t.addData("smRotBR", rotations[3]);
        t.addData("angBR", angle[3]);
        t.addData("angFL", angle[0]);
        t.addData("angFR", angle[1]);
        t.addData("angBL", angle[2]);
        t.addData("smRotBl", rotations[2]);
        t.addData("smRotFl", rotations[0]);
        t.addData("smRotFR", rotations[1]);
        t.addData("smBL", sm[2]);
        t.addData("smFL", sm[0]);
        t.addData("smFR", sm[1]);
        t.addData("smRawFR", voltages[1]);
        t.addData("smRawFL", voltages[0]);
        t.addData("smRawBR", voltages[3]);
        t.addData("smRawBL", voltages[2]);
        t.addData("lastSM", differenceMs[3]);
        t.addData("x1 class", x1class);
        t.addData("modules", modulesTable.size());
        t.addData("BLtemp", degreesRawBL * smallToBigPulley);
        t.addData("BRtemp", degreesRawBR * smallToBigPulley);
        t.addData("FLtemp", degreesRawFL * smallToBigPulley);
        t.addData("FRtemp", degreesRawFR * smallToBigPulley);
        t.addData("blr", degreesRawBL);
        t.addData("brr", degreesRawBR );
        t.addData("flr", degreesRawFL);
        t.addData("frr", degreesRawFR);
        t.update();
    }
    public void loop() {
//        ArrayList<Object> writingFinal;
        /*      writing final looks like this
       Big Angle            M1, M2... MN
       Small rotation       ...
       Small Angle          ...

       except it's all one line and it actually looks like:

        */

        for (int m = 0; m < modulesTable.size(); m++) {
            voltages[m] = encoders.get(m).getVoltage();
        }

        for(int m = 0; m < modulesTable.size(); m++) {
            smallPulleyAngleAccumulator(voltages[m], m);
            updateBigPulleyCalculator(m);
        }
        // go through for each module, get the newest voltage, update angle measurement, update last angles
        opMode.telemetry.addData("Big Angles", Arrays.toString(angle));
        opMode.telemetry.addData("Small rotations", Arrays.toString(rotations));
        opMode.telemetry.addData("Small Angles", Arrays.toString(sm));
        System.arraycopy(sm, 0, lastSm, 0, 4);
        // write to the txt everything (it overwrites it thankfully)
        // everything being big pulley angle > small pulley full rotations > small pulley angle pose
    }
    public void stopAndLog() {
        for (int m = 0; m < modulesTable.size(); m++) {
            voltages[m] = encoders.get(m).getVoltage();
        }
        for(int m = 0; m < modulesTable.size(); m++) {
            smallPulleyAngleAccumulator(voltages[m], m);
            updateBigPulleyCalculator(m);
            lastAngStringsWriting[m] = Double.toString(angle[m]);
            smallAngString[m] = Double.toString(sm[m]);
            smallRotString[m] = Integer.toString(rotations[m]);
        }
        StringBuilder sb = getStringBuilder();
        finalString = sb.toString();
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(dataLog))) {
            writer.write(finalString);
        } catch (IOException e) {
            opMode.telemetry.addData("Error", "Failed to write to file: " + e.getMessage());
        }
    }
    @NonNull
    private StringBuilder getStringBuilder() {
        StringBuilder sb = new StringBuilder();
        for (String s : lastAngStringsWriting) {
            sb.append(s).append(",");
        }
        for (String s : smallRotString) {
            sb.append(s).append(",");
        }
        for (String s : smallAngString) {
            sb.append(s).append(",");
        }
        for (String s : offsetStrings) {
            sb.append(s).append(",");
        }
        return sb;
    }
    public double[] getBigPulleyAngles() {
        return angle;
    }
    public double voltsToAngle(double voltage, int module) {
        double out;
        Map<Double, Double> targetTable = modulesTable.get(module);
        Set<Double> keySet = targetTable.keySet();
        Collection<Double> valueSet = targetTable.values();
        // both of the below are doubles but it won't let me
        Object[] values = valueSet.toArray();
        Object[] keys = keySet.toArray();
        if (keySet.contains(voltage)) {
            return targetTable.get(voltage);
        } else {
            for (int i = 0; i < keys.length; i++) {
                if (voltage < Double.parseDouble(keys[i].toString()) ) {
                    double y1, x1, y2, x2, m;
                    x2 = Double.parseDouble(keys[i].toString());
                    x1 = Double.parseDouble(keys[i-1].toString());
                    y1 = Double.parseDouble(values[i-1].toString());
                    y2 = Double.parseDouble(values[i].toString());
                    m = (y2 - y1)/(x2 - x1);
                    // point slope of the line b/w the points its between
                    out = (m * (voltage - x1)) + y1;
                    // input x as the voltage into the formula
                    return out;
                }
            }
        }
        return 0;
    }
    public void smallPulleyAngleAccumulator(double inputVoltage, int module) {
        sm[module] = voltsToAngle(inputVoltage, module);
        double difference = sm[module] - lastSm[module];
        differenceMs[module] = difference;
        if (Math.abs(difference) > 180) {
            if (sm[module] < lastSm[module]) {
                rotations[module]++;
            } else {
                rotations[module]--;
            }
        }
        lastSm[module] = sm[module];
        // this updates the small pulley things
    }
    public void updateBigPulleyCalculator(int m) {
        switch (m) {
            // Can't find the core problem but the error seem linear to total distance moved and the solution might work
            case 0:
                degreesRawFL = sm[m] + (rotations[m] * 360) + offsets[0];
                tempAngFL = ((degreesRawFL * smallToBigPulley)/ 0.94 ) % 360;
                if (tempAngFL < 0) {
                    tempAngFL += 360;
                }
                angle[m] = tempAngFL;
                break;
            case 1:
                degreesRawFR = sm[m] + (rotations[m] * 360) + offsets[1];
                tempAngFR = ((degreesRawFR * smallToBigPulley)/ 0.94) % 360;
                if (tempAngFR < 0) {
                    tempAngFR += 360;
                }
                angle[m] = tempAngFR;
                break;
            case 2:
                degreesRawBL = sm[m] + (rotations[m] * 360) + offsets[2];
                tempAngBL= ((degreesRawBL * smallToBigPulley) / 0.94 )% 360;
                if (tempAngBL < 0) {
                    tempAngBL += 360;
                }
                angle[m] = tempAngBL;
                break;
            case 3:
                degreesRawBR = sm[m] + (rotations[m] * 360) + offsets[3];
                tempAngBR = ((degreesRawBR * smallToBigPulley) / 0.94) % 360;
                if (tempAngBR < 0) {
                    tempAngBR += 360;
                }
                angle[m] = tempAngBR;
                break;
        }
    }
}
