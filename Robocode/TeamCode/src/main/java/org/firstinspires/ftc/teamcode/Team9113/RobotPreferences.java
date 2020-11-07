package org.firstinspires.ftc.teamcode.Team9113;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class RobotPreferences {
    public int timeThreshold = 350;

    public RobotPreferences() {
        String fileName = "robotpref.txt";
        String folderName = "FIRST";
        File file = new File("/" + folderName + "/" + fileName);
        if (!file.isFile())
            return;
        Scanner scn;
        try {
            scn = new Scanner(file);
        } catch (FileNotFoundException e) {
            return;
        }
        while (scn.hasNextLine()) {
            String currentLine = scn.nextLine();
            if (isValid(currentLine)) {
                String key = getKey(currentLine);
                String keyValue = getKeyValue(currentLine);
                setValues(key, keyValue);
            }
        }
    }

    public boolean isValid(String value) {
        int colonIndex = value.indexOf(':');
        int semiColonIndex = value.indexOf(';');
        return (semiColonIndex > 0 && colonIndex > 0) && semiColonIndex > colonIndex;
    }

    public String getKey(String line) {
        int colonIndex = line.indexOf(':');
        return line.substring(0, colonIndex);
    }

    public String getKeyValue(String line) {
        int length = line.length();
        int colonIndex = line.indexOf(':');
        return line.substring(colonIndex + 1, length - 1);
    }

    public void setValues(String key, String keyValue) {
        if (key.equals("timeThreshold"))
            timeThreshold = Integer.parseInt(keyValue);
    }
}
