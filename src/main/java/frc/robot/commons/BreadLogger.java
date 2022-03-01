package frc.robot.commons;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;

public class BreadLogger {

    public static List<String> files = new ArrayList<>();
    public final String path;
    
    // Constructs a new bread logger object with the file name "fileName"
    public BreadLogger(String fileName) {
        this.path = Filesystem.getOperatingDirectory() + "/" + fileName + ".csv";
        files.add(fileName);
        
    }
    

    // Method to clear this file
    public void clear() throws IOException {
        new FileWriter(path).close();
    }

    // Method to write to this file
    public void write(String... content) throws IOException {
        if (content.length > 0) {
            String toWrite = "";
            for (int i = 0; i < content.length - 1; i++) {
                toWrite = toWrite + content[i] + ", ";
            }
            toWrite = toWrite + content[content.length - 1] + "\n";
            FileWriter w = new FileWriter(path, true);
            w.write(toWrite);
            w.close();
        }
    }

    // Overload to write to this file
    public void write(int... content) throws IOException {
        String[] stringContent = new String[content.length];
        for (int i = 0; i < content.length; i++) {
            stringContent[i] = String.valueOf(content[i]);
        }
        write(stringContent);
    }

    // Overload to read from this file
    public void write(double... content) throws IOException {
        String[] stringContent = new String[content.length];
        for (int i = 0; i < content.length; i++) {
            stringContent[i] = String.valueOf(content[i]);
        }
        write(stringContent);
    }
}
