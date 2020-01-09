/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.firstinspires.ftc.teamcode.angleDrifter;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

/**
 *
 * @author Owner
 */
public class PointRoster {
    public File file;
    private FileReader reader;
    private BufferedReader buffer;
    
    public PointRoster(File file){
        
        try{
            this.file = file;
            reader = new FileReader(file);
            buffer = new BufferedReader(reader);
        } catch(FileNotFoundException e){
            e.printStackTrace();
        }
    }

    public String getFilePath(){
        return file.getAbsolutePath();
    }
    
    public String readNextLine() throws IOException{
        return buffer.readLine();
    }
}
