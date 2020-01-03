package com.frc1678.lib.logging;

import com.frc1678.c2019.loops.Looper;
import com.frc1678.c2019.loops.Loop;
import com.frc1678.c2019.loops.ILooper;

import java.util.ArrayList;
import java.io.FileWriter;

public class LoggingSystem {
    private static LoggingSystem mInstance; 
    //  LoggableItems object
    ArrayList<ILoggable> loggable_items = new ArrayList<ILoggable>();

    //  creating a usingFileWriter object per loggable file
    ArrayList<FileWriter> loggable_files = new ArrayList<FileWriter>();
    private LoggingSystem() {
    }
    public synchronized static LoggingSystem getInstance() {
        if (mInstance == null) {
            mInstance = new LoggingSystem();
        }
        return mInstance; 
    }
    public void register(ILoggable new_loggable, String filename) {  //  start function that opens files
        FileWriter fileWriter = null;
        try {
            fileWriter = new FileWriter(filename);
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            fileWriter = null;
        }
        ArrayList<String> item_names = new_loggable.getItemNames();
        loggable_files.add(fileWriter);
        // write names to file
        try {
            for (int h=0; h < item_names.size(); h++) {
                fileWriter.write(item_names.get(h));
                if (h < item_names.size() - 1)
                    fileWriter.write(",");
            }
            fileWriter.write("\n");
            //  adding Loggable to Loggable_items list
            loggable_items.add(new_loggable);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    void log() {  //  function that gets called and told when to log by main
        try{
            for (int i=0; i < loggable_items.size(); i++) {
               ArrayList<ArrayList<Double>> items = loggable_items.get(i).getItems();
               //  assertArrayEquals(items[i], item_names[i]);
               //  get object fileWriter from the list 
               FileWriter fileWriter = loggable_files.get(i);
               //  write to files
               for (int j=0; j < items.size(); j++) {
                   ArrayList<Double> data = items.get(j);
                   for (int m=0; m < data.size(); m++){
                    fileWriter.write(data.get(m).toString());
                    fileWriter.write(",");  
                }
                fileWriter.write("\n");
               }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }  finally { // making compiler happy by trying to catch stuff that could crash 
            flush();
        }
    }
    void flush() {  //  flush buffered writes to safely pull on disable
        try {
            for (int i=0; i< loggable_files.size(); i++) {
                FileWriter fileWriter = loggable_files.get(i);
                fileWriter.flush();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    public void registerLoops(ILooper looper) {
        looper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }
            @Override 
            public void onLoop(double timestamp) {
                log();
            }
            @Override 
            public void onStop(double timestamp) {
                flush();
            }
        });
    }
}
