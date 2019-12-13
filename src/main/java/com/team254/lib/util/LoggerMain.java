//  main logging function 
//  import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import java.util.ArrayList;
import java.io.FileWriter; 
public class LoggerMain {
    public static void main(String[] args) {
        // creating a loggable object 
        ILoggable loggable = new TestLoggable();
        //  creating a usingFileWriter object
        try{
        FileWriter fileWriter = new FileWriter("Test.csv");
        //  LoggableItems object
        ArrayList<ILoggable> loggable_items = new ArrayList<ILoggable>();
        //  adding Loggable to Loggable_items list
        loggable_items.add(loggable);
        //  for loop will loop over anything that implements the logger
        for (int i=0; i < loggable_items.size(); i++) {
           ArrayList<Double> items = loggable_items.get(i).get_items();
           ArrayList<String> item_names = loggable_items.get(i).get_item_names();
           //  assertArrayEquals(items[i], item_names[i]);
           //  write to files
           // fileWriter.write(items);
           //  fileWrter.write(item_names);
           for (int h=0; h < item_names.size(); h++) {
           fileWriter.write(item_names.get(h));
           fileWriter.write(",");
           }
           fileWriter.write("\n");
           for (int j=0; j < items.size(); j++) {
            fileWriter.write(items.get(j).toString());
            fileWriter.write(",");
           } 
           fileWriter.write("\n");
        }
        fileWriter.close(); // makes sure its actually written
    } catch (Exception e) {}  // making compiler happy by trying to catch stuff that could crash 
    }
 }