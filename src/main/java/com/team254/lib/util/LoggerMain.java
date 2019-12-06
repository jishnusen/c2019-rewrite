//main logging function
public class LoggerMain {
    public static void main(String[] args) {
        ILoggable loggable = new TestLoggable(); 
        float[] get_items = loggable.get_items();
        String[] get_item_names = loggable.get_item_names();
        System.out.println(get_items[0]); //println will be changed to files --> write stuff to file
        System.out.println(get_item_names[0]);
    }
 }