/*test interface*/
public class TestLoggable implements ILoggable {
    public float[] get_items() {
        float[] arr = new float[3];
        arr[0] = 1;
        arr[1] = 2;
        arr[2] = 3;
        return(arr);
    }
    public String[] get_item_names() {
        String[] ar = new String[3];
        ar[0] = "var3";
        ar[1] = "var2";
        ar[2] = "var1";
        return(ar);


    }
}
