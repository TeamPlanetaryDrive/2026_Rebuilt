package frc.robot.utils;
public class UtilityMethods {
    public static boolean contains(String[] arr, String target) {
        for (String s : arr) {
            if (s.equals(target))
                return true;
        }
        return false;
    }
}
