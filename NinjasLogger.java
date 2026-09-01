package frc.lib.NinjasLib;

import dev.doglog.DogLog;

import java.util.ArrayList;
import java.util.List;

public class NinjasLogger extends DogLog {
//    private static final List<String> events = new ArrayList<>();
    private static long eventsCount;

//    private static final char[] ALPHABET = {
//        '3', '4', '6', '7', '9',
//        'A', 'C', 'D', 'E', 'F', 'G', 'H', 'J', 'K', 'L', 'M',
//        'N', 'P', 'Q', 'R', 'T', 'U', 'V', 'W', 'X', 'Y'
//    };

    private static final char[] ALPHABET = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
        'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L'
    };

    private static String numToId(long number) {
        if (number == 0) {
            return String.valueOf(ALPHABET[0]);
        }

        boolean isNegative = number < 0;
        // Use Math.abs or handle Long.MIN_VALUE boundary safely
        long value = Math.abs(number);

        StringBuilder sb = new StringBuilder();
        int base = ALPHABET.length;

        while (value > 0) {
            int remainder = (int) (value % base);
            sb.append(ALPHABET[remainder]);
            value /= base;
        }

        if (isNegative) {
            sb.append('-');
        }

        return sb.reverse().toString();
    }

    public static void logEvent(String event) {
        event = "[" + numToId(eventsCount) + "] " + event;
        eventsCount++;

        System.out.println(event);
        log("Event", event);
//        events.add(event);
//        log("Events", events.toArray(new String[0]));
    }
}
