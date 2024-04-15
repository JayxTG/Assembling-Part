string dayOfProgrammer(int year) {
    string date;
    if (year == 1918) {
        // In 1918, the date of the 256th day of the year was September 26th, due to a calendar change.
        date = "26.09.1918";
    } else if (year < 1918) {
        // In the Julian calendar, leap years are divisible by 4.
        if (year % 4 == 0) {
            date = "12.09." + to_string(year);
        } else {
            date = "13.09." + to_string(year);
        }
    } else {
        // In the Gregorian calendar, leap years are either divisible by 400 or divisible by 4 but not divisible by 100.
        if ((year % 400 == 0) || (year % 4 == 0 && year % 100 != 0)) {
            date = "12.09." + to_string(year);
        } else {
            date = "13.09." + to_string(year);
        }
    }
    return date;
}
