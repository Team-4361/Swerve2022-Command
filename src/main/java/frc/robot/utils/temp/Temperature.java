package frc.robot.utils.temp;

/**
 * This class is designed to make it easier to convert temperatures, such as with
 * Fahrenheit, Kelvin, Celsius, etc.
 */
public class Temperature {
    /**
     * Converts a specified Temperature to Fahrenheit, and adjusts the formula based on
     * the Temperature Unit it is originally at.
     *
     * @param value The Temperature to Convert.
     * @param currentUnit The Current {@link TempUnit} of the Temperature.
     * @return The Temperature converted to Fahrenheit.
     */
    public static double toFahrenheit(double value, TempUnit currentUnit) {
        switch (currentUnit) {
            case CELSIUS:
                return (value*9/5)+32;

            case SI:
            case KELVIN:
                return (value-273.15)*9/5+32;

            default:
                return value;
        }
    }

    /**
     * Converts a specified Temperature to Celsius, and adjusts the formula based on
     * the Temperature Unit it is originally at.
     *
     * @param value The Temperature to Convert.
     * @param currentUnit The Current {@link TempUnit} of the Temperature.
     * @return The Temperature converted to Celsius.
     */
    public static double toCelsius(double value, TempUnit currentUnit) {
        switch (currentUnit) {
            case FAHRENHEIT:
                return (value-32)*5/9;

            case SI:
            case KELVIN:
                return value-273.15;

            default:
                return value;
        }
    }

    public static double toUnit(double value, TempUnit oldUnit, TempUnit newUnit) {
        switch (newUnit) {
            case FAHRENHEIT:
                return toFahrenheit(value, oldUnit);
            case CELSIUS:
                return toCelsius(value, oldUnit);
            case KELVIN:
            case SI:
                return toKelvin(value, oldUnit);
            default:
                return value;
        }
    }

    /**
     * Converts a specified Temperature to Kelvin, and adjusts the formula based on
     * the Temperature Unit it is originally at.
     *
     * @param value The Temperature to Convert.
     * @param currentUnit The Current {@link TempUnit} of the Temperature.
     * @return The Temperature converted to Kelvin.
     */
    public static double toKelvin(double value, TempUnit currentUnit) {
        switch (currentUnit) {
            case FAHRENHEIT:
                return (value-32)*5/9+273.15;

            case CELSIUS:
                return value+273.15;

            default:
                return value;
        }
    }
}
