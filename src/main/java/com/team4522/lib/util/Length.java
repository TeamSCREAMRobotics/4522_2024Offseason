package com.team4522.lib.util;

public class Length {
    private double inches = 0.0;

    private Length(double inches){
        this.inches = inches;
    }

    public static Length fromInches(double inches){
        return from(inches, 1.0);
    }

    public static Length fromFeet(double feet){
        return from(feet, 12.0);
    }

    public static Length fromCentimeters(double centimeters){
        return from(centimeters, 1.0 / 2.54);
    }

    public static Length fromMeters(double meters){
        return from(meters, 39.37);
    }

    public static Length from(double value, double inchConversionFactor){
        return new Length(value * inchConversionFactor);
    }

    public Length plus(Length other){
        return new Length(this.inches + other.getInches());
    }

    public Length minus(Length other){
        return new Length(this.inches - other.getInches());
    }

    public Length times(double scalar){
        return new Length(this.inches * scalar);
    }

    public Length div(double scalar){
        return new Length(this.inches / scalar);
    }

    public double getInches(){
        return inches;
    }

    public double getFeet(){
        return inches / 12.0;
    }

    public double getCentimeters(){
        return inches * 2.54;
    }

    public double getMeters(){
        return inches / 39.37;
    }
}
