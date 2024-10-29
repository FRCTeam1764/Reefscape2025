package frc.robot.libraries.external.util;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}
