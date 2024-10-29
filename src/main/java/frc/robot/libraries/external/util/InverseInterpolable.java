package frc.robot.libraries.external.util;

public interface InverseInterpolable<T> {
    double inverseInterpolate(T upper, T query);
}
