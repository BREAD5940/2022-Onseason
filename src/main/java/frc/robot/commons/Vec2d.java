package frc.robot.commons;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Vec2d extends Translation2d {

    public Vec2d() {
        super();
    }

    public Vec2d(double x, double y) {
        super(x, y);
    }

    public Vec2d(double distance, Rotation2d rotation) {
        super(distance, rotation);
    }

    // Returns the dot product of the two vectors
    public double dot(Vec2d other) {
        return this.getX() * other.getX() + this.getY() * other.getY();
    }   
    
    // Returns the scalar cross product of two vectors
    public double cross(Vec2d other) {
        return this.getX() * other.getY() - this.getY() * other.getX();
    }
    
}
