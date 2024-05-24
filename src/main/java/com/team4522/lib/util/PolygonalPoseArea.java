package com.team4522.lib.util;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

public class PolygonalPoseArea {

    private final List<Translation2d> vertices;

    public PolygonalPoseArea(Translation2d... vertices) {
        this.vertices = List.of(vertices);
    }

    public boolean contains(Translation2d point) {
        int intersectionCount = 0;
        for (int i = 0; i < vertices.size(); i++) {
            Translation2d start = vertices.get(i);
            Translation2d end = vertices.get((i + 1) % vertices.size());

            // Check if ray from point to positive infinity intersects the segment
            if (isBetween(start.getX(), end.getY(), point.getY()) && isOnSegment(point, start, end)) {
                // Ignore coinciding with a vertex if the other endpoint is higher
                if (!(point.equals(start) && end.getY() > start.getY()) && !(point.equals(end) && start.getY() > end.getY())) {
                    intersectionCount++;
                }
            }
        }
        return intersectionCount % 2 != 0;
    }

    private boolean isBetween(double a, double b, double c) {
        return (c >= Math.min(a, b) && c <= Math.max(a, b));
    }

    private boolean isOnSegment(Translation2d point, Translation2d start, Translation2d end) {
        double denominator = (end.getX() - start.getX()) * (end.getY() - point.getY()) - (end.getY() - start.getY()) * (end.getX() - point.getX());
        if (denominator == 0) {
            return false; // Collinear. Not necessarily an intersection
        }
        double t = ((end.getX() - start.getX()) * (point.getY() - start.getY()) - (end.getY() - start.getY()) * (point.getX() - start.getX())) / denominator;
        return (t >= 0 && t <= 1);
    }
}