package org.firstinspires.ftc.teamcode.subsystems;
import com.pedropathing.geometry.Pose;

public class LaunchDetector {

        private static class Triangle {
            double x1, y1, x2, y2, x3, y3;

            Triangle(double x1, double y1, double x2, double y2, double x3, double y3) {
                this.x1 = x1; this.y1 = y1;
                this.x2 = x2; this.y2 = y2;
                this.x3 = x3; this.y3 = y3;
            }

            /**
             * Checks if the center point (px, py) is inside the triangle using Barycentric coordinates.
             */
            boolean containsCenter(double px, double py) {
                double detT = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3);
                double alpha = ((y2 - y3) * (px - x3) + (x3 - x2) * (py - y3)) / detT;
                double beta = ((y3 - y1) * (px - x3) + (x1 - x3) * (py - y3)) / detT;
                double gamma = 1.0 - alpha - beta;

                return alpha >= 0 && beta >= 0 && gamma >= 0;
            }
        }

        // --- DECODE FIELD COORDINDATES (INCHES) ---
        private static final Triangle LARGE_LAUNCH_ZONE = new Triangle(72.0, 72.0, 144.0, 144.0, 0.0, 144.0);
        private static final Triangle SMALL_LAUNCH_ZONE = new Triangle(48.0, 0.0, 72.0, 24.0, 96.0, 0.0);

        // --- ROBOT DIMENSIONS (412.7mm x 429.612mm -> Inches) ---
        private static final double ROBOT_WIDTH = 16.248;
        private static final double ROBOT_LENGTH = 16.914;

        public static boolean isOverlappingLaunchZone(Pose robotPose) {
            double cx = robotPose.getX();
            double cy = robotPose.getY();
            double heading = robotPose.getHeading();

            // TEST 1: Absolute Subsumption Check (Is the robot's center fully inside either zone?)
            //if (LARGE_LAUNCH_ZONE.containsCenter(cx, cy) || SMALL_LAUNCH_ZONE.containsCenter(cx, cy)) {
            if (LARGE_LAUNCH_ZONE.containsCenter(cx, cy)) {
                return true;
            }

            // TEST 2: Straddling Check (Do the robot's perimeter walls cross the triangle tape lines?)
            double hW = ROBOT_WIDTH / 2.0;
            double hL = ROBOT_LENGTH / 2.0;

            double[][] localCorners = {
                    { hL, -hW}, { hL,  hW}, {-hL,  hW}, {-hL, -hW}
            };

            double[][] globalCorners = new double[4][2];
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);

            for (int i = 0; i < 4; i++) {
                globalCorners[i][0] = cx + (localCorners[i][0] * cos - localCorners[i][1] * sin);
                globalCorners[i][1] = cy + (localCorners[i][0] * sin + localCorners[i][1] * cos);
            }

            return robotEdgesIntersectTriangle(globalCorners, LARGE_LAUNCH_ZONE) ||
                    robotEdgesIntersectTriangle(globalCorners, SMALL_LAUNCH_ZONE);
        }

        private static boolean robotEdgesIntersectTriangle(double[][] rCorners, Triangle t) {
            double[][] tEdges = {
                    {t.x1, t.y1, t.x2, t.y2},
                    {t.x2, t.y2, t.x3, t.y3},
                    {t.x3, t.y3, t.x1, t.y1}
            };

            for (int i = 0; i < 4; i++) {
                double rx1 = rCorners[i][0]; double ry1 = rCorners[i][1];
                double rx2 = rCorners[(i + 1) % 4][0]; double ry2 = rCorners[(i + 1) % 4][1];

                for (double[] tEdge : tEdges) {
                    if (lineIntersectsSegment(rx1, ry1, rx2, ry2, tEdge[0], tEdge[1], tEdge[2], tEdge[3])) {
                        return true;
                    }
                }
            }
            return false;
        }

        private static boolean lineIntersectsSegment(double x1, double y1, double x2, double y2,
                                                     double x3, double y3, double x4, double y4) {
            double den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
            if (den == 0) return false;

            double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
            double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;

            return t >= 0 && t <= 1 && u >= 0 && u <= 1;
        }
}
