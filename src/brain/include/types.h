#pragma once
#include <cmath>
#include <string>
#include <vector>
#include <numeric>
#include <iterator>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <math.h>

using namespace std;

/* ------------------ Struct ------------------------*/

// Pitch dimension information
struct FieldDimensions
{
    double length;            // The length of the pitch.
    double width;             // The width of the pitch.
    double penaltyDist;       // The straight-line distance from the penalty spot to the bottom line.
    double goalWidth;         // The width of the goal.
    double circleRadius;      // The radius of the center circle.
    double penaltyAreaLength; // The length of the penalty area.
    double penaltyAreaWidth;  // The width of the penalty area.
    double goalAreaLength;    // The length of the goal area.
    double goalAreaWidth;     // The width of the goal area.
                              // Note: The penalty area is larger than the goal area; the actual lengths and widths of the penalty area and the goal area are smaller. This naming is to be consistent with the competition rules.
};

// const FieldDimensions FD_KIDSIZE{9, 6, 1.5, 2.6, 0.75, 2, 5, 1, 3};
const FieldDimensions FD_KIDSIZE{14, 9, 2.1, 2.6, 1.5, 3, 6, 1, 4};
const FieldDimensions FD_ADULTSIZE{22, 14, 3.5, 2.5, 2, 5.0, 8.0, 2.0, 5.0};


// Pose2D, used to record a point on a plane and its orientation
struct Pose2D
{
    double x = 0;
    double y = 0;
    double theta = 0; // rad, counterclockwise is positive starting from the positive direction of the x-axis.
};

// Point, used to record a three-dimensional point
struct Point
{
    double x;
    double y;
    double z;
};

// Point2D, used to record a two-dimensional point
struct Point2D
{
    double x;
    double y;

    Point2D operator+(const Point2D &other) const {
        return {x + other.x, y + other.y};
    }

    Point2D operator-(const Point2D &other) const {
        return {x - other.x, y - other.y};
    }
    
    Point2D operator/(const double &value)
    {
        x /= value;
        y /= value;
        return {x, y};
    }

    Point2D operator*(const double &value)
    {
        x *= value;
        y *= value;
        return {x, y};
    }

    friend Point2D operator*(double value, const Point2D &p) {
        return {p.x * value, p.y * value};
    }

    Point2D rotateAround(const Point2D &value, double angle) const
    {
        Point2D aux;
        aux.x = x;
        aux.y = y;

        aux = aux - value;

        // rotate
        angle = angle * M_PI / 180.0;
        aux.x = aux.x * cos(angle) - aux.y * sin(angle);
        aux.y = aux.x * sin(angle) + aux.y * cos(angle);

        return aux + value;
    }

    double norm() const
    {
        return sqrt(x*x + y*y);
    }

    Point2D normalized() const
    {
        Point2D aux{x, y};
        return aux / sqrt(x*x + y*y);
    }

    double distanceToPoint(const Point2D &value) const 
    {
        return sqrt((x - value.x) * (x - value.x) +
                    (y - value.y) * (y - value.y));
    }

    bool interceptLine(const Point2D &p1, const Point2D &p2, double threshold) const
    {
        Point2D v, u, w, a, p3;
        p3.x = x;
        p3.y = y;

        v = p2 - p1;
        u = p3 - p1;
        w = u.perpendVector(v);

        double distance = w.norm();

        if (distance <= threshold)
        {
            Point2D vt2dP, Pa, Pb, Pc;
            float fP_PaPb = 0, fPaPb_PaPb = 0, fP_PcPb = 0, fPcPb_PcPb = 0;

            a = w;
            a = a.normalized(); // A^ = W→/|W→|
            a = a * threshold;

            Pa = a + p1;
            Pb = p2 + a;
            Pc = p2 - a;
            vt2dP = p3 - Pb;

            fP_PaPb = vt2dP.dotProduct(Pa - Pb);
            fPaPb_PaPb = (Pa - Pb).dotProduct(Pa - Pb);
            fP_PcPb = vt2dP.dotProduct(Pc - Pb);
            fPcPb_PcPb = (Pc - Pb).dotProduct(Pc - Pb);

            if (fP_PaPb >= 0 &&
                fP_PaPb <= fPaPb_PaPb && // Se 0 < P→ . (Pa-Pb)→ < |Pa-Pb|² E 0 < P→
                                        // . (Pc-Pb)→ < |Pc-Pb|² o ponto P3 está
                fP_PcPb >= 0 && fP_PcPb <= fPcPb_PcPb) // dentro do retângulo
                return true;
        }
        return false;
    }

    Point2D perpendVector(const Point2D &v) const
    {
        Point2D u;
        u.x = x;
        u.y = y;

        return u - ((u.dotProduct(v) / v.normSquared()) * v); // W→ = u→ - ( ( (u→ . v→)/ |v→|²) * v→)
    }

    double dotProduct(const Point2D &other) const
    {
        return x*other.x + y*other.y;
    }

    double normSquared() const
    {
        return x*x + y*y;
    }
};


// BoundingBox
struct BoundingBox
{
    double xmin;
    double xmax;
    double ymin;
    double ymax;
};

/// GameObject, used to store the information of important entities in the game, such as Ball, Goalpost, etc. Compared with the detection::DetectedObject in the /detect message, it has more abundant information.
struct GameObject
{
    // --- Obtained from the /detect message ---
    string label;              // What the object is identified as.
    BoundingBox boundingBox;   // The recognition box of the object in the camera, with the upper left corner as the origin, x increasing to the right and y increasing downward.
    Point2D precisePixelPoint; // The precise pixel point position of the object. Only ground landmark points have this data.
    double confidence;         // The confidence of the identification.
    Point posToRobot;          // The position of the object in the robot's body coordinate system. The position is 2D, ignoring the z value.

    // --- Calculated and obtained in the processDetectedObject function ---
    string info;                     // Used to store additional information. For example, for a goalpost object, it can store which goalpost it is.
    Point posToField;                // The position of the object in the field coordinate system. The position is 2D, ignoring the z value. x is forward and y is leftward.
    double range;                    // The straight-line distance from the object to the projection point of the robot's center on the field plane.
    double pitchToRobot, yawToRobot; // The pitch and yaw of the object relative to the front of the robot, in rad. Downward and leftward are positive.
    rclcpp::Time timePoint;          // The time when the object was detected.
};

// 起身
struct RobotRecoveryStateData {
    uint8_t state; // IS_READY = 0, IS_FALLING = 1, HAS_FALLEN = 2, IS_GETTING_UP = 3,  
    uint8_t is_recovery_available; // 1 for available, 0 for not available
    uint8_t current_planner_index;
};


enum class RobotRecoveryState {
    IS_READY = 0,
    IS_FALLING = 1,
    HAS_FALLEN = 2,
    IS_GETTING_UP = 3
};
