#ifndef BSPLINE_H
#define BSPLINE_H

#include <vector>
#include <cmath>
#include "types.h"
#include "GeoLoader.h"

namespace CalculatedPath {

    struct Point3D {
        double x, y, z;
    };

    class BSpline {
    public:
        // Ramer-Douglas-Peucker
        static std::vector<Node> Simplify(const std::vector<Node>& path, double epsilon) {
            if (path.size() < 3) return path;

            std::vector<Node> result;
            
            // Find the point with the maximum distance from the line (start -> end)
            double dmax = 0;
            size_t index = 0;
            size_t end = path.size() - 1;

            for (size_t i = 1; i < end; ++i) {
                double d = perpendicular_distance(path[i], path[0], path[end]);
                if (d > dmax) {
                    index = i;
                    dmax = d;
                }
            }

            // If max distance is greater than epsilon, recursively simplify
            if (dmax > epsilon) {
                // Recursive call
                std::vector<Node> first_seg(path.begin(), path.begin() + index + 1);
                std::vector<Node> last_seg(path.begin() + index, path.end());

                std::vector<Node> rec_results_1 = Simplify(first_seg, epsilon);
                std::vector<Node> rec_results_2 = Simplify(last_seg, epsilon);

                // Build the result list (remove duplicate point at the join)
                result.assign(rec_results_1.begin(), rec_results_1.end() - 1);
                result.insert(result.end(), rec_results_2.begin(), rec_results_2.end());
            } else {
                // Keep only start and end
                result.push_back(path[0]);
                result.push_back(path[end]);
            }

            return result;
        }

        // B-splein
        static std::vector<Waypoint> Generate(
            const std::vector<Node>& path, 
            GeoLoader& loader, 
            const MapData& map, 
            int samples_per_segment = 10) 
        {
            if (path.size() < 2) return {};

            std::vector<Point3D> points;
            for (const auto& n : path) {
                points.push_back({ (double)n.x + 0.5, (double)n.y + 0.5, (double)n.alt });
            }

            // Clamp Start/End
            points.insert(points.begin(), points[0]); 
            points.insert(points.begin(), points[0]); 
            points.push_back(points.back());
            points.push_back(points.back());

            std::vector<Waypoint> smoothPath;

            for (size_t i = 1; i < points.size() - 2; ++i) {
                for (int j = 0; j < samples_per_segment; ++j) {
                    double t = (double)j / samples_per_segment;
                    Point3D p = evaluate_segment(points[i-1], points[i], points[i+1], points[i+2], t);

                    Waypoint wp;
                    wp.wp_nr = 0; 
                    wp.autocontinue = 1;
                    wp.accept_radius = 5;

                    if (loader.grid_to_waypoint(map, p.x, p.y, (float)p.z, wp)) {
                        smoothPath.push_back(wp);
                    }
                }
            }
            
            // Add the final point
            Point3D end = points.back();
            Waypoint endWp;
            if (loader.grid_to_waypoint(map, end.x, end.y, (float)end.z, endWp)) {
                smoothPath.push_back(endWp);
            }

            return smoothPath;
        }

    private:
        static double perpendicular_distance(Node p, Node lineStart, Node lineEnd) {
            double num = std::abs((lineEnd.y - lineStart.y) * p.x - (lineEnd.x - lineStart.x) * p.y + lineEnd.x * lineStart.y - lineEnd.y * lineStart.x);
            double den = std::sqrt(std::pow(lineEnd.y - lineStart.y, 2) + std::pow(lineEnd.x - lineStart.x, 2));
            return (den == 0) ? 0 : num / den;
        }

        static Point3D evaluate_segment(Point3D p0, Point3D p1, Point3D p2, Point3D p3, double t) {
            double it = 1.0 - t;
            double b0 = it * it * it / 6.0;
            double b1 = (3 * t * t * t - 6 * t * t + 4) / 6.0;
            double b2 = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6.0;
            double b3 = t * t * t / 6.0;

            return {
                b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x,
                b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y,
                b0 * p0.z + b1 * p1.z + b2 * p2.z + b3 * p3.z
            };
        }
    };
}

#endif