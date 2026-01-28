#ifndef GEOLOADER_H
#define GEOLOADER_H

#include <string>
#include <vector>
#include <gdal_priv.h>
#include <ogr_spatialref.h>
#include "types.h"

namespace CalculatedPath {

    struct MapData {
        int width;
        int height;
        double geoTransform[6];
        std::vector<float> elevation_grid;
        
        // Stored transformation objects for efficiency
        OGRSpatialReference targetSRS;
        OGRSpatialReference sourceSRS;
        
        bool is_valid(int x, int y) const {
            return x >= 0 && x < width && y >= 0 && y < height;
        }

        float get_elevation(int x, int y) const {
            if (!is_valid(x, y)) return -9999.0f;
            return elevation_grid[y * width + x];
        }
    };

    class GeoLoader {
    public:
        GeoLoader();
        ~GeoLoader();

        // Loads the TIFF and initializes coordinate systems
        bool load_map(const std::string& filepath, MapData& out_map);

        // Converts a Lat/Lon Waypoint to a grid Node
        bool waypoint_to_node(const MapData& map, const Waypoint& wp, Node& out_node);

        // Converts a grid Node to a Lat/Lon Waypoint
        bool node_to_waypoint(const MapData& map, const Node& node, Waypoint& out_wp);

        bool grid_to_waypoint(const MapData& map, double x, double y, float alt, Waypoint& out_wp);

    private:
        bool initialized;
    };
}

#endif