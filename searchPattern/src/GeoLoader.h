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
        double geoTransform[6]; // 0: originX, 1: pixelWidth, 2: rot1, 3: originY, 4: rot2, 5: pixelHeight
        std::vector<float> elevation_grid;
        
        OGRSpatialReference targetSRS; // The projection of the TIF (usually UTM)
        OGRSpatialReference sourceSRS; // WGS84 (Lat/Lon)
        
        bool is_valid(int x, int y) const {
            return x >= 0 && x < width && y >= 0 && y < height;
        }

        float get_raw_elevation(int x, int y) const {
            if (!is_valid(x, y)) return -9999.0f;
            return elevation_grid[y * width + x];
        }
    };

    class GeoLoader {
    public:
        GeoLoader();
        ~GeoLoader();

        bool load_map(const std::string& filepath, MapData& out_map);

        // NEW: Get precise elevation for a Lat/Lon using bilinear interpolation
        float get_elevation_at_coordinate(const MapData& map, double lat, double lon) const;

        // Keep these if needed for other legacy parts, but searchPattern won't strictly need them
        bool waypoint_to_node(const MapData& map, const Waypoint& wp, Node& out_node);
        bool node_to_waypoint(const MapData& map, const Node& node, Waypoint& out_wp);

    private:
        bool initialized;
    };
}

#endif