#include "GeoLoader.h"
#include <iostream>

namespace CalculatedPath {

    GeoLoader::GeoLoader() {
        GDALAllRegister();
        initialized = true;
    }

    GeoLoader::~GeoLoader() {
        // GDAL cleanup happens automatically at program exit or can be forced
    }

    bool GeoLoader::load_map(const std::string& filepath, MapData& out_map) {
        GDALDataset* dataset = (GDALDataset*)GDALOpen(filepath.c_str(), GA_ReadOnly);
        if (!dataset) return false;

        out_map.width = dataset->GetRasterXSize();
        out_map.height = dataset->GetRasterYSize();
        dataset->GetGeoTransform(out_map.geoTransform);

        // Set up Coordinate Systems
        out_map.sourceSRS.SetWellKnownGeogCS("WGS84");
        out_map.sourceSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
        
        out_map.targetSRS.importFromWkt(dataset->GetProjectionRef());
        out_map.targetSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

        // Load the elevation grid (DOM10)
        GDALRasterBand* band = dataset->GetRasterBand(1);
        out_map.elevation_grid.resize(out_map.width * out_map.height);
        
        band->RasterIO(GF_Read, 0, 0, out_map.width, out_map.height, 
                       &out_map.elevation_grid[0], out_map.width, out_map.height, 
                       GDT_Float32, 0, 0);

        GDALClose(dataset);
        return true;
    }

    bool GeoLoader::waypoint_to_node(const MapData& map, const Waypoint& wp, Node& out_node) {
        // Create transformer
        OGRCoordinateTransformation* coordTrans = OGRCreateCoordinateTransformation(
            (OGRSpatialReference*)&map.sourceSRS, (OGRSpatialReference*)&map.targetSRS);
        
        if (!coordTrans) return false;

        double x = wp.lon;
        double y = wp.lat;

        if (!coordTrans->Transform(1, &x, &y)) {
            OCTDestroyCoordinateTransformation(coordTrans);
            return false;
        }

        // Convert UTM meters to Pixel Index
        out_node.x = static_cast<int>(std::floor((x - map.geoTransform[0]) / map.geoTransform[1]));
        out_node.y = static_cast<int>(std::floor((y - map.geoTransform[3]) / map.geoTransform[5]));
        
        // Fetch altitude for the node
        out_node.alt = map.get_elevation(out_node.x, out_node.y);

        OCTDestroyCoordinateTransformation(coordTrans);
        return map.is_valid(out_node.x, out_node.y);
    }

    bool GeoLoader::node_to_waypoint(const MapData& map, const Node& node, Waypoint& out_wp) {
        // 1. Convert Grid Index -> UTM Coordinates (Meters)
        // Formula: World = Origin + (Index * Resolution)
        // We add 0.5 to 'x' and 'y' to get the CENTER of the pixel, 
        // rather than the top-left corner.
        double pixel_center_x = node.x + 0.5;
        double pixel_center_y = node.y + 0.5;

        double utm_x = map.geoTransform[0] + pixel_center_x * map.geoTransform[1] + pixel_center_y * map.geoTransform[2];
        double utm_y = map.geoTransform[3] + pixel_center_x * map.geoTransform[4] + pixel_center_y * map.geoTransform[5];

        // 2. Create Transformer: Target (UTM) -> Source (WGS84 Lat/Lon)
        OGRCoordinateTransformation* coordTrans = OGRCreateCoordinateTransformation(
            (OGRSpatialReference*)&map.targetSRS,  // FROM: The Map's Projection
            (OGRSpatialReference*)&map.sourceSRS); // TO:   GPS (Lat/Lon)
        
        if (!coordTrans) return false;

        // 3. Transform (Updates utm_x and utm_y in place to became Lon/Lat)
        // Note: With TRADITIONAL_GIS_ORDER, the output x is Lon, y is Lat
        if (!coordTrans->Transform(1, &utm_x, &utm_y)) {
            OCTDestroyCoordinateTransformation(coordTrans);
            return false;
        }

        // 4. Populate Waypoint
        out_wp.lon = utm_x;
        out_wp.lat = utm_y;
        out_wp.alt = node.alt; // Pass the altitude straight through

        OCTDestroyCoordinateTransformation(coordTrans);
        return true;
    }
}