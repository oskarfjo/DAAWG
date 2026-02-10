#include "GeoLoader.h"
#include <iostream>
#include <cmath>

namespace CalculatedPath {

    GeoLoader::GeoLoader() {
        GDALAllRegister();
        // initialized = true; // removed member var not in struct if unused, or keep if part of your class
    }

    GeoLoader::~GeoLoader() {
    }

    bool GeoLoader::load_map(const std::string& filepath, MapData& out_map) {
        GDALDataset* dataset = (GDALDataset*)GDALOpen(filepath.c_str(), GA_ReadOnly);
        if (!dataset) {
            std::cerr << "GDALOpen failed for: " << filepath << std::endl;
            return false;
        }

        out_map.width = dataset->GetRasterXSize();
        out_map.height = dataset->GetRasterYSize();
        dataset->GetGeoTransform(out_map.geoTransform);

        // Set up Coordinate Systems
        out_map.sourceSRS.SetWellKnownGeogCS("WGS84");
        out_map.sourceSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
        
        out_map.targetSRS.importFromWkt(dataset->GetProjectionRef());
        out_map.targetSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

        GDALRasterBand* band = dataset->GetRasterBand(1);
        out_map.elevation_grid.resize(out_map.width * out_map.height);
        
        // FIX: Check return value to satisfy warn_unused_result
        CPLErr err = band->RasterIO(GF_Read, 0, 0, out_map.width, out_map.height, 
                       &out_map.elevation_grid[0], out_map.width, out_map.height, 
                       GDT_Float32, 0, 0);

        if (err != CE_None) {
            std::cerr << "RasterIO failed to read elevation data." << std::endl;
            GDALClose(dataset);
            return false;
        }

        GDALClose(dataset);
        return true;
    }

    float GeoLoader::get_elevation_at_coordinate(const MapData& map, double lat, double lon) const {
        // 1. Create Transform: WGS84 -> Map Projection (e.g., UTM)
        OGRCoordinateTransformation* coordTrans = OGRCreateCoordinateTransformation(
            (OGRSpatialReference*)&map.sourceSRS, 
            (OGRSpatialReference*)&map.targetSRS);

        if (!coordTrans) return -9999.0f;

        double x_proj = lon;
        double y_proj = lat;

        // Transform Lat/Lon to Projected Coordinates (Meters)
        if (!coordTrans->Transform(1, &x_proj, &y_proj)) {
            OCTDestroyCoordinateTransformation(coordTrans);
            return -9999.0f;
        }
        OCTDestroyCoordinateTransformation(coordTrans);

        // 2. Convert Projected Coords -> Grid Indices (inverse GeoTransform)
        double det = map.geoTransform[1] * map.geoTransform[5] - map.geoTransform[2] * map.geoTransform[4];
        double X_pixel = (map.geoTransform[5] * (x_proj - map.geoTransform[0]) - map.geoTransform[2] * (y_proj - map.geoTransform[3])) / det;
        double Y_pixel = (map.geoTransform[1] * (y_proj - map.geoTransform[3]) - map.geoTransform[4] * (x_proj - map.geoTransform[0])) / det;

        // 3. Bilinear Interpolation
        int x0 = static_cast<int>(std::floor(X_pixel));
        int y0 = static_cast<int>(std::floor(Y_pixel));
        
        // Bounds check (ensure we have neighbours)
        if (x0 < 0 || x0 >= map.width - 1 || y0 < 0 || y0 >= map.height - 1) {
            return -9999.0f; // Outside map
        }

        float dx = X_pixel - x0;
        float dy = Y_pixel - y0;

        float z00 = map.get_raw_elevation(x0, y0);
        float z10 = map.get_raw_elevation(x0 + 1, y0);
        float z01 = map.get_raw_elevation(x0, y0 + 1);
        float z11 = map.get_raw_elevation(x0 + 1, y0 + 1);

        // Interpolate
        float z0 = z00 * (1 - dx) + z10 * dx;
        float z1 = z01 * (1 - dx) + z11 * dx;
        
        return z0 * (1 - dy) + z1 * dy;
    }

    bool GeoLoader::waypoint_to_node(const MapData& map, const Waypoint& wp, Node& out_node) {
        OGRCoordinateTransformation* coordTrans = OGRCreateCoordinateTransformation(
            (OGRSpatialReference*)&map.sourceSRS, (OGRSpatialReference*)&map.targetSRS);
        if (!coordTrans) return false;
        double x = wp.lon; double y = wp.lat;
        if (!coordTrans->Transform(1, &x, &y)) { OCTDestroyCoordinateTransformation(coordTrans); return false; }
        out_node.x = static_cast<int>(std::floor((x - map.geoTransform[0]) / map.geoTransform[1]));
        out_node.y = static_cast<int>(std::floor((y - map.geoTransform[3]) / map.geoTransform[5]));
        out_node.alt = map.get_raw_elevation(out_node.x, out_node.y);
        OCTDestroyCoordinateTransformation(coordTrans);
        return map.is_valid(out_node.x, out_node.y);
    }
    
    bool GeoLoader::node_to_waypoint(const MapData& map, const Node& node, Waypoint& out_wp) {
         double pixel_center_x = node.x + 0.5;
        double pixel_center_y = node.y + 0.5;
        double utm_x = map.geoTransform[0] + pixel_center_x * map.geoTransform[1] + pixel_center_y * map.geoTransform[2];
        double utm_y = map.geoTransform[3] + pixel_center_x * map.geoTransform[4] + pixel_center_y * map.geoTransform[5];
        OGRCoordinateTransformation* coordTrans = OGRCreateCoordinateTransformation(
            (OGRSpatialReference*)&map.targetSRS, (OGRSpatialReference*)&map.sourceSRS);
        if (!coordTrans) return false;
        if (!coordTrans->Transform(1, &utm_x, &utm_y)) { OCTDestroyCoordinateTransformation(coordTrans); return false; }
        out_wp.lon = utm_x;
        out_wp.lat = utm_y;
        out_wp.alt = node.alt;
        OCTDestroyCoordinateTransformation(coordTrans);
        return true;
    }

}