#include "happly.h"
#include <embree3/rtcore.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
//#include <fmt/format.h>
//#include <fmt/ostream.h>
#include <filesystem>
#include <vector>
#include <string>
#include <random>
#include <numeric>
#include <memory>
#include <cassert>
#include <iostream>
#include <sstream>
#include <string>
#include <local_to_latlon.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

//constants from RFP
const double CIRCLE_RAD = 50.0; //radius around landing site (m)
const double MAX_SLOPE_DEG = 10.0; //maximum slope allowed within landing site area
const float PI = 3.141529;

const float MAP_CELL_PITCH = 1.00; // meters
const float ROVER_HEIGHT = 1.00; // meters
const float LANDER_HEIGHT = 3.00; // meters

struct TerrainMesh {
    Eigen::MatrixXd      vertices;
    Eigen::MatrixXd vertexNormals;
    Eigen::MatrixXi   faceIndices;
};

struct Coord {
    double lat;
    double lon;
};

TerrainMesh loadTerrainMesh(const std::string& plyfile) {
    TerrainMesh tmesh;

    happly::PLYData plyIn(plyfile);
    std::vector<std::array<double, 3>> vertices = plyIn.getVertexPositions();
    std::vector<std::vector<size_t>> faceIndices = plyIn.getFaceIndices<size_t>();
    std::vector<float> nx = plyIn.getElement("vertex").getProperty<float>("nx");
    std::vector<float> ny = plyIn.getElement("vertex").getProperty<float>("ny");
    std::vector<float> nz = plyIn.getElement("vertex").getProperty<float>("nz");

    assert(vertices.size() == nx.size());
    assert(vertices.size() == ny.size());
    assert(vertices.size() == nz.size());

    tmesh.vertices.resize(3, vertices.size());
    tmesh.vertexNormals.resize(3, nx.size());
    tmesh.faceIndices.resize(3, faceIndices.size());

    for (int i = 0; i < vertices.size(); ++i) {
        tmesh.vertices(0, i) = vertices[i][0];
        tmesh.vertices(1, i) = vertices[i][1];
        tmesh.vertices(2, i) = vertices[i][2];

        tmesh.vertexNormals(0, i) = nx[i];
        tmesh.vertexNormals(1, i) = ny[i];
        tmesh.vertexNormals(2, i) = nz[i];
    }

    for (int i = 0; i < faceIndices.size(); ++i) {
        tmesh.faceIndices(0, i) = faceIndices[i][0];
        tmesh.faceIndices(1, i) = faceIndices[i][1];
        tmesh.faceIndices(2, i) = faceIndices[i][2];
    }
    return tmesh;
}

void savePointCloud(const std::string& plyfile, const Eigen::MatrixXd& cloud) {
    happly::PLYData plyOut;
    std::vector<std::array<double, 3>> meshVertexPositions(cloud.cols());
    for (int i = 0; i < cloud.cols(); ++i) {
        meshVertexPositions[i][0] = cloud(0, i);
        meshVertexPositions[i][1] = cloud(1, i);
        meshVertexPositions[i][2] = cloud(2, i);
    }
    plyOut.addVertexPositions(meshVertexPositions);
    plyOut.write(plyfile, happly::DataFormat::Binary);
}

void savePNG(const std::string& filename, const Eigen::MatrixXd& imgR, const Eigen::MatrixXd& imgG, const Eigen::MatrixXd& imgB) {
    int rows = imgR.rows();
    int cols = imgR.cols();

    unsigned char* data = (unsigned char*)malloc(3 * rows * cols * sizeof(unsigned char));

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            data[3 * i * cols + 3 * j + 0] = imgR(i, j);
            data[3 * i * cols + 3 * j + 1] = imgG(i, j);
            data[3 * i * cols + 3 * j + 2] = imgB(i, j);
        }
    }
    int ret = stbi_write_png(filename.c_str(), cols, rows, 3, data, 3 * sizeof(data[0]) * cols);
    free(data);
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd>
buildTerrainMaps(const TerrainMesh& tmesh, double pitch) {
    using namespace Eigen;

    VectorXd aobb_min = tmesh.vertices.rowwise().minCoeff();
    VectorXd aobb_max = tmesh.vertices.rowwise().maxCoeff();
    VectorXi map_dims = ((aobb_max - aobb_min) / pitch).cast<int>();

    double Z_HEIGHT = aobb_max[2] + 10.0;

    int numVerts = tmesh.vertices.cols();
    int numFaces = tmesh.faceIndices.cols();

    RTCDevice device = rtcNewDevice(NULL);
    RTCScene  scene = rtcNewScene(device);
    RTCGeometry geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
    float* vb = (float*)rtcSetNewGeometryBuffer(geometry, RTC_BUFFER_TYPE_VERTEX,
        0, RTC_FORMAT_FLOAT3,
        3 * sizeof(float), numVerts);
    for (int i = 0; i < numVerts; ++i) {
        vb[3 * i + 0] = (float)tmesh.vertices(0, i);
        vb[3 * i + 1] = (float)tmesh.vertices(1, i);
        vb[3 * i + 2] = (float)tmesh.vertices(2, i);
    }
    unsigned* ib = (unsigned*)rtcSetNewGeometryBuffer(geometry, RTC_BUFFER_TYPE_INDEX,
        0, RTC_FORMAT_UINT3,
        3 * sizeof(unsigned), numFaces);
    for (int i = 0; i < numFaces; ++i) {
        ib[3 * i + 0] = tmesh.faceIndices(0, i);
        ib[3 * i + 1] = tmesh.faceIndices(1, i);
        ib[3 * i + 2] = tmesh.faceIndices(2, i);
    }

    rtcCommitGeometry(geometry);
    rtcAttachGeometry(scene, geometry);
    rtcReleaseGeometry(geometry);
    rtcCommitScene(scene);

    MatrixXd  elevationMap(map_dims[1], map_dims[0]);
    MatrixXd      slopeMap(map_dims[1], map_dims[0]);
    elevationMap.fill(Z_HEIGHT);
    slopeMap.fill(0.0);

    for (int i = 0; i < elevationMap.rows(); ++i) {
        //fmt::print("Building Terrain Maps: {}/{}\n", i, elevationMap.rows());
        std::cout << "Building Terrain Maps:" << "current row in map: " << i << "total num rows: " << elevationMap.rows() << std::endl;
#pragma omp parallel for
        for (int j = 0; j < elevationMap.cols(); ++j) {
            RTCRayHit rayhit;
            rayhit.ray.org_x = aobb_min[0] + j * pitch;
            rayhit.ray.org_y = aobb_min[1] + (elevationMap.rows() - i) * pitch;
            rayhit.ray.org_z = Z_HEIGHT;
            rayhit.ray.dir_x = 0.0f;
            rayhit.ray.dir_y = 0.0f;
            rayhit.ray.dir_z = -1.0f;
            rayhit.ray.tnear = 0.0f;
            rayhit.ray.tfar = std::numeric_limits<float>::infinity();
            rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);
            rtcIntersect1(scene, &context, &rayhit);

            if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
                elevationMap(i, j) = Z_HEIGHT - rayhit.ray.tfar;
                float nx = rayhit.hit.Ng_x;
                float ny = rayhit.hit.Ng_y;
                float nz = rayhit.hit.Ng_z;
                float nn = std::sqrt(nx * nx + ny * ny + nz * nz);
                slopeMap(i, j) = std::acos(nz / nn) * 180 / PI; //remmeber
            }
        }
    }

    rtcReleaseScene(scene);
    rtcReleaseDevice(device);

    return std::make_tuple(elevationMap, slopeMap);
}

void addSlopeColormap(const Eigen::MatrixXd& slopeMap, 
    Eigen::MatrixXd& visR,
    Eigen::MatrixXd& visG, 
    Eigen::MatrixXd& visB)
{
    // Use the slope map to color code the result
            // image in shades of red/orange/yellow.
    for (int i = 0; i < slopeMap.rows(); ++i) {
        for (int j = 0; j < slopeMap.cols(); ++j) {
            double s = slopeMap(i, j);
            if (s < 5) {
                visR(i, j) = 63.0;
                visG(i, j) = 11.0;
                visB(i, j) = 27.0;
            }
            if (5 < s && s < 10) {
                visR(i, j) = 122.0;
                visG(i, j) = 22.0;
                visB(i, j) = 49.0;
            }
            if (10 < s && s < 15) {
                visR(i, j) = 207.0;
                visG(i, j) = 66.0;
                visB(i, j) = 60.0;
            }
            if (15 < s && s < 20) {
                visR(i, j) = 252.0;
                visG(i, j) = 125.0;
                visB(i, j) = 73.0;
            }
            if (20 < s) {
                visR(i, j) = 255.0;
                visG(i, j) = 212.0;
                visB(i, j) = 98.0;
            }
        }
    }
}

void drawLandingSite(const Eigen::MatrixXd& slopeMap,
    Eigen::MatrixXd& visR,
    Eigen::MatrixXd& visG,
    Eigen::MatrixXd& visB,
    int li, int lj)
{
    // Loop over a square centered on the landing site.
    for (int i = -CIRCLE_RAD / MAP_CELL_PITCH; i <= CIRCLE_RAD / MAP_CELL_PITCH; ++i) {
        for (int j = -CIRCLE_RAD / MAP_CELL_PITCH; j <= CIRCLE_RAD / MAP_CELL_PITCH; ++j) {
            // Skip pixels that are out of bounds.
            if (li + i >= visR.rows() ||
                li + i < 0 || lj + j < 0 ||
                lj + j >= visR.cols()) {
                continue;
            }

            // Draw a blue circle around the landing site.
            if (std::sqrt(i * i + j * j) < CIRCLE_RAD / MAP_CELL_PITCH) {
                double alpha = 0.6;
                visR(li + i, lj + j) = std::clamp((1 - alpha) * visR(li + i, lj + j) + alpha * 0, 0.0, 255.0);
                visG(li + i, lj + j) = std::clamp((1 - alpha) * visG(li + i, lj + j) + alpha * 0, 0.0, 255.0);
                visB(li + i, lj + j) = std::clamp((1 - alpha) * visR(li + i, lj + j) + alpha * 255, 0.0, 255.0);
            }

            // Draw a white dot at the center of the landing site.
            if (std::sqrt(i * i + j * j) < 4.0 / MAP_CELL_PITCH) {
                visR(li + i, lj + j) = 255.0;
                visG(li + i, lj + j) = 255.0;
                visB(li + i, lj + j) = 255.0;
            }
        }
    }
}

void addLOScolormap(const Eigen::MatrixXd& LOSpercent,
    Eigen::MatrixXd& visR,
    Eigen::MatrixXd& visG,
    Eigen::MatrixXd& visB)
{
    double alpha = 0.4;
    // Use the slope map to color code the result
    // image in shades of green at the rover sites
    for (int i = 0; i < LOSpercent.rows(); ++i) {
        for (int j = 0; j < LOSpercent.cols(); ++j) {
            double p = LOSpercent(i, j);
            if (0 < p && p < 25) {
               visR(i, j) = std::clamp((1 - alpha) * visR(i, j) + alpha * 0, 0.0, 255.0);
               visG(i, j) = std::clamp((1 - alpha) * visG(i, j) + alpha * 51, 0.0, 255.0);
               visB(i, j) = std::clamp((1 - alpha) * visR(i, j) + alpha * 0, 0.0, 255.0);
            }
            if (25 < p && p < 50) {
                visR(i, j) = std::clamp((1 - alpha) * visR(i, j) + alpha * 0, 0.0, 255.0);
                visG(i, j) = std::clamp((1 - alpha) * visG(i, j) + alpha * 102, 0.0, 255.0);
                visB(i, j) = std::clamp((1 - alpha) * visR(i, j) + alpha * 0, 0.0, 255.0);
            }
            if (50 < p && p < 75) {
                visR(i, j) = std::clamp((1 - alpha) * visR(i, j) + alpha * 0, 0.0, 255.0);
                visG(i, j) = std::clamp((1 - alpha) * visG(i, j) + alpha * 153, 0.0, 255.0);
                visB(i, j) = std::clamp((1 - alpha) * visR(i, j) + alpha * 0, 0.0, 255.0);
            }
            if (75 < p && p < 100) {
                visR(i, j) = std::clamp((1 - alpha) * visR(i, j) + alpha * 0, 0.0, 255.0);
                visG(i, j) = std::clamp((1 - alpha) * visG(i, j) + alpha * 204, 0.0, 255.0);
                visB(i, j) = std::clamp((1 - alpha) * visR(i, j) + alpha * 0, 0.0, 255.0);
            }
            if (p == 100) {
                visR(i, j) = std::clamp((1 - alpha) * visR(i, j) + alpha * 0, 0.0, 255.0);
                visG(i, j) = std::clamp((1 - alpha) * visG(i, j) + alpha * 255, 0.0, 255.0);
                visB(i, j) = std::clamp((1 - alpha) * visR(i, j) + alpha * 0, 0.0, 255.0);
            }
        }
    }
}

void getLOSpercent(Eigen::MatrixXd& LOSpercent,
    const TerrainMesh& tmesh,
    const Eigen::MatrixXd& elevationMap,
    const Eigen::MatrixXd& slopeMap,
    int li, int lj) {
    using namespace Eigen;

    double pitch = MAP_CELL_PITCH;

    //MatrixXd LOSpercent(elevationMap.rows(), elevationMap.cols());

    VectorXd aobb_min = tmesh.vertices.rowwise().minCoeff();
    VectorXd aobb_max = tmesh.vertices.rowwise().maxCoeff();
    VectorXi map_dims = ((aobb_max - aobb_min) / pitch).cast<int>();

    int numVerts = tmesh.vertices.cols();
    int numFaces = tmesh.faceIndices.cols();

    RTCDevice device = rtcNewDevice(NULL);
    RTCScene  scene = rtcNewScene(device);
    RTCGeometry geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
    float* vb = (float*)rtcSetNewGeometryBuffer(geometry, RTC_BUFFER_TYPE_VERTEX,
        0, RTC_FORMAT_FLOAT3,
        3 * sizeof(float), numVerts);
    for (int i = 0; i < numVerts; ++i) {
        vb[3 * i + 0] = (float)tmesh.vertices(0, i);
        vb[3 * i + 1] = (float)tmesh.vertices(1, i);
        vb[3 * i + 2] = (float)tmesh.vertices(2, i);
    }
    unsigned* ib = (unsigned*)rtcSetNewGeometryBuffer(geometry, RTC_BUFFER_TYPE_INDEX,
        0, RTC_FORMAT_UINT3,
        3 * sizeof(unsigned), numFaces);
    for (int i = 0; i < numFaces; ++i) {
        ib[3 * i + 0] = tmesh.faceIndices(0, i);
        ib[3 * i + 1] = tmesh.faceIndices(1, i);
        ib[3 * i + 2] = tmesh.faceIndices(2, i);
    }

    rtcCommitGeometry(geometry);
    rtcAttachGeometry(scene, geometry);
    rtcReleaseGeometry(geometry);
    rtcCommitScene(scene);

    // Compute lander position in 3D from the (li, lj) map coordinates.
    Vector3d lander_pos;
    lander_pos[0] = aobb_min[0] + lj * pitch;
    lander_pos[1] = aobb_min[1] + (elevationMap.rows() - li) * pitch;
    lander_pos[2] = elevationMap(li, lj) + LANDER_HEIGHT;

    //fmt::print("Evaluating Site: {} {}\n", li, lj);
    std::cout << "    Evaulating Position:" << li << ", " << lj << std::endl;

    // Loop over the entire map. At each site on the map, place a rover.
    // Shoot a ray from the lander antenna to the rover antenna.
    // If the ray is not obstructed, blend green into the map color.
    for (int ri = 0; ri < elevationMap.rows(); ++ri) {
        for (int rj = 0; rj < elevationMap.cols(); ++rj) {
            // Compute rover position in 3D from (ri, rj) map coordinates.
            Vector3d rover_pos;
            rover_pos[0] = aobb_min[0] + rj * pitch;
            rover_pos[1] = aobb_min[1] + (elevationMap.rows() - ri) * pitch;
            rover_pos[2] = elevationMap(ri, rj) + ROVER_HEIGHT;

            // Compute the normalized direction of
            // a ray from the lander to the rover.
            Vector3d ray_dir = (rover_pos - lander_pos);
            double lander_to_rover_dist = ray_dir.norm();
            ray_dir /= lander_to_rover_dist;

            // Construct the inputs for the embree raytracing function.
            RTCRayHit rayhit;
            rayhit.ray.org_x = lander_pos[0];
            rayhit.ray.org_y = lander_pos[1];
            rayhit.ray.org_z = lander_pos[2];
            rayhit.ray.dir_x = ray_dir[0];
            rayhit.ray.dir_y = ray_dir[1];
            rayhit.ray.dir_z = ray_dir[2];
            rayhit.ray.tnear = 0.0f;
            rayhit.ray.tfar = std::numeric_limits<float>::infinity();
            rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

            // Ask embree to raytrace for you.
            RTCIntersectContext context;
            rtcInitIntersectContext(&context);
            rtcIntersect1(scene, &context, &rayhit);

            // found_hit is true if the ray hit the mesh somewhere.
            bool found_hit = rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID;

            // This is the distance from the lander to the hit.
            double hitdist = rayhit.ray.tfar;

            // If no hit was found, line-of-sight comms is possible.
            // If a hit was found, but the hit is farther away from
            // the lander than the rover, line-of-sight is still possible.
            if (!found_hit ||
                (found_hit && hitdist > lander_to_rover_dist))
            {
                LOSpercent(ri, rj)+= 1.0;
            }
        }
    }
}

void gridSearchTargeted(const TerrainMesh& tmesh,
    const Eigen::MatrixXd& elevationMap,
    const Eigen::MatrixXd& slopeMap,
    Coord landSite) {
    using namespace Eigen;

    MatrixXd LOSpercent(elevationMap.rows(), elevationMap.cols());

    MatrixXd visR(elevationMap.rows(), elevationMap.cols());
    MatrixXd visG(elevationMap.rows(), elevationMap.cols());
    MatrixXd visB(elevationMap.rows(), elevationMap.cols());

    // Use the slope map to color code the result
            // image in shades of red/orange/yellow.
    addSlopeColormap(slopeMap, visR, visG, visB);

    double STEP_SIZE = CIRCLE_RAD * 2;
    double CIRCLE_STEP_SIZE = 20.0; //step size inside a landing circle (m)
    
    //int li, lj = 0;
    //latlon_to_grid(landSite.lat, landSite.lon, elevationMap.rows(), MAP_CELL_PITCH, li, lj);
    double x, y;
    latlon_to_local(landSite.lat, landSite.lon, x, y);
    int li = elevationMap.rows() - (int)(y / MAP_CELL_PITCH);
    int lj = (int)(x / MAP_CELL_PITCH);

    // Evaluate slopes inside the circle centered on this landing site.
            // If any slope is too steep, skip this landing site and move on to the next one.
    double max_slope = 0;
    int count = 0;
    for (int i = -CIRCLE_RAD / MAP_CELL_PITCH; i <= CIRCLE_RAD / MAP_CELL_PITCH; ++i) {
        for (int j = -CIRCLE_RAD / MAP_CELL_PITCH; j <= CIRCLE_RAD / MAP_CELL_PITCH; ++j) {
            if (std::sqrt(i * i + j * j) > CIRCLE_RAD / MAP_CELL_PITCH) { continue; }
            if (li + i < 0 || li + i >= slopeMap.rows() ||
                lj + j < 0 || lj + j >= slopeMap.cols()) {
                continue;
            }
            max_slope = std::max(max_slope, slopeMap(li + i, lj + j));
            count++;
        }
    }
    if (max_slope > MAX_SLOPE_DEG) 
        std::cout << "THERE WAS A SLOPE OVER THE MAX ALLOWED SLOPE FOUND IN LANDING CIRCLE" << std::endl;

    std::cout << "Evaulating Site (lat, lon): " << landSite.lat << ", " << landSite.lon << std::endl;
    int numLandCirclePnts = 0;
    int radiusGrid = (int)(CIRCLE_RAD / MAP_CELL_PITCH);
    int gridStep = (int)(CIRCLE_STEP_SIZE / MAP_CELL_PITCH);

    // Loop over points inside landing circle,
    // determining percent of line of site
    for (int ci = -radiusGrid; ci <= radiusGrid; ci += gridStep) {
        //#pragma omp parallel for // This line tells OpenMP to parallelize this loop.
        for (int cj = -radiusGrid; cj <= radiusGrid; cj += gridStep) {
            // Skip pixels that are out of bounds.
            if (li + ci >= visR.rows() ||
                li + ci < 0 || lj + cj < 0 ||
                lj + cj >= visR.cols()) {
                continue;
            }

            // Only process points that are inside landing circle
            if (std::sqrt(ci * ci + cj * cj) < CIRCLE_RAD / MAP_CELL_PITCH) {

                numLandCirclePnts++;

                // Draw a white dot at each point inside landing circle that was checked
                visR(li + ci, lj + cj) = 255.0;
                visG(li + ci, lj + cj) = 255.0;
                visB(li + ci, lj + cj) = 255.0;

                getLOSpercent(LOSpercent, tmesh, elevationMap, slopeMap, li + ci, lj + cj);
            }
        }
    }

    //average the LOS values over the number of points within landing circle to get the percentage 
    LOSpercent = LOSpercent / numLandCirclePnts * 100;

    //color map different shade of green according to percentage of rover points 
    // that are within LOS of the landing circle sampled points 
    addLOScolormap(LOSpercent, visR, visG, visB);

    // After we do raytracing of the line-of-sight comms,
    // We can overlay position and radius around the landing site onto the map.
    drawLandingSite(slopeMap, visR, visG, visB, li, lj);

    //savePNG(fmt::format("site_{:03}_{:03}.png", li, lj), visR, visG, visB);
    std::stringstream fileName;
    fileName << "site_" << landSite.lat << "_" << landSite.lon << ".png";
    savePNG(fileName.str(), visR, visG, visB);
    std::cout << "SAVED IMAGE!" << std::endl;
}

void gridSearch(const TerrainMesh& tmesh,
    const Eigen::MatrixXd& elevationMap,
    const Eigen::MatrixXd& slopeMap) {
    using namespace Eigen;

    double STEP_SIZE = CIRCLE_RAD*2;
    double CIRCLE_STEP_SIZE = 10.0; //step size inside a landing circle (m)
  
    // Loop over a grid of possible landing sites on the map.
    for (int li = 0; li < elevationMap.rows(); li += (int)(STEP_SIZE / MAP_CELL_PITCH)) {
       #pragma omp parallel for // This line tells OpenMP to parallelize this loop.
        for (int lj = 0; lj < elevationMap.cols(); lj += (int)(STEP_SIZE / MAP_CELL_PITCH)) {

            //percentage of points inside lander circle line-of-site for each rover point
            MatrixXd LOSpercent(elevationMap.rows(), elevationMap.cols());

            MatrixXd visR(elevationMap.rows(), elevationMap.cols());
            MatrixXd visG(elevationMap.rows(), elevationMap.cols());
            MatrixXd visB(elevationMap.rows(), elevationMap.cols());

            // Use the slope map to color code the result
            // image in shades of red/orange/yellow.
            addSlopeColormap(slopeMap, visR, visG, visB);

            // Evaluate slopes inside the circle centered on this landing site.
            // If any slope is too steep, skip this landing site and move on to the next one.
            double max_slope = 0;
            int count = 0;
            for (int i = -CIRCLE_RAD / MAP_CELL_PITCH; i <= CIRCLE_RAD / MAP_CELL_PITCH; ++i) {
                for (int j = -CIRCLE_RAD / MAP_CELL_PITCH; j <= CIRCLE_RAD / MAP_CELL_PITCH; ++j) {
                    if (std::sqrt(i * i + j * j) > CIRCLE_RAD / MAP_CELL_PITCH) { continue; }
                    if (li + i < 0 || li + i >= slopeMap.rows() ||
                        lj + j < 0 || lj + j >= slopeMap.cols()) {
                        continue;
                    }
                    max_slope = std::max(max_slope, slopeMap(li + i, lj + j));
                    count++;
                }
            }
            if (max_slope > MAX_SLOPE_DEG) { continue; }

            std::cout << "Evaulating Site: " << li << ", " << lj << std::endl;
            int numLandCirclePnts = 0;
            // Loop over points inside landing circle,
            // determining percent of line of site
            for (int ci = -CIRCLE_RAD / MAP_CELL_PITCH; ci <= CIRCLE_RAD / MAP_CELL_PITCH; ci += CIRCLE_STEP_SIZE / MAP_CELL_PITCH) {
                for (int cj = -CIRCLE_RAD / MAP_CELL_PITCH; cj <= CIRCLE_RAD / MAP_CELL_PITCH; cj += CIRCLE_STEP_SIZE / MAP_CELL_PITCH) {
                    // Skip pixels that are out of bounds.
                    if (li + ci >= visR.rows() ||
                        li + ci < 0 || lj + cj < 0 ||
                        lj + cj >= visR.cols()) {
                        continue;
                    }

                    // Only process points that are inside landing circle
                    if (std::sqrt(ci * ci + cj * cj) < CIRCLE_RAD / MAP_CELL_PITCH) {

                        numLandCirclePnts++;

                        // Draw a white dot at each point inside landing circle that was checked
                        visR(li + ci, lj + cj) = 255.0;
                        visG(li + ci, lj + cj) = 255.0;
                        visB(li + ci, lj + cj) = 255.0;

                        getLOSpercent(LOSpercent, tmesh, elevationMap, slopeMap, li + ci, lj + cj);
                    }
                }
            }

            //average the LOS values over the number of points within landing circle to get the percentage 
            LOSpercent = LOSpercent / numLandCirclePnts * 100;
            
            //color map different shade of green according to percentage of rover points 
            // that are within LOS of the landing circle sampled points 
            addLOScolormap(LOSpercent, visR, visG, visB);

            // After we do raytracing of the line-of-sight comms,
            // We can overlay position and radius around the landing site onto the map.
            drawLandingSite(slopeMap, visR, visG, visB, li, lj);

            //savePNG(fmt::format("site_{:03}_{:03}.png", li, lj), visR, visG, visB);
            std::stringstream fileName;
            fileName << "site_" << li << "_" << lj << ".png";
            savePNG(fileName.str(), visR, visG, visB);
            std::cout << "SAVED IMAGE!" << std::endl;
        }
    }
}

int main(int argc, char* argv[]) {
    using namespace Eigen;
    //using namespace fmt;

    // 0. Set configuration parameters.
    //const float MAP_CELL_PITCH = 1.00; // meters
    //const float ROVER_HEIGHT = 1.00; // meters
    //const float LANDER_HEIGHT = 3.00; // meters

    // 1. Read .ply mesh file.
    auto terrainFile = (argc > 1) ? argv[1] : "C:/Users/emili/source/repos/FanFeast/Engineering_Computation/meshes/lacus_mortis.ply";
    auto tmesh = loadTerrainMesh(terrainFile);

    // 2. Construct elevation and slope maps.
    auto [elevationMap, slopeMap] = buildTerrainMaps(tmesh, MAP_CELL_PITCH);

    // 3. Evaluate landing sites.
    //gridSearch(tmesh, elevationMap, slopeMap);

    Coord landSite1 = { 44.961, 25.628 };
    gridSearchTargeted(tmesh, elevationMap, slopeMap, landSite1);

    Coord landSite2 = { 44.967, 25.629 };
    //gridSearchTargeted(tmesh, elevationMap, slopeMap, landSite2);

    Coord landSite3 = { 44.956, 25.608 };
    //gridSearchTargeted(tmesh, elevationMap, slopeMap, landSite3);

    return 0;
}
