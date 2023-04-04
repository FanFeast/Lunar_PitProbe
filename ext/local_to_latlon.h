#pragma once

// NAC_DTM_LACUS_MORT02 Lat/Lon Extents
const double maxlat = 45.99763814;
const double minlat = 43.82653900;
const double maxlon = 26.00561469;
const double minlon = 25.04491352;

const double dtm_width = 20395.0;
const double dtm_height = 65835.0;

const double model_origin_x = 11096.52;
const double model_origin_y = 33410.65;

void local_to_latlon(double x, double y, double& lat, double& lon) {
    double x_in_dtm = x + model_origin_x;
    double y_in_dtm = y + model_origin_y;

    lon = x_in_dtm / dtm_width * (maxlon - minlon) + minlon;
    lat = y_in_dtm / dtm_height * (maxlat - minlat) + minlat;
}

void latlon_to_local(double lat, double lon, double& x, double& y) {
    double x_in_dtm = (lon - minlon) / (maxlon - minlon) * dtm_width;
    double y_in_dtm = (lat - minlat) / (maxlat - minlat) * dtm_height;

    x = x_in_dtm - model_origin_x;
    y = y_in_dtm - model_origin_y;
}

void latlon_to_grid(double lat, double lon, int numRows, double cell_pitch, int& i, int& j) {
    
    double x, y;
    latlon_to_local(lat, lon, x, y);

    i = numRows - (int)(y / cell_pitch);
    j = (int)(x / cell_pitch);
}