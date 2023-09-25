#pragma once

#include <eigen3/Eigen/Eigen>
#include "odrparser/odr_1_5.h"
#include "odrparser/odrparser.h"

#include <string>
#include <vector>
#include <deque>
#include <map>

class XodrBuilder
{
public:
    struct GeoReference
    {
        int lat_0;
        int lon_0;
        int x_0;
        int y_0;
    };
    typedef struct SValue
    {
        SValue(int f=0) : flag(f) {}
        odr_1_5::t_road_planView_geometry * psubroad{nullptr};
        odr_1_5::t_road_lanes_laneSection * psection{nullptr};
        int gindex{0};
        double gs{0}; // geometry s
        int sindex{0};
        int flag{0}; // -1 starting, 0 intermediate, +1 closing
    } SValue;
    class LanePoint : public Eigen::Vector4d
    {
        public: LanePoint(double x, double y, double z, double w) : Eigen::Vector4d(x,y,z,w) {}
        float heading;
    };
    class Lane : public std::vector<LanePoint> { public: std::string roadmarktype; std::string lanetype; };
    typedef std::map<int, std::map<int, std::map<int, std::map<int, Lane>>>> LanesContainer;

    // The doOptimize flag will skip collecting points for lines, except first and last
    XodrBuilder(const std::string & xodrfile, float xodrResolution = 1.0f, bool doOptimize = false);
    ~XodrBuilder() = default;
    const LanesContainer & getBoundaries()                  { return m_boundaries; } 
    const LanesContainer & getCenters()                     { return m_centers; }
    size_t getNumberOfPoints()                              { return m_totalPointsN; }
    const std::deque<Eigen::Matrix4d> getTrafficSigns()     { return m_ts; }
    const std::deque<Eigen::Matrix4d> getTrafficLights()    { return m_tl; }
    GeoReference getGeoReference()                          { return m_geoRef; }

private:
    std::string checkZip(std::string);
    std::multimap<double, SValue> collectSValues(const odr_1_5::t_road &);

private:
    const float m_XodrRes{1};
    // key1: roadID, key2: roadShapeID, key3: laneSection, key4: laneID. Value: {x,y,z,heading}
    LanesContainer m_boundaries;
    LanesContainer m_centers;
    std::deque<Eigen::Matrix4d> m_ts; // traffic signs
    std::deque<Eigen::Matrix4d> m_tl; // traffic lights
    size_t m_totalPointsN{0};
    GeoReference m_geoRef;
};