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
    typedef struct
    {
        odr_1_5::t_road_planView_geometry * psubroad{nullptr};
        odr_1_5::t_road_lanes_laneSection * psection{nullptr};
        int gindex{0};
        double gs{0}; // geometry s
        int sindex{0};
    } SValue;
    class LanePoint : public Eigen::Vector4d
    {
        public: LanePoint(double x, double y, double z, double w) : Eigen::Vector4d(x,y,z,w) {}
        float heading;
    };
    class Lane : public std::vector<LanePoint> { public: std::string roadmarktype; std::string lanetype; };
    typedef std::map<int, std::map<int, std::map<int, std::map<int, Lane>>>> LanesContainer;

    XodrBuilder(const std::string & xodrfile, float xodrResolution);
    ~XodrBuilder() = default;
    const LanesContainer & getBoundaries()                  { return m_boundaries; } 
    const LanesContainer & getCenters()                     { return m_centers; }
    size_t getNumberOfPoints()                              { return m_totalPointsN; }
    const std::deque<Eigen::Matrix4d> getTrafficSigns()     { return m_ts; }
    const std::deque<Eigen::Matrix4d> getTrafficLights()    { return m_tl; }

private:
    void parseXodr(const std::string & xodrfile);
    std::multimap<double, SValue> collectSValues(const odr_1_5::t_road &);

private:
    const float m_XodrRes{1};
    // key1: roadID, key2: roadShapeID, key3: laneSection, key4: laneID. Value: {x,y,z,heading}
    LanesContainer m_boundaries;
    LanesContainer m_centers;
    std::deque<Eigen::Matrix4d> m_ts; // traffic signs
    std::deque<Eigen::Matrix4d> m_tl; // traffic lights
    size_t m_totalPointsN{0};
};