#include "XodrBuilder.h"

#include <set>
#include <iostream>
#include <deque>
#include <omp.h>

using namespace odr_1_5;
using namespace odr;
using namespace std;
using namespace Eigen;

//                     sub-road
// |-------------------------------------------------------------|
//
//                   geometry 1                      geometry 2          sub_planView.sub_geometry
// | --------------------------------------------|---------------|


//
//                                              /               /
//                                             /               /
//                                            /               /
// ........................................../               /
//                                                          /
//                                                .        /
//                                                        /
//                                                       /
// ...................................................../
//                          .
//                    .
//                .          <----- one of two lanes ends (its width will change according to poly function)
//             .  
// .......


// |------------------------|------------------------------------|
//  lane secion 1 (two lanes)    lane section 2 (one lane)


XodrBuilder::XodrBuilder(const string & xodrfile, float xodrRes, bool doOptimize) : m_XodrRes(xodrRes)
{
    OpenDRIVEFile ODR;
    try
    {
      loadFile(xodrfile, ODR);
      /* code */

        for (auto && odr_road : ODR.OpenDRIVE1_5->sub_road)
        {
            // Eventually a bug in xodr parser: odr_road._id is string (instead of int)
            int roadid = atoi((*odr_road._id).c_str());

            // skip of no lanes available
            if (!odr_road.sub_lanes)
                continue;

            // skip if lanes empty
            if (odr_road.sub_lanes->sub_laneSection.empty())
                continue;

            // collect all s values for the road
            auto svalues = collectSValues(odr_road);

            // We need to remember the velocity, normal and P for each subroad due to Spiral
            Eigen::Vector3d velocity(1.0,0.0,0.0);
            Eigen::Vector4d normal(0.0,1.0,0.0,0.0);
            Eigen::Vector4d P(0, 0, 0, 1);

            for (auto && sv : svalues)
            {
                auto && odr_subroad = *sv.second.psubroad;
                // starting matrix for this segment:
                Eigen::Matrix4d M; M.setIdentity();
                M.block(0,0,3,3) = Eigen::AngleAxisd(*odr_subroad._hdg, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                M.block(0,3,3,1) = Eigen::Vector3d(*odr_subroad._x, *odr_subroad._y, 0.0);
                if (sv.first == *odr_subroad._s)
                {
                    // if a new subroad, update the velocity, normal and P (only due to Spiral geometry specific computation)
                    velocity = Eigen::Vector3d(1.0,0.0,0.0);
                    normal = Eigen::Vector4d(0.0,1.0,0.0,0.0);
                    P = Eigen::Vector4d(0, 0, 0, 1);
                }

                double S = sv.first;
                double s = sv.second.gs;

                if (odr_subroad.sub_arc)
                {
                    double R = 1.0/(*odr_subroad.sub_arc->_curvature);
                    double radians = s / R;
                    M.block(0,0,3,3) = Eigen::AngleAxisd(*odr_subroad._hdg - M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                    P = Eigen::Vector4d(R*cos(radians) - R, R*sin(radians), 0.0, 1.0); // found from trial and error
                    // velocity as first derivative of position:
                    velocity.x() =-R*sin(radians);
                    velocity.y() = R*cos(radians);
                    if (R < 0) velocity = -velocity; // found from trial and error
                }
                else if (odr_subroad.sub_line)
                {
                    if (sv.second.flag || !doOptimize)
                    {
                        P = Eigen::Vector4d(s, 0.0, 0.0, 1.0);
                        // velocity as first derivative of position:
                        velocity.x() = 1;
                        velocity.y() = 0;
                    }
                    else goto signals;
                }
                else if (odr_subroad.sub_paramPoly3 && odr_subroad._length)
                {
                    auto poly3 = odr_subroad.sub_paramPoly3;
                    double t = s;
                    if (!odr_subroad.sub_paramPoly3->_pRange || *odr_subroad.sub_paramPoly3->_pRange == "normalized") t /= (*odr_subroad._length);
                    // else "arcLength" and t = s
                    P = Eigen::Vector4d(
                        *poly3->_aU + *poly3->_bU * t + *poly3->_cU * t * t + *poly3->_dU * t * t * t,
                        *poly3->_aV + *poly3->_bV * t + *poly3->_cV * t * t + *poly3->_dV * t * t * t,
                        0.0,
                        1.0);
                    // velocity as first derivative of position:
                    velocity.x() = *poly3->_bU + *poly3->_cU * 2 * t + *poly3->_dU * 3 * t * t;
                    velocity.y() = *poly3->_bV + *poly3->_cV * 2 * t + *poly3->_dV * 3 * t * t;
                }
                else if (odr_subroad.sub_poly3 && odr_subroad._length)
                {
                    auto poly3 = odr_subroad.sub_poly3;
                    P = Eigen::Vector4d(
                        s,
                        *poly3->_a + *poly3->_b * s + *poly3->_c * s * s + *poly3->_d * s * s * s,
                        0.0,
                        1.0);
                    // velocity as first derivative of position:
                    velocity.x() = 1;
                    velocity.y() = *poly3->_b + *poly3->_c * 2 * s + *poly3->_d * 3 * s * s;
                }
                else if (odr_subroad.sub_spiral && s)
                {
                    double t = s/(*odr_subroad._length);
                    double curvs = *odr_subroad.sub_spiral->_curvStart;
                    double curve = *odr_subroad.sub_spiral->_curvEnd;
                    double curvature = (1.0 - t)*curvs + t*curve;
                    if (abs(curvature) < 1e-10) curvature = curvature < 0 ? -1e-10 : 1e-10;
                    double R = 1.0/curvature;
                    Vector3d P2center = R*normal.block(0,0,3,1);
                    Vector3d center = P.block(0,0,3,1) + P2center;
                    auto Rot = AngleAxisd(curvature*m_XodrRes, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                    Vector3d center2pnext = Rot*(-P2center);
                    Vector3d nextP = center + center2pnext;
                    velocity.block(0,0,3,1) = (AngleAxisd(curvature < 0 ? -M_PI_2 : M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix()*center2pnext).normalized();
                    P.block(0,0,3,1) = nextP;
                }
                {
                    // left normal:
                    normal.x() =-velocity.y();
                    normal.y() = velocity.x();
                    normal.z() = 0;
                    normal.normalize();

                    auto polyInter = [&](double T, auto & polis, double (*getS)(void * itt) ) -> double
                    {
                        auto pit = polis.begin(); // polis iterator
                        while (pit != polis.end() && getS(&(*pit)) < T) ++pit;
                        if (pit != polis.begin()) --pit;
                        double t = T - getS(&(*pit));
                        return *(pit->_a) + *(pit->_b)*t + *(pit->_c)*t*t + *(pit->_d)*t*t*t;
                    };

                    // Elevation:
                    if (odr_road.sub_elevationProfile && !odr_road.sub_elevationProfile->sub_elevation.empty())
                        P.z() = polyInter(S, odr_road.sub_elevationProfile->sub_elevation, [](void * it) ->double { return *static_cast<decltype(&odr_road.sub_elevationProfile->sub_elevation[0])>(it)->_s; });

                    // Super-elevation:
                    if (odr_road.sub_lateralProfile && !odr_road.sub_lateralProfile->sub_superelevation.empty())
                    {
                        double roll = polyInter(S, odr_road.sub_lateralProfile->sub_superelevation, [](void * it) ->double { return *static_cast<decltype(&odr_road.sub_lateralProfile->sub_superelevation[0])>(it)->_s; });
                        Matrix3d ori; ori.setIdentity();
                        ori.block(0,0,3,1) = velocity.normalized();
                        ori.block(0,1,3,1) =-normal.block(0,0,3,1); // to make right handed CS
                        ori.block(0,2,3,1) = Vector3d(0,0,1);
                        ori = ori*Eigen::AngleAxisd(-roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
                        normal.block(0,0,3,1) =-ori.block(0,1,3,1);
                    }

                    // heading:
                    Vector3d velocityGlobal = M.block(0,0,3,3) * Vector3d(velocity.x(), velocity.y(), 0);
                    double heading = atan2(velocityGlobal.y(), velocityGlobal.x());

                    uint32_t sindex = sv.second.sindex;
                    uint32_t gindex = sv.second.gindex;
                    auto && odr_section = *sv.second.psection;

                    // set the lanes:
                    double offset = 0.0;
                    if (odr_road.sub_lanes && !odr_road.sub_lanes->sub_laneOffset.empty())
                        offset = polyInter(S, odr_road.sub_lanes->sub_laneOffset, [](void * it) ->double { return *static_cast<decltype(&odr_road.sub_lanes->sub_laneOffset[0])>(it)->_s; });
                    
                    if (odr_section.sub_center)
                        for (auto && odr_sublane : odr_section.sub_center->sub_lane)
                        {
                            // ONLY 1 center line, actually loop not needed
                            Eigen::Vector4d Ptrf = M * (P + normal*offset);
                            m_boundaries[roadid][gindex][sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z(), 1.0);
                            m_boundaries[roadid][gindex][sindex][*odr_sublane._id].lanetype = *odr_sublane._type;
                            m_boundaries[roadid][gindex][sindex][*odr_sublane._id].back().heading = heading;
                            if (!odr_sublane.sub_roadMark.empty())
                            {
                            auto && roadMark = odr_sublane.sub_roadMark[0]; // KB: can be more!!! Proper implementation: iteration!!!
                            m_boundaries[roadid][gindex][sindex][*odr_sublane._id].roadmarktype = *roadMark._type;
                            }
                        }
                    auto buildLane = [&](auto & odr_sublane, int dir, double & twidth)
                    {
                        // Boundary:
                        double width = polyInter(S - *odr_section._s, odr_sublane.sub_width, [](void * it) ->double { return *static_cast<decltype(&odr_sublane.sub_width[0])>(it)->_sOffset; });
                        twidth += dir*width;
                        Eigen::Vector4d Ptrf = M * (P + normal*twidth);
                        m_boundaries[roadid][gindex][sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z(), 1.0);
                        m_boundaries[roadid][gindex][sindex][*odr_sublane._id].lanetype = *odr_sublane._type;
                        m_boundaries[roadid][gindex][sindex][*odr_sublane._id].back().heading = heading;
                        if (!odr_sublane.sub_roadMark.empty())
                        {
                            auto && roadMark = odr_sublane.sub_roadMark[0]; // KB: can be more!!! Proper implementation: iteration!!!
                            m_boundaries[roadid][gindex][sindex][*odr_sublane._id].roadmarktype = *roadMark._type;
                            m_centers[roadid][gindex][sindex][*odr_sublane._id].roadmarktype = *roadMark._type;
                        }
                        // Center:
                        Ptrf = M * (P + normal*(twidth - dir*width/2));
                        m_centers[roadid][gindex][sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z(), 1.0);
                        m_centers[roadid][gindex][sindex][*odr_sublane._id].lanetype = *odr_sublane._type;
                        m_centers[roadid][gindex][sindex][*odr_sublane._id].back().heading = heading;
                        ++m_totalPointsN;
                    };
                    double twidth = offset;
                    if (odr_section.sub_left)
                    {
                        map<int, t_road_lanes_laneSection_left_lane> sublanes_map;
                        for (auto && odr_sublane : odr_section.sub_left->sub_lane) sublanes_map[abs<int>(*odr_sublane._id)] = odr_sublane;
                        for (auto && it : sublanes_map) buildLane(it.second, 1, twidth);
                    }
                    twidth = offset;
                    if (odr_section.sub_right)
                    {
                        map<int, t_road_lanes_laneSection_right_lane> sublanes_map;
                        for (auto && odr_sublane : odr_section.sub_right->sub_lane) sublanes_map[abs<int>(*odr_sublane._id)] = odr_sublane;
                        for (auto && it : sublanes_map) buildLane(it.second, -1, twidth);
                    }
                }
signals:
                // SIGNALS PROCESSING:
                auto processSignal = [&](auto signal)
                {
                    // For building the traffic signs (and traffic lights) we use a simple approach:
                    // we will find the closest object to the current S within the xodrResolution:
                    if (signal._name->find("Sign_") != string::npos)
                    {
                        // traffic signs
                        if (abs(*signal._s - S) < 0.5 * m_XodrRes)
                        {
                            Eigen::Vector4d Ptrf = M * (P - normal * *signal._t);
                            Eigen::Matrix4d trf; trf.setIdentity();
                            trf.block(0,0,3,3) = Eigen::AngleAxisd(*signal._hOffset, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                            trf.block(0,3,4,1) = M * (P - normal * *signal._t);
                            m_ts.push_back(trf);
                        }
                    }
                    if (signal._name->find("Signal_") != string::npos)
                    {
                        // traffic lights
                    }
                };
                // Iterate the signals and references:
                if (odr_road.sub_signals)
                {
                    // signals
                    for (auto &&signal : odr_road.sub_signals->sub_signal)
                    {
                        processSignal(signal);
                    }
                    // references: at the moment the references are not working properly
                    // for (auto && sigref : odr_road.sub_signals->sub_signalReference)
                    // {
                    //   auto signal = *trafficSinganls[*sigref._id]; // COPY!
                    //   *signal._orientation = *sigref._orientation;
                    //   *signal._s = *sigref._s;
                    //   *signal._t = *sigref._t;
                    //   processSignal(signal);
                    // }
                }
            }
        }
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
}


multimap<double, XodrBuilder::SValue> XodrBuilder::collectSValues(const t_road & odr_road)
{
    // specific for subroad and sections:
    auto addElement = [](auto && odr_container, multimap<double, SValue> & svalues, function<void(void * p, SValue & sv)> f)
    {
        for (auto && val : odr_container)
        {
            SValue sv(-1);
            f(&val, sv);
            // specific for sub_roads and sections: add the same s for the butt of two consequtive piecces: |____||____||____||
            if (!svalues.empty())
            {
                svalues.insert({*val._s, SValue(+1)});
            }
            svalues.insert({*val._s, sv});
        }
    };
    multimap<double, SValue> svalues_psubroad; int gindex = 0;
    addElement(odr_road.sub_planView->sub_geometry, svalues_psubroad, [&gindex](void * p, SValue & sv)
        { sv.psubroad = static_cast<decltype(&odr_road.sub_planView->sub_geometry[0])>(p); sv.gindex = gindex; ++gindex;});
    multimap<double, SValue> svalues_psubsect; int sindex = 0;
    addElement(odr_road.sub_lanes->sub_laneSection, svalues_psubsect, [&sindex](void * p, SValue & sv)
        { sv.psection = static_cast<decltype(&odr_road.sub_lanes->sub_laneSection[0])>(p); sv.sindex = sindex; ++sindex;});
    // merge the subroads and sections
    multimap<double, SValue> svalues = move(svalues_psubroad);
    svalues.insert(svalues_psubsect.begin(), svalues_psubsect.end());

    // add the closing s == odr_road.length:  |____||____||____||____|
    svalues.insert({*odr_road._length, SValue(+1)});
    // add the s with the default Res:
    for (double s = m_XodrRes; s < *odr_road._length; s += m_XodrRes)
        svalues.insert({s, SValue()});
    
    // populate the empty values:
    auto sv = svalues.begin()->second; double L = 0.0;
    for (auto && item : svalues)
    {
        if (!item.second.psubroad)
        {
            item.second.psubroad = sv.psubroad;
            item.second.gindex = sv.gindex;
            item.second.gs = item.first - L + *sv.psubroad->_length;
        }
        else
        {
            sv = item.second;
            L += *item.second.psubroad->_length;
        }
    }
    sv = svalues.begin()->second;
    for (auto && item : svalues)
    {
        if (!item.second.psection)
        {
            item.second.psection = sv.psection;
            item.second.sindex = sv.sindex;
        }
        else sv = item.second;
    }

    svalues.erase(svalues.begin()); // remove redundant first [0] element (can be incomplete)

    return svalues;
}

//////////////////////////////// Python bindings ////////////////////////////////

extern "C" {
	XodrBuilder * getXodrBuilder(char * xodrname, double resolution)
    {
        static XodrBuilder * pxb = new XodrBuilder(xodrname, resolution);
        return pxb;
    }
    size_t getNumberOfPoints(XodrBuilder * xb)
    {
        return xb->getNumberOfPoints();
    }
    double * getAllPoints(XodrBuilder * xb)
    {
        static double * points = new double[xb->getNumberOfPoints()*4];
        size_t counter = 0;
        for (auto && r : xb->getCenters()) // roads
        {
            for (auto && g : r.second) // geometries
            {
                for (auto && s : g.second) // sections
                {
                    for (auto && l : s.second) // lanes
                    {
                        for (auto && p : l.second) // points
                        {
                            points[counter++] = p[0];
                            points[counter++] = p[1];
                            points[counter++] = p[2];
                            points[counter++] = p[3];
                        }
                    }
                }
            }
        }
        return points;
    }
}