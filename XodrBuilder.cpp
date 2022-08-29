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


XodrBuilder::XodrBuilder(const string & xodrfile, float xodrRes) : m_XodrRes(xodrRes)
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

            for (auto && sv : svalues)
            {
                auto && odr_subroad = *sv.second.psubroad;
                // starting matrix for this segment:
                Eigen::Matrix4d M; M.setIdentity();
                M.block(0,0,3,3) = Eigen::AngleAxisd(*odr_subroad._hdg, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                M.block(0,3,3,1) = Eigen::Vector3d(*odr_subroad._x, *odr_subroad._y, 0.0);
                Eigen::Vector3d velocity(1.0,0.0,0.0);
                Eigen::Vector4d normal(0.0,1.0,0.0,0.0);
                Eigen::Vector4d P(0, 0, 0, 1);

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
                    P = Eigen::Vector4d(s, 0.0, 0.0, 1.0);
                    // velocity as first derivative of position:
                    velocity.x() = 1;
                    velocity.y() = 0;
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
                    auto poly3 = odr_subroad.sub_paramPoly3;
                    P = Eigen::Vector4d(
                        s,
                        *poly3->_aV + *poly3->_bV * s + *poly3->_cV * s * s + *poly3->_dV * s * s * s,
                        0.0,
                        1.0);
                    // velocity as first derivative of position:
                    velocity.x() = 1;
                    velocity.y() = *poly3->_bV + *poly3->_cV * 2 * s + *poly3->_dV * 3 * s * s;
                }
                else if (odr_subroad.sub_spiral)
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

                // left normal:
                normal.x() =-velocity.y();
                normal.y() = velocity.x();
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
                        m_boundaries[roadid][gindex][sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z(), heading);
                    }
                auto buildLane = [&](auto & odr_sublane, int dir, double & twidth)
                {
                    // Boundary:
                    double width = polyInter(S - *odr_section._s, odr_sublane.sub_width, [](void * it) ->double { return *static_cast<decltype(&odr_sublane.sub_width[0])>(it)->_sOffset; });
                    twidth += dir*width;
                    Eigen::Vector4d Ptrf = M * (P + normal*twidth);
                    m_boundaries[roadid][gindex][sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z(), heading);
                    // Center:
                    Ptrf = M * (P + normal*(twidth - dir*width/2));
                    m_centers[roadid][gindex][sindex][*odr_sublane._id].emplace_back(Ptrf.x(), Ptrf.y(), Ptrf.z(), heading);
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
            SValue sv;
            f(&val, sv);
            // specific for sub_roads and sections: add the same s for the butt of two consequtive piecces: |____||____||____||
            if (!svalues.empty()) svalues.insert({*val._s, SValue()});
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
    multimap<double, SValue> svalues = svalues_psubroad;
    for (auto && sec : svalues_psubsect)
        svalues.insert(sec);

    // now remove the redundant 0 key - there should be only one let:
    auto range = svalues.equal_range(0); range.second--;
    range.second->second.psubroad = find_if(range.first, range.second, [](auto it){ return it.second.psubroad; })->second.psubroad;
    range.second->second.psection = find_if(range.first, range.second, [](auto it){ return it.second.psection; })->second.psection;
    svalues.erase(range.first, range.second);
    // add the closing s == odr_road.length:  |____||____||____||____|
    svalues.insert({*odr_road._length, SValue()});
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

    return svalues;
}
