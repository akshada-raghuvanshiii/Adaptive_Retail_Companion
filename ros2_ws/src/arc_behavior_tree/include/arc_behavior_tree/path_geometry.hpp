#ifndef ARC_BEHAVIOR_TREE__PATH_GEOMETRY_HPP_
#define ARC_BEHAVIOR_TREE__PATH_GEOMETRY_HPP_

/*
 * Small 2-D helpers shared by arc_motion_gate and fake_person_tracker.
 *
 * Both nodes need "how far along the Dijkstra route is X" so that the
 * FOLLOW gap is measured along the route, not as a straight line.
 * Nothing here plans or publishes a goal.
 */

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace arc_bt
{

struct Point2
{
    double x = 0.0;
    double y = 0.0;
};


struct Projection
{
    double s = 0.0;          // arc length from the path start, m
    double lateral = 0.0;    // distance from the path, m
    bool valid = false;
};


inline double dist(const Point2& a, const Point2& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}


/*
 * A polyline with cumulative arc length.
 */
class Polyline
{
public:

    void set(const std::vector<geometry_msgs::msg::PoseStamped>& poses)
    {
        pts_.clear();
        cum_.clear();

        for (const auto& p : poses)
        {
            Point2 q{p.pose.position.x, p.pose.position.y};

            if (!pts_.empty() && dist(pts_.back(), q) < 1e-6)
            {
                continue;
            }

            pts_.push_back(q);
        }

        rebuild();
    }


    void setPoints(const std::vector<Point2>& pts)
    {
        pts_ = pts;
        rebuild();
    }


    bool empty() const
    {
        return pts_.size() < 2;
    }


    double length() const
    {
        return cum_.empty() ? 0.0 : cum_.back();
    }


    const std::vector<Point2>& points() const
    {
        return pts_;
    }


    Point2 end() const
    {
        return pts_.empty() ? Point2{} : pts_.back();
    }


    /*
     * Closest point on the polyline to q.
     */
    Projection project(const Point2& q) const
    {
        Projection out;

        if (empty())
        {
            return out;
        }

        double best = std::numeric_limits<double>::max();

        for (std::size_t i = 0; i + 1 < pts_.size(); ++i)
        {
            const Point2& a = pts_[i];
            const Point2& b = pts_[i + 1];

            const double vx = b.x - a.x;
            const double vy = b.y - a.y;
            const double len2 = vx * vx + vy * vy;

            double t = 0.0;

            if (len2 > 1e-12)
            {
                t = ((q.x - a.x) * vx + (q.y - a.y) * vy) / len2;
                t = std::clamp(t, 0.0, 1.0);
            }

            const Point2 c{a.x + t * vx, a.y + t * vy};
            const double d = dist(q, c);

            if (d < best)
            {
                best = d;
                out.s = cum_[i] + t * std::sqrt(len2);
                out.lateral = d;
                out.valid = true;
            }
        }

        return out;
    }


    /*
     * Point at arc length s (clamped to the path).
     * heading_out receives the segment heading.
     */
    Point2 pointAt(double s, double* heading_out = nullptr) const
    {
        if (pts_.empty())
        {
            return Point2{};
        }

        if (pts_.size() == 1)
        {
            if (heading_out) { *heading_out = 0.0; }
            return pts_.front();
        }

        s = std::clamp(s, 0.0, length());

        std::size_t i = 0;

        while (i + 2 < pts_.size() && cum_[i + 1] < s)
        {
            ++i;
        }

        const Point2& a = pts_[i];
        const Point2& b = pts_[i + 1];
        const double seg = cum_[i + 1] - cum_[i];
        const double t = seg > 1e-9 ? (s - cum_[i]) / seg : 0.0;

        if (heading_out)
        {
            *heading_out = std::atan2(b.y - a.y, b.x - a.x);
        }

        return Point2{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
    }


private:

    void rebuild()
    {
        cum_.assign(pts_.size(), 0.0);

        for (std::size_t i = 1; i < pts_.size(); ++i)
        {
            cum_[i] = cum_[i - 1] + dist(pts_[i - 1], pts_[i]);
        }
    }


    std::vector<Point2> pts_;
    std::vector<double> cum_;
};

}  // namespace arc_bt

#endif  // ARC_BEHAVIOR_TREE__PATH_GEOMETRY_HPP_
