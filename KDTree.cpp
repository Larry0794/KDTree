/*
 * file: KDTree.hpp
 * author: J. Frederico Carvalho
 *
 * This is an adaptation of the KD-tree implementation in rosetta code
 * https://rosettacode.org/wiki/K-d_tree
 *
 * It is a reimplementation of the C code using C++.  It also includes a few
 * more queries than the original, namely finding all points at a distance
 * smaller than some given distance to a point.
 *
 */

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <vector>
#include <cfloat>

#include "KDTree.hpp"

KDNode::KDNode() = default;

KDNode::KDNode(const point_t &pt, const size_t &idx_, const KDNodePtr &left_,
               const KDNodePtr &right_)
{
    x = pt;
    index = idx_;
    left = left_;
    right = right_;
}

KDNode::KDNode(const pointIndex &pi, const KDNodePtr &left_,
               const KDNodePtr &right_)
{
    x = pi.first;
    index = pi.second;
    left = left_;
    right = right_;
}

KDNode::~KDNode() = default;

double KDNode::coord(const size_t &idx) { return x[idx]; }
KDNode::operator bool() { return (!x.empty()); }
KDNode::operator point_t() { return x; }
KDNode::operator size_t() { return index; }
KDNode::operator pointIndex() { return pointIndex(x, index); }

KDNodePtr NewKDNodePtr()
{
    KDNodePtr mynode = std::make_shared<KDNode>();
    return mynode;
}

inline double dist2(const point_t &a, const point_t &b)
{
    double distc = 0;
    for (size_t i = 0; i < a.size(); i++)
    {
        double di = a[i] - b[i];
        distc += di * di;
    }
    return distc;
}

inline double dist2(const KDNodePtr &a, const KDNodePtr &b)
{
    return dist2(a->x, b->x);
}

inline double dist(const point_t &a, const point_t &b)
{
    return std::sqrt(dist2(a, b));
}

inline double dist(const KDNodePtr &a, const KDNodePtr &b)
{
    return std::sqrt(dist2(a, b));
}

comparer::comparer(size_t idx_) : idx{idx_} {};

inline bool comparer::compare_idx(const pointIndex &a, //
                                  const pointIndex &b  //
)
{
    return (a.first[idx] < b.first[idx]); //
}

inline void sort_on_idx(const pointIndexArr::iterator &begin, //
                        const pointIndexArr::iterator &end,   //
                        size_t idx)
{
    comparer comp(idx);

    using std::placeholders::_1;
    using std::placeholders::_2;
    std::nth_element(begin, begin + std::distance(begin, end) / 2,
                     end, std::bind(&comparer::compare_idx, comp, _1, _2));
}

using pointVec = std::vector<point_t>;

KDNodePtr KDTree::make_tree(const pointIndexArr::iterator &begin, //
                            const pointIndexArr::iterator &end,   //
                            const size_t &level                   //
)
{
    if (begin == end)
    {
        return leaf; // empty tree
    }
    const size_t length = end - begin;

    if (length > 1)
    {
        sort_on_idx(begin, end, level);
    }
    auto middle = begin + (length / 2);

    KDNodePtr left = make_tree(begin, middle, (level + 1) % dim);
    KDNodePtr right = make_tree(middle + 1, end, (level + 1) % dim);

    return std::make_shared<KDNode>(*middle, left, right);
}

KDTree::KDTree(pointVec point_array)
{
    leaf = std::make_shared<KDNode>();

    // 校验维数相同且非0
    dim = point_array[0].size();
    // iterators
    pointIndexArr arr;
    for (size_t i = 0; i < point_array.size(); i++)
    {
        if (dim && dim == point_array[i].size())
        {
            arr.push_back(pointIndex(point_array[i], i));
        }
        else
        {
            root = std::make_shared<KDNode>();
            dim = 0;
            return;
        }
    }

    auto begin = arr.begin();
    auto end = arr.end();
    size_t level = 0;

    root = KDTree::make_tree(begin, end, level);
}

// default caller
KDNodePtr KDTree::nearest_(const point_t &pt)
{
    KDNodePtr best;

    double best_dist = DBL_MAX;
    size_t dim = pt.size();

    KDNodePtr curr = root;
    size_t level = 0;
    std::vector<std::pair<KDNodePtr, bool>> stack;
    do
    {
        // null
        if (!bool(*curr))
        {
            KDNodePtr curr_root;
            point_t curr_root_pt;
            while (!stack.empty())
            {
                if (stack.back().second)
                {
                    curr = stack.back().first;
                    stack.pop_back();
                    level = (level - 1) % dim;
                    continue;
                }
                curr_root = stack.back().first;
                stack.back().second = true;
                // 剪枝判断，level-1为当前根节点划分子树的依据维度
                double dx = point_t(*curr_root)[level - 1] - pt[level - 1];
                if (dx * dx <= best_dist)
                {
                    if (curr == curr_root->left && bool(*curr_root->right))
                    {
                        curr = curr_root->right;
                        break;
                    }
                    else if (bool(*curr_root->left))
                    {
                        curr = curr_root->left;
                        break;
                    }
                }
            }
        }
        else
        {
            point_t curr_pt(*curr);
            // 更新最有节点、距离
            double d = dist2(curr_pt, pt);
            if (d < best_dist)
            {
                best_dist = d;
                best = curr;
            }
            // 子节点入栈
            double dx = pt[level] - curr_pt[level];
            // select which branch makes sense to check
            if (dx < 0)
            {
                stack.push_back(std::make_pair(curr, false));
                level = (level + 1) % dim;
                curr = curr->left;
            }
            else
            {
                stack.push_back(std::make_pair(curr, false));
                level = (level + 1) % dim;
                curr = curr->right;
            }
        }
    } while (!stack.empty());

    return best;
}

point_t KDTree::nearest_point(const point_t &pt)
{
    return point_t(*nearest_(pt));
}
size_t KDTree::nearest_index(const point_t &pt)
{
    return size_t(*nearest_(pt));
}
pointIndex KDTree::nearest_pointIndex(const point_t &pt)
{
    KDNodePtr Nearest = nearest_(pt);
    return pointIndex(point_t(*Nearest), size_t(*Nearest));
}

pointIndexArr KDTree::neighborhood_( //
    const point_t &pt,               //
    const double &rad                //
)
{
    pointIndexArr nbh;

    double limit_dist = rad * rad;
    size_t dim = pt.size();

    KDNodePtr curr = root;
    size_t level = 0;
    std::vector<std::pair<KDNodePtr, bool>> stack;
    do
    {
        // null
        if (!bool(*curr))
        {
            KDNodePtr curr_root;
            point_t curr_root_pt;
            while (!stack.empty())
            {
                if (stack.back().second)
                {
                    curr = stack.back().first;
                    stack.pop_back();
                    level = (level - 1) % dim;
                    continue;
                }
                curr_root = stack.back().first;
                stack.back().second = true;
                double dx = point_t(*curr_root)[level - 1] - pt[level - 1];
                if (dx * dx <= limit_dist)
                {
                    if (curr == curr_root->left && bool(*curr_root->right))
                    {
                        curr = curr_root->right;
                        break;
                    }
                    else if (bool(*curr_root->left))
                    {
                        curr = curr_root->left;
                        break;
                    }
                }
            }
        }
        else
        {
            point_t curr_pt(*curr);
            // 更新最有节点、距离
            double d = dist2(curr_pt, pt);
            if (d < limit_dist)
            {
                nbh.push_back(pointIndex(*curr));
            }
            // 子节点入栈
            double dx = pt[level - 1] - curr_pt[level - 1];
            // select which branch makes sense to check
            if (dx < 0)
            {
                stack.push_back(std::make_pair(curr, false));
                level = (level + 1) % dim;
                curr = curr->left;
            }
            else
            {
                stack.push_back(std::make_pair(curr, false));
                level = (level + 1) % dim;
                curr = curr->right;
            }
        }
    } while (!stack.empty());
    return nbh;
};

pointIndexArr KDTree::neighborhood( //
    const point_t &pt,              //
    const double &rad)
{
    return neighborhood_(pt, rad);
}

pointVec KDTree::neighborhood_points( //
    const point_t &pt,                //
    const double &rad)
{
    pointIndexArr nbh = neighborhood_(pt, rad);
    pointVec nbhp;
    nbhp.resize(nbh.size());
    std::transform(nbh.begin(), nbh.end(), nbhp.begin(),
                   [](pointIndex x)
                   { return x.first; });
    return nbhp;
}

indexArr KDTree::neighborhood_indices( //
    const point_t &pt,                 //
    const double &rad)
{
    pointIndexArr nbh = neighborhood_(pt, rad);
    indexArr nbhi;
    nbhi.resize(nbh.size());
    std::transform(nbh.begin(), nbh.end(), nbhi.begin(),
                   [](pointIndex x)
                   { return x.second; });
    return nbhi;
}

pointIndexArr KDTree::neighborhood_( //
    const point_t &pt,               //
    const size_t &n                  //
)
{
    pointIndexArr nbh{pointIndex(*root)};
    size_t pos = 1;

    size_t dim = pt.size();

    KDNodePtr curr = root;
    size_t level = 0;
    std::vector<std::pair<KDNodePtr, bool>> stack;
    do
    {
        // null
        if (!bool(*curr))
        {
            KDNodePtr curr_root;
            while (!stack.empty())
            {
                if (stack.back().second)
                {
                    curr = stack.back().first;
                    stack.pop_back();
                    level = (level - 1) % dim;
                    continue;
                }
                curr_root = stack.back().first;
                stack.back().second = true;
                double dx = point_t(*curr_root)[level - 1] - pt[level - 1];
                if (pos <= n - 1 || dx * dx <= dist2(nbh[pos - 1].first, pt))
                {
                    if (curr == curr_root->left && bool(*curr_root->right))
                    {
                        curr = curr_root->right;
                        break;
                    }
                    else if (bool(*curr_root->left))
                    {
                        curr = curr_root->left;
                        break;
                    }
                }
            }
        }
        else
        {
            point_t curr_pt(*curr);
            // 更新最有节点、距离
            double d = dist2(curr_pt, pt);
            if (d < dist2(nbh[pos - 1].first, pt))
            {
                int i = pos - 2;
                if (pos <= n - 1)
                {
                    nbh.push_back(nbh[pos - 1]);
                    ++pos;
                }
                while (i >= 0 && d < dist2(nbh[i].first, pt))
                {
                    nbh[i + 1] = nbh[i];
                    --i;
                }
                nbh[i + 1] = pointIndex(*curr);
            }
            else if (pos > 1 && pos <= n - 1)
            {
                nbh.push_back(pointIndex(*curr));
                ++pos;
            }
            // 子节点入栈
            double dx = pt[level] - curr_pt[level];
            // select which branch makes sense to check
            if (dx < 0)
            {
                stack.push_back(std::make_pair(curr, false));
                level = (level + 1) % dim;
                curr = curr->left;
            }
            else
            {
                stack.push_back(std::make_pair(curr, false));
                level = (level + 1) % dim;
                curr = curr->right;
            }
        }
    } while (!stack.empty());
    return nbh;
};

pointIndexArr KDTree::neighborhood( //
    const point_t &pt,              //
    const size_t &n)
{
    return neighborhood_(pt, n);
}

pointVec KDTree::neighborhood_points( //
    const point_t &pt,                //
    const size_t &n)
{
    pointIndexArr nbh = neighborhood_(pt, n);
    pointVec nbhp;
    nbhp.resize(nbh.size());
    std::transform(nbh.begin(), nbh.end(), nbhp.begin(),
                   [](pointIndex x)
                   { return x.first; });
    return nbhp;
}

indexArr KDTree::neighborhood_indices( //
    const point_t &pt,                 //
    const size_t &n)
{
    pointIndexArr nbh = neighborhood_(pt, n);
    indexArr nbhi;
    nbhi.resize(nbh.size());
    std::transform(nbh.begin(), nbh.end(), nbhi.begin(),
                   [](pointIndex x)
                   { return x.second; });
    return nbhi;
}