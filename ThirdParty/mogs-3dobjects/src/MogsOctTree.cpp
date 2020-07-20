#include "MogsOctTree.h"

MogsOctTree::MogsOctTree(const MogsGeometry* geom,
                         double precision)
{
    //ctor

    geom->get_points(points_);
    nb_points_ = points_.size();
    std::cout<<"There are "<< nb_points_<<" points."<<std::endl;



    Eigen::Matrix<double,3,1> min(3),max(3);
    for (int i=0;i<3;i++)
    {
        min(i)= std::numeric_limits<double>::max();
        max(i)= - std::numeric_limits<double>::max();
    }
    for (int i=0;i<nb_points_;i++)
    {
        const Eigen::Matrix<double,3,1> p = points_[i];
        for (int j=0;j<3;j++)
        {
            if (p(j) < min(j))  min(j) = p(j);
            if (p(j) > max(j))  max(j) = p(j);
        }
    }
    std::cout<<"size of the object: "<<std::endl;
    std::cout<<"X = ["<< min(0)<<" : "<< max(0)<<" ] "<<std::endl;
    std::cout<<"Y = ["<< min(1)<<" : "<< max(1)<<" ] "<<std::endl;
    std::cout<<"Z = ["<< min(2)<<" : "<< max(2)<<" ] "<<std::endl;


    size_ = 0;
    for (int i=0;i<3;i++)
    {
        double v = max(i) - min(i);
        if (v>size_) size_ = v;
    }
    std::cout<<" size = "<< size_ <<std::endl;

    for (int i=0;i<3;i++)
    {
        double mid = (min(i) + max(i))/2.;
        dim_min_(i) = mid - size_ /2;
        dim_max_(i) = mid + size_ /2;
    }

    sons_.push_back(new MogsOctTree(dim_min_,dim_max_,points_,precision));
}

MogsOctTree::~MogsOctTree()
{
    //dtor
}

MogsOctTree::MogsOctTree(   const Eigen::Matrix<double,3,1> min,
                            const Eigen::Matrix<double,3,1> max,
                            std::vector<Eigen::Matrix<double,3,1> > p,
                            double precision)
{
    dim_min_ = min;
    dim_max_ = max;
    Eigen::Matrix<double,3,1> mid;
    size_ = 0;
    for (int i=0;i<3;i++)
    {
        double v = max(i) - min(i);
        if (v>size_) size_ = v;
    }
    if (size_ > precision/2)
    {
        for (int i=0;i<3;i++)
            mid(i) = (min(i)+max(i))/2.0;

        std::vector<Eigen::Matrix<bool,3,1> > B(8);
        B[0](0) = false;    B[0](1) = false;    B[0](2) = false;
        B[1](0) = false;    B[1](1) = false;    B[1](2) = true;
        B[2](0) = false;    B[2](1) = true;     B[2](2) = false;
        B[3](0) = false;    B[3](1) = true;     B[3](2) = true;
        B[4](0) = true;     B[4](1) = false;    B[4](2) = false;
        B[5](0) = true;     B[5](1) = false;    B[5](2) = true;
        B[6](0) = true;     B[6](1) = true;     B[6](2) = false;
        B[7](0) = true;     B[7](1) = true;     B[7](2) = true;

        for(int i=0;i<8;i++)
        {
            Eigen::Matrix<double,3,1> MIN = dim_min_;
            Eigen::Matrix<double,3,1> MAX = dim_max_;
            for (int j=0;j<3;j++)
            {
                if( B[i](j))    MIN(j) = mid(j);    else MAX(j) = mid(j);
            }
            std::vector< Eigen::Matrix<double,3,1> > points;
            for (int j=0;j<p.size();j++)
            {
                if ( p[j](0) >= MIN(0) && p[j](0) <= MAX(0) && p[j](1) >= MIN(1) && p[j](1) <= MAX(1) && p[j](2) >= MIN(2) && p[j](2) <= MAX(2))
                {
                    points.push_back(p[j]);
                }
            }

            if (points.size()>0)
                sons_.push_back(new MogsOctTree(MIN,MAX,points,precision));
        }
    }
}

unsigned int MogsOctTree::get_nb_cross (double x, double y, std::vector<double>& z, double precision)
{

    if ( x >= dim_min_(0) && x <= dim_max_(0) && y >= dim_min_(1) && y <= dim_max_(1))
    {
        if (size_ < precision)
        {
            if (z.size() > 0 && fabs(z[z.size()-1] - dim_min_(2))< precision/10)
            {
                z[z.size()-1] = dim_max_(2);
                return 0;
            }
            z.push_back((dim_min_(2)+dim_max_(2))/2.0);
            return 1;
        }

        unsigned int nb = 0;
        for (int i=0;i<sons_.size();i++)
        {
            nb += sons_[i]->get_nb_cross(x,y,z,precision);
        }
        return nb;
    }
    return 0;
}
