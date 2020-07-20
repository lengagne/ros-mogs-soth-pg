#ifndef MOGSOCTTREE_H
#define MOGSOCTTREE_H

#include "MogsGeometry.h"


class MogsOctTree
{
    public:
        MogsOctTree(const MogsGeometry* geom,
                    double precision = 0.01);
        virtual ~MogsOctTree();

        std::vector<Eigen::Matrix<double,3,1> > points_;

        void get_min_max(   Eigen::Matrix<double,3,1>& min,
                            Eigen::Matrix<double,3,1>& max)
        {
            min = dim_min_;
            max = dim_max_;
        }

        unsigned int get_nb_cross (double x, double y, std::vector<double>& z, double precision = 0.01);



        unsigned int nb_points_=0;



    protected:
    private:
        MogsOctTree(const Eigen::Matrix<double,3,1> min,
                            const Eigen::Matrix<double,3,1> max,
                            std::vector<Eigen::Matrix<double,3,1> > p,
                            double precision);

        std::vector<MogsOctTree*> sons_;
        Eigen::Matrix<double,3,1> dim_min_;
        Eigen::Matrix<double,3,1> dim_max_;

        double size_;


};

#endif // MOGSOCTTREE_H
