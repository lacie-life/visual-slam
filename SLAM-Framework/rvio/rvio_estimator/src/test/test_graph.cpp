#include <stdio.h>
#include <fstream>
#include "gtsam_backend/graph.h"
#include "ros/ros.h"

graph_solver graph_solver_obj;

int main(int argc, char** argv)
{

    ros::init(argc, argv, "rvio_estimator_test");
    ros::NodeHandle n;

    if(argc != 2)
    {
        std::cout << "input the text file " << std::endl;
        return -1;
    }

    ifstream in(argv[1]);
    if (!in) {
        std::cout << "error opening: " << argv[1] << "\n";
        return 1;
    }

    Eigen::Vector3d Ps; Eigen::Matrix3d Rs; Eigen::Vector3d Vs;  Eigen::Vector3d Bas; Eigen::Vector3d Bgs;
    Ps.setZero(); Rs.setIdentity(); Vs.setZero();
    graph_solver_obj.initialize(Ps, Rs, Vs, Bas, Bgs);

    int frame; int last_frame = 1;
    int counter =0;
    while(true)
    {
        char line[1024];
        in.getline(line, sizeof (line));
        std::stringstream ss(line);
        char type;

        ss >> type;
        ss >> frame;

        //std::cout << "frame" << frame << std::endl;
        //std::cout << "last frame " << last_frame << std::endl;

        if(frame != last_frame || in.eof())
        {
            counter++;
            std::cout << "counter " << counter << std::endl;
            double t =  ros::Time::now().toSec();
            graph_solver_obj.progateWithIMU(t, Rs, Ps);
            graph_solver_obj.optimize();

            if (in.eof())
            {
                break;
            }
        }

        if(type == 'i')
        {
            std::vector<pair<double, Eigen::Vector3d>> acc_vec;
            std::vector<pair<double, Eigen::Vector3d>> ang_vel_vec;

            double ax, ay, az;
            double gx, gy, gz;

            double t = ros::Time::now().toSec();
            ss >> ax; ss >> ay; ss >> az;
            ss >> gx; ss >> gy; ss >> gz;
            Eigen::Vector3d acc(ax, ay, az);
            Eigen::Vector3d ang_vel(ax, ay, az);

            acc_vec.push_back(std::make_pair(t,acc));
            ang_vel_vec.push_back(std::make_pair(t, ang_vel));

            graph_solver_obj.addIMUMeas(acc_vec, ang_vel_vec);
        }
        else if(type == 's')
        {
            int landmark;
            double xl, xr, y;

            ss >> landmark;
            ss >> xl;
            ss >> xr;
            ss >> y;

            Eigen::Vector3d point3d;
            Eigen::Vector2d point0, point1;
            point0 << xl, y;
            point1 << xr, y;

            graph_solver_obj.addStereoMeas(landmark, point0, point1, point3d);
        }

        last_frame = frame;
        usleep(1/800);
    }

    return 0;
}
