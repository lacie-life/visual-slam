//
// Created by lacie-life on 2023/24/07.   
//

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>
#include <unistd.h>

#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

using TrajectoryType = std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>;

void read_file(const std::string& file_nam,std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>& data);
void DrawTrajectory (const TrajectoryType& gt, const  TrajectoryType& esti);


int main(int argc, char** argv){
    // Read trajectory txt file
    TrajectoryType trajectory_gt;
    read_file(argv[1], trajectory_gt);
    //check size of trajectory
    std::cout<<"GT trajectory size: "<<trajectory_gt.size()<<std::endl;

    //read estimated trajectory
    TrajectoryType trajectory_est;
    read_file(argv[2], trajectory_est);
    //check size of trajectory
    std::cout<<"estimated trajectory size: "<<trajectory_est.size()<<std::endl;

    //check both trajectory have same size
    if(trajectory_gt.size()!=trajectory_est.size()){
        std::cout<<"trajectory size not match"<<std::endl;
        return -1;
    }

    //compute error for SE3
    double error_se3 = 0;
    for(int i=0;i<trajectory_gt.size();i++){
        error_se3 += (trajectory_gt[i].inverse()*trajectory_est[i]).log().norm();
    }
    error_se3 /= double (trajectory_gt.size());
    std::cout<<"SE3 error: "<<error_se3<<"[ ]"<<std::endl;

    //compute error for translation
    double error_translation = 0;
    for(int i=0;i<trajectory_gt.size();i++){
        error_translation += (trajectory_gt[i].inverse()*trajectory_est[i]).translation().norm();
    }
    error_translation /= double (trajectory_gt.size());
    std::cout<<"translation error: "<<error_translation<< "[m]"<< std::endl;

    //draw trajectory
    DrawTrajectory(trajectory_gt,trajectory_est);


    return 0;
}

void read_file(const std::string& file_nam,TrajectoryType& data){
    // read file
    std::ifstream file(file_nam);
    //check if file is open
    if(!file.is_open()){
        std::cout<<"file is not open"<<std::endl;
        return;
    }

    //save the line in variable time, tx, ty, tz, qx, qy, qz, qw
    while(!file.eof()){
        double time, tx, ty, tz, qx, qy, qz, qw;
        file>>time>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
        Sophus::SE3d T(Sophus::SO3d(Eigen::Quaternion<double>(qw, qx, qy, qz)),Eigen::Vector3d(tx,ty,tz));
        data.push_back(T);
    }

}

void DrawTrajectory (const TrajectoryType& gt, const  TrajectoryType& esti)
{
    //create pangolin window
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
            );

    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while( !pangolin::ShouldQuit() )
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);

        //draw trajectory
        for(size_t i = 0; i < gt.size()-1; i++)
        {
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            auto p1 = gt[i];
            auto p2 = gt[i+1];
            glVertex3d(p1.translation()(0), p1.translation()(1), p1.translation()(2));
            glVertex3d(p2.translation()(0), p2.translation()(1), p2.translation()(2));
            glEnd();
        }

        //draw trajectory
        for(size_t i = 0; i < esti.size()-1; i++)
        {
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = esti[i];
            auto p2 = esti[i+1];
            glVertex3d(p1.translation()(0), p1.translation()(1), p1.translation()(2));
            glVertex3d(p2.translation()(0), p2.translation()(1), p2.translation()(2));
            glEnd();
        }

        pangolin::FinishFrame();
        //sleep 5ms
        usleep(5000);

    }

}

