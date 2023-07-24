//
// Created by lacie-life on 2023/24/07.   
//

#include <iostream>
#include <Eigen/Core>

#include <unistd.h>

#include <pangolin/pangolin.h>
#include <Eigen/Geometry>

void draw_trajectory(const std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& trajectory);


//create a function read the data from trajectory.txt
void read_data(const std::string& filename, std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& data)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Cannot open file: " << filename << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        double timestamp, tx, ty, tz, qx, qy, qz, qw;
        iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

        Eigen::Isometry3d T = Eigen::Isometry3d(Eigen::Quaterniond(qw, qx, qy, qz));
        T.pretranslate(Eigen::Vector3d(tx, ty, tz));
        data.push_back(T);
    }
}


int main(){

    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> trajectory;


    //read the trajectory.txt file
    const std::string trajectoryFile("/home/dinhnambkhn/Desktop/Visual_SLAM_book/Visual_SLAM_tutorial/ch03/trajectory.txt");

    read_data(trajectoryFile, trajectory);

    //show the size of trajectory
    std::cout << "trajectory size: " << trajectory.size() << std::endl;
    //show the first element of trajectory
    std::cout << "first element of trajectory: " << trajectory[0].matrix() << std::endl;
    //draw the trajectory
    draw_trajectory(trajectory);

    return 0;
}

//draw trajectory function
void draw_trajectory(const std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& trajectory)
{
    //create a window
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Create opengl render,
    // Object representing attached OpenGl Matrices / transforms.
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    //create display
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
            .SetHandler(&handler);

    //create trajectory points
    while (!pangolin::ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);

        for (auto & pose : trajectory) {
            /* take rotation
            auto rot = pose.rotation();
            //to quaternion
            Eigen::Quaterniond q(rot);
            //to angle axis
            Eigen::AngleAxisd angle_axis(q);
            */

            //create vector3d for pose
            Eigen::Vector3d Ow = pose.translation();
            Eigen::Vector3d Xw = pose * (0.1 * Eigen::Vector3d(1, 0, 0));
            Eigen::Vector3d Yw = pose * (0.1 * Eigen::Vector3d(0, 1, 0));
            Eigen::Vector3d Zw = pose * (0.1 * Eigen::Vector3d(0, 0, 1));
            glBegin(GL_LINES);
            glColor3f(1.0f, 0.0f, 0.0f); //red, Ox
            glVertex3d(Ow[0], Ow[1], Ow[2]); //origin O
            glVertex3d(Xw[0], Xw[1], Xw[2]); // Px
            glColor3f(0.0f, 1.0f, 0.0f); //green, Oy
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Yw[0], Yw[1], Yw[2]);
            glColor3f(0.0f, 0.0f, 1.0f); //blue, Oz
            glVertex3d(Ow[0], Ow[1], Ow[2]);
            glVertex3d(Zw[0], Zw[1], Zw[2]);
            glEnd();
        }
        //create line for all pose
        for(size_t i =0; i < trajectory.size(); i++){
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            //connect pose i and i+1
            auto p1 = trajectory[i];
            auto p2 = trajectory[i+1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000); //sleep 5ms

    }

}