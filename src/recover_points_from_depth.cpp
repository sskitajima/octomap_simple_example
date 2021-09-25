#include "recover_points_from_depth.hpp"

#include <fstream>

// using namespace cv;
// using namespace std;

// camera parameter
// Camera.cols: 960
// Camera.rows: 540

const double fx =  530.2126;
const double fy =  530.1829;
const double cx =  475.1043;
const double cy =  265.1346;

// Camera.k1: 0.060082494282151171
// Camera.k2: -0.072665526039814521
// Camera.p1: 0.00096326179384144969
// Camera.p2: -0.0014841003265764945
// Camera.k3: 0.026152189230298935

Eigen::Vector4d recover_points(const double depth, const double ux, const double uy )
{
    Eigen::Vector4d xyz;
    xyz[0] = ( ux - cx ) * depth/fx;
    xyz[1] = ( uy - cy ) * depth/fy;
    xyz[2] = depth;
    xyz[3] = 1.0;
    return xyz;
}

void read_cloud_from_depth(const cv::Mat& depth_img, octomap::Pointcloud& cloud)
{
    // cout << "cols " << depth_img.cols << " rows: " << depth_img.rows << endl;

    const double cam_dmin = 0.02;
    const double cam_dmax = 4.5;

    for(int i=0; i<depth_img.cols; i++)
    {
        for(int j=0; j<depth_img.rows; j++)
        {
            double depth = depth_img.at<float> ( j,i );

            // 有効なdepthが存在するときのみ有効な点群として扱う
            if ( depth > cam_dmin && depth < cam_dmax ) {
                const Eigen::Vector4d cp = recover_points(depth, i, j);  

                cloud.push_back(cp[0], cp[1], cp[2]);
            }
        }
    }

}

void insert_cloud_to_octomap(octomap::OcTree* const octomap_tree, const octomap::Pointcloud& point_cloud)
{
    // ここでは，原点から点群を得たものとしてoctomapに挿入する
    octomap::point3d sensor_origin{0,0,0};
    octomap_tree->insertPointCloud(point_cloud, sensor_origin);

}

bool dump_points(const octomap::Pointcloud& point_cloud, const std::string& out_point_path)
{
    std::ofstream f(out_point_path.c_str());

    if(!f)
    {
        std::cerr << "[dump_points()]: failed to open file\n";
        return false;
    }

    // 点群を取得し，テキストファイルに書き込んでいく
    for(int i=0; i<point_cloud.size(); i++)
    {
        const double x = point_cloud[i].x();
        const double y = point_cloud[i].y();
        const double z = point_cloud[i].z();

        f << x << "," << y << "," << z << "\n";
    }

    f.close();

    return true;
}


bool dump_octomap(octomap::OcTree* const octomap_tree, const std::string& out_octomap_path)
{
    // バイナリファイル形式で保存（開いても人間には読めない）
    bool is_success = octomap_tree->writeBinary(out_octomap_path);

    return is_success;
}
