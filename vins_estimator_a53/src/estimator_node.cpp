#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "feature_tracker.h"

#include "DatasetLoader.h"

FeatureTracker trackerData;

double first_image_time;
bool first_image_flag = true;
int pub_count = 1;
double last_image_time = 0;

bool init_pub = 0;
bool init_feature = 0;


Estimator estimator;

std::condition_variable con;
double current_time = -1;
double last_imu_t = 0;


queue<StImuPtr> imu_buf;
queue<StFeaturePointCloudPtr> feature_buf;
queue<StImagePtr> raw_image_buf;

std::mutex m_buf;
std::mutex m_raw_buf;

camodocal::CameraPtr m_camera;

int FREQ;
int EQUALIZE;
bool PUB_THIS_FRAME;

Eigen::Matrix3d RIC;
Eigen::Vector3d TIC;
double TD;

std::string VINS_NO_LOOP_RESULT_PATH;


void img_callback(StImagePtr img)
{
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img->timestamp;
        last_image_time = img->timestamp;
        return;
    }

    // detect unstable camera stream
    if (img->timestamp - last_image_time > 1.0 || img->timestamp < last_image_time)
    {
        //printf("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;


        //printf("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        estimator.clearState();
        estimator.setParameter(TIC, RIC, TD);
        current_time = -1;
        last_imu_t = 0;

		//new_sequence();

        return;
    }
    last_image_time = img->timestamp;

    PUB_THIS_FRAME = true;

    cv::Mat image(img->row, img->col, CV_8UC1);
    memcpy(image.data, img->data, img->row * img->col);

    TicToc t_r;
    trackerData.readImage(image, img->timestamp, PUB_THIS_FRAME, EQUALIZE);

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        completed |= trackerData.updateID(i);
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        pub_count++;

		StFeaturePointCloudPtr feature_points = std::make_shared<StFeaturePointCloud>();
		feature_points->timestamp = img->timestamp;

        auto &un_pts = trackerData.cur_un_pts;
        auto &cur_pts = trackerData.cur_pts;
        auto &ids = trackerData.ids;
        auto &pts_velocity = trackerData.pts_velocity;
        for (unsigned int j = 0; j < ids.size(); j++)
        {
            if (trackerData.track_cnt[j] > 1)
            {
				StFeaturePoint p;
				p.x = un_pts[j].x;
				p.y = un_pts[j].y;
				p.z = 1;
				p.id = ids[j];
				p.u = cur_pts[j].x;
				p.v = cur_pts[j].y;
				p.velocity_x = pts_velocity[j].x;
				p.velocity_y = pts_velocity[j].y;

                feature_points->points.push_back(p);
            }
        }

        printf("image at %f\n", feature_points->timestamp);
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else{

			if (!init_feature)
		    {
		        init_feature = 1;
		    } else {
			    m_buf.lock();
			    feature_buf.push(feature_points);
			    if(feature_buf.size() > 1)
			    	feature_buf.pop();
			    m_buf.unlock();
			    con.notify_one();
			}
		}

        /*if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                }
            }
            pub_match.publish(ptr->toImageMsg());
        }*/
    }
    printf("whole feature tracker processing costs: %f\n", t_r.toc());
}

void pubKeyframe(const Estimator &estimator)
{
    // pub camera pose, 2D-3D points of keyframe
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        
        ofstream foutC(VINS_NO_LOOP_RESULT_PATH, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(6);
        foutC << estimator.Headers[WINDOW_SIZE] << " ";
        foutC << estimator.Ps[WINDOW_SIZE].x() << " "
              << estimator.Ps[WINDOW_SIZE].y() << " "
              << estimator.Ps[WINDOW_SIZE].z() << " "
              << tmp_Q.x() << " "
              << tmp_Q.y() << " "
              << tmp_Q.z() << " "
              << tmp_Q.w() << endl;
        foutC.close();
	}
}


std::vector<std::pair<std::vector<StImuPtr>, StFeaturePointCloudPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<StImuPtr>, StFeaturePointCloudPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(imu_buf.back()->timestamp > feature_buf.front()->timestamp + estimator.td))
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            return measurements;
        }

        if (!(imu_buf.front()->timestamp < feature_buf.front()->timestamp + estimator.td))
        {
            //printf("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        StFeaturePointCloudPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<StImuPtr> IMUs;
        while (imu_buf.front()->timestamp < img_msg->timestamp + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            printf("no imu between two image\n");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}


void imu_callback(double & time_imu, float imu_data[])
{

	StImuPtr imu = std::make_shared<StImu>(imu_data[0],imu_data[1],imu_data[2],imu_data[3],imu_data[4],imu_data[5],time_imu);

    if (imu->timestamp <= last_imu_t)
    {
        //printf("imu message in disorder!");
        return;
    }
    last_imu_t = imu->timestamp;

    m_buf.lock();
    imu_buf.push(imu);
    m_buf.unlock();
    con.notify_one();

}

// thread: visual-inertial odometry
void process()
{
    while (true)
    {
        std::vector<std::pair<std::vector<StImuPtr>, StFeaturePointCloudPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg->timestamp;
                double img_t = img_msg->timestamp + estimator.td;
                if (t <= img_t)
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    assert(dt >= 0);
                    current_time = t;
                    dx = imu_msg->acc[0];
                    dy = imu_msg->acc[1];
                    dz = imu_msg->acc[2];
                    rx = imu_msg->gyr[0];
                    ry = imu_msg->gyr[1];
                    rz = imu_msg->gyr[2];
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    assert(dt_1 >= 0);
                    assert(dt_2 >= 0);
                    assert(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->acc[0];
                    dy = w1 * dy + w2 * imu_msg->acc[1];
                    dz = w1 * dz + w2 * imu_msg->acc[2];
                    rx = w1 * rx + w2 * imu_msg->gyr[0];
                    ry = w1 * ry + w2 * imu_msg->gyr[1];
                    rz = w1 * rz + w2 * imu_msg->gyr[2];
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            //printf("processing vision data with stamp %f \n", img_msg->timestamp);

            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->points[i].id;
                int feature_id = v;
                int camera_id = 0;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->points[i].u;
                double p_v = img_msg->points[i].v;
                double velocity_x = img_msg->points[i].velocity_x;
                double velocity_y = img_msg->points[i].velocity_y;
                assert(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            estimator.processImage(image, img_msg->timestamp);

            pubKeyframe(estimator);
        }
    }
}


void feature_tracking()
{

	StImagePtr p = nullptr;
	while(1){
		p = nullptr;
		m_raw_buf.lock();
		if(raw_image_buf.size() != 0){
			p = raw_image_buf.front();
			raw_image_buf.pop();
		}
		m_raw_buf.unlock();

		if(p != nullptr)
			img_callback(p);
			
		usleep(2*1000);

	}
}

void load_dataset(std::string & dataset_type, std::string & dataset_path)
{
	std::cout << "load " << dataset_type << " at: " << dataset_path << std::endl;
	
	std::shared_ptr<datasetloader::DatasetLoader> dataset;

	if(dataset_type == std::string("cla"))
		dataset.reset(new datasetloader::ClaLoader);
	else if(dataset_type == std::string("euroc"))
		dataset.reset(new datasetloader::EurocLoader);
		
	dataset->loadData(dataset_path);
	long num = dataset->getTotalDataNumber();

	for(long i=0; i<num; i++){
		datasetloader::DataType type = dataset->getNextDataType();
		
		if(type == datasetloader::TYPE_IMAGE){

			double time_sec;
			cv::Mat im;
			dataset->getNextImageData(time_sec, im);
			
			StImagePtr img = std::make_shared<StImage>(im.rows, im.cols, time_sec);
			memcpy(img->data, im.data, im.rows * im.cols);
			
			m_raw_buf.lock();
			raw_image_buf.emplace(img);
			if(raw_image_buf.size() > 1)
				raw_image_buf.pop();
			m_raw_buf.unlock();

		}else if(type == datasetloader::TYPE_IMU){
		
			double time_sec;
			float imu[6];
			dataset->getNextImuData(time_sec, imu);

			imu_callback(time_sec, imu);
		}
	}

}

int main(int argc, char **argv)
{
	if(argc != 2){
		printf("usage: vins_estimator config.yaml");
		return 0;
	}

    std::string config_file(argv[1]);

    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

	int MAX_CNT;
	int MIN_DIST;
	int ROW;
	int COL;
	double F_THRESHOLD;

    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    EQUALIZE = fsSettings["equalize"];

    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

	double FOCAL_LENGTH = m_camera->getFocalLength();

    trackerData.m_camera = m_camera;
	trackerData.max_cnt = MAX_CNT;
	trackerData.min_dist = MIN_DIST;
	trackerData.f_threshold = F_THRESHOLD;
	trackerData.focal_length = FOCAL_LENGTH;
	trackerData.row = ROW;
	trackerData.col = COL;

    //printf("ROW: %d COL: %d ", ROW, COL);


	double INIT_DEPTH = 5.0;
	double MIN_PARALLAX;
	double SOLVER_TIME;
	int NUM_ITERATIONS;
	int ESTIMATE_TD;

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    ESTIMATE_TD = fsSettings["estimate_td"];
    TD = fsSettings["td"];

	double ACC_N, ACC_W;
	double GYR_N, GYR_W;
	Eigen::Vector3d G{0,0,9.8};

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];

	int ESTIMATE_EXTRINSIC;
    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        RIC = Eigen::Matrix3d::Identity();
        TIC = Eigen::Vector3d::Zero();
    }
    else 
    {
        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC = eigen_R;
        TIC = eigen_T;
        //printf("Extrinsic_R : " << std::endl << RIC);
        //printf("Extrinsic_T : " << std::endl << TIC.transpose());
    } 

    estimator.setParameter(TIC, RIC, TD);
	estimator.setFeatureManager(INIT_DEPTH, MIN_PARALLAX, FOCAL_LENGTH);
	estimator.acc_n = ACC_N;
	estimator.acc_w = ACC_W;
	estimator.gyr_n = GYR_N;
	estimator.gyr_w = GYR_W;
	estimator.solver_time = SOLVER_TIME;
	estimator.num_iterations = NUM_ITERATIONS;
	estimator.estimate_td = ESTIMATE_TD;
	estimator.G = G;
	estimator.estimate_extrinsic = ESTIMATE_EXTRINSIC;

    std::thread measurement_process{process};

    VINS_NO_LOOP_RESULT_PATH = "./output/vins_result_no_loop.csv";

	std::string DATASET_PATH, DATASET_TYPE;
	fsSettings["dataset_type"] >> DATASET_TYPE;
	fsSettings["dataset_Path"] >> DATASET_PATH;


    fsSettings.release();

	std::thread feature_track(feature_tracking);

	load_dataset(DATASET_TYPE, DATASET_PATH);


    return 0;
}
