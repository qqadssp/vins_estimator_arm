#ifndef DATASETLOADER_H
#define DATASETLOADER_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

namespace datasetloader
{

enum DataType
{
    TYPE_IMAGE = 0,
    TYPE_IMU
};

class DatasetLoader
{
public:

	virtual void loadData(std::string & datasetpath) = 0;
	
	virtual long getTotalDataNumber() = 0;

	virtual DataType getNextDataType() = 0;

	virtual void getNextImageData(double & time, cv::Mat & im) = 0;
	
	virtual void getNextImuData(double & time, float imu[]) = 0;

public:
	std::string dataset_path;
	std::vector<std::pair<long long, std::string>> pub;
	std::vector<std::pair<long long, std::string>>::iterator it;
	
	long long last_time;
};



class ClaLoader : public DatasetLoader
{
public:

	void loadData(std::string & datasetpath)
	{
		std::cout << "load dataset: " << datasetpath << std::endl;

		dataset_path = datasetpath;

		std::string img_file = dataset_path + "/img.txt";
		std::string imu_file = dataset_path + "/imu.txt";

		std::ifstream img_ifs(img_file);
		std::ifstream imu_ifs(imu_file);
	
		std::string tmp;
	
		pub.clear();

		long time;
		float imu_data[6];

		while(getline(img_ifs, tmp)){
			sscanf(tmp.c_str(), "%ld", &time);
			pub.emplace_back(std::pair<long, std::string>(time, tmp));
		}
	
		while(getline(imu_ifs, tmp)){
			sscanf(tmp.c_str(), "%ld %f %f %f %f %f %f", &time, imu_data, imu_data+1, imu_data+2, imu_data+3, imu_data+4, imu_data+5);
			pub.emplace_back(std::pair<long, std::string>(time, tmp));
		}

		sort(pub.begin(), pub.end(), [](auto & a, auto & b){return a.first < b.first;});

		it = pub.begin();
		last_time = 0;
	}
	
	long getTotalDataNumber()
	{
		return pub.size();
	}

	DataType getNextDataType()
	{
		if(it->second.size() < 20){
			return TYPE_IMAGE;
		}else{
			return TYPE_IMU;
		}
	}

	void getNextImageData(double & time, cv::Mat & im)
	{
		if(last_time != 0){
			usleep(it->first - last_time);
		}

		last_time = it->first;

		time = double(it->first) * 1e-6;

		std::string fname = dataset_path + "/cam0/" + std::to_string(it->first) + ".png";
		cv::Mat img = cv::imread(fname, cv::IMREAD_GRAYSCALE);
		img.copyTo(im);
		
		it++;
	}
	
	void getNextImuData(double & time, float imu[])
	{
		if(last_time != 0){
			usleep(it->first - last_time);
		}

		last_time = it->first;

		time = double(it->first) * 1e-6;

		long t;
		sscanf(it->second.c_str(), "%ld %f %f %f %f %f %f", &t, imu, imu+1, imu+2, imu+3, imu+4, imu+5);

		it++;
	}


};







class EurocLoader : public DatasetLoader
{
public:

	void loadData(std::string & datasetpath)
	{
		std::cout << "load dataset: " << datasetpath << std::endl;

		dataset_path = datasetpath;

		std::string img_file = dataset_path + "/cam0/data.csv";
		std::string imu_file = dataset_path + "/imu0/data.csv";

		std::ifstream img_ifs(img_file);
		std::ifstream imu_ifs(imu_file);

		std::string tmp;
	
		pub.clear();

		long long time;
		float imu_data[6];

		getline(img_ifs, tmp);
		while(getline(img_ifs, tmp)){
			for(int i=0; i<tmp.size(); i++)
				if(tmp[i] == ',')
					tmp[i] = ' ';
			printf("%s\n", tmp.c_str());
			sscanf(tmp.c_str(), "%lld %s", &time);
			pub.emplace_back(std::pair<long long, std::string>(time, tmp));
		}
		printf("total img %ld\n", pub.size());
		getline(imu_ifs, tmp);
		while(getline(imu_ifs, tmp)){
			for(int i=0; i<tmp.size(); i++)
				if(tmp[i] == ',')
					tmp[i] = ' ';
			sscanf(tmp.c_str(), "%lld %f %f %f %f %f %f", &time, imu_data, imu_data+1, imu_data+2, imu_data+3, imu_data+4, imu_data+5);
			pub.emplace_back(std::pair<long long, std::string>(time, tmp));
		}
		printf("total data %ld\n", pub.size());
		sort(pub.begin(), pub.end(), [](auto & a, auto & b){return a.first < b.first;});

		it = pub.begin();
		last_time = 0;
	}
	
	long getTotalDataNumber()
	{
		return pub.size();
	}

	DataType getNextDataType()
	{
		if(it->second.size() < 50){
			return TYPE_IMAGE;
		}else{
			return TYPE_IMU;
		}
	}

	void getNextImageData(double & time, cv::Mat & im)
	{
		if(last_time != 0){
			usleep((it->first/1000 - last_time)*10);
		}

		last_time = it->first/1000;

		time = double(it->first) * 1e-9;

		std::string fname = dataset_path + "/cam0/data/" + std::to_string(it->first) + ".png";
		cv::Mat img = cv::imread(fname, cv::IMREAD_GRAYSCALE);
		img.copyTo(im);
		
		it++;
	}
	
	void getNextImuData(double & time, float imu[])
	{
		if(last_time != 0){
			usleep((it->first/1000 - last_time)*10);
		}

		last_time = it->first/1000;

		time = double(it->first) * 1e-9;

		long t;
		sscanf(it->second.c_str(), "%ld %f %f %f %f %f %f", &t, imu+3, imu+4, imu+5, imu, imu+1, imu+2);

		it++;
	}

};

}

#endif
