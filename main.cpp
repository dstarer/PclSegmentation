#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <pcl/point_representation.h>
#include <Eigen/Eigen>
#include <pcl/common/eigen.h>

typedef pcl::PointXYZ PointT;
struct Plane {
	Eigen::Vector3f normal;
	float d;
	Eigen::Vector3f getNormal() {
		return normal;
	}

	double distance(float x, float y, double z) {
		return normal[0] * x + normal[1] * y + normal[2] * z;
	}
};

bool pca(const Eigen::MatrixXf &x, Eigen::Matrix3f eigenVectors, Eigen::Vector3f &eigenVals);
/**
 * segmentation cloud
 * */
class FastSegmentation {
	class HeightCompartor {
		FastSegmentation* instance;
	public:
		HeightCompartor(FastSegmentation * segmentation) {
			instance = segmentation;
		}
		bool operator()(int _t1, int _t2) {
			return (*instance->inputCloudPtr)[_t1].z < (*instance->inputCloudPtr)[_t2].z;
		}
	};
public:

	void setInputCloud(pcl::PointCloud<PointT>::Ptr &ptr) {
		inputCloudPtr = ptr;
	}

	void setGroundThreshold(double v) {
		this->groundThresh = v;
	}

	void setNLR(int nlr) {
		this->nlr = nlr;
	}
	void setSeedsThreshold(double v) {
		this->seedsThresh = v;
	}
	void segmentation() {
		// sorted based on height z, using indices instead.
		std::sort(indicies.begin(), indicies.end(), new HeightCompartor(this));
		// select seeds
		selectSeeds();
		for (int i = 0; i < maxIter; ++i) {
			// estimate plane.
			// clear ground and non-ground.
			Plane model = estimateGroundPlane();
			groundIndices.clear();
			nonGroundIndices.clear();
			for (int j = 0; j < inputCloudPtr->size(); ++j) {
				PointT p = inputCloudPtr->at(j);
				double dist = model.distance(p.x, p.y, p.z);
				if (dist < this->groundThresh) {
					groundIndices.push_back(j);
				} else{
					nonGroundIndices.push_back(j);
				}
			}
		}
		// pub ground and non ground indices.
	}

protected:
	void selectSeeds() {
		this->groundIndices.clear();
		double average_height = 0;
		for (int i = 0; i < this->nlr; ++ i) {
			size_t ind = static_cast<size_t >(indicies[i]);
			average_height += inputCloudPtr->at(ind).z;
		}
		average_height /= this->nlr;
		for (int j = 0; j < this->indicies.size(); ++j) {
			size_t ind = static_cast<size_t >(indicies[j]);
			if (inputCloudPtr->at(ind).z < average_height + this->seedsThresh) {
				groundIndices.push_back(ind);
			}
		}
	}

	Plane estimateGroundPlane() {
		Eigen::Vector3d mean(0, 0, 0);
		for (int i = 0; i < groundIndices.size(); ++i) {
			auto ind = groundIndices[i];
			PointT p = inputCloudPtr->at(ind);
			mean[0] += p.x;
			mean[1] += p.y;
			mean[2] += p.z;
		}
		mean /= groundIndices.size();
		Eigen::Matrix3d covariance;
		covariance.setZero();
		for (int i = 0; i < groundIndices.size(); ++i) {
			auto ind = groundIndices[i];
			PointT p = inputCloudPtr->at(ind);
			Eigen::Vector3d q;
			q[0] = p.x - mean[0];
			q[1] = p.y - mean[1];
			q[2] = p.z - mean[2];
			covariance += q.transpose() * q;
		}
		covariance /= (groundIndices.size() - 1);
		// compute eigen values and eigen vectors, the first two.
		Eigen::MatrixXf eigenVectors;
		Eigen::Vector3f eigenVals;
		pca(covariance, eigenVectors, eigenVals);
		Eigen::Vector3f v1 = eigenVectors.col(0);
		Eigen::Vector3f v2 = eigenVectors.col(1);
		v1.cross(v2);
		Plane plane;
		plane.normal = v1;
	}

	pcl::PointCloud<PointT>::Ptr getGround() {

	}

private:
	int maxIter;
	pcl::PointCloud<PointT>::Ptr inputCloudPtr;
	pcl::PointCloud<PointT>::Ptr ringCloud;
	pcl::PointCloud<PointT>::Ptr obstacles;
	std::vector<int> indicies;
	std::vector<int> groundIndices;
	std::vector<int> nonGroundIndices;
	double groundThresh;
	double seedsThresh;
	int nlr;
};

int main() {

	return 0;
}