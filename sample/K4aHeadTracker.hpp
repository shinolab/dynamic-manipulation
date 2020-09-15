#include <k4a/k4a.hpp>
#include <k4abt.hpp>
#include <memory>
#include <Eigen/Geometry>

class K4aHeadTracker {
public:
	K4aHeadTracker(const Eigen::Vector3f& pos_device, const Eigen::Matrix3f& rot_device);
	static std::shared_ptr<K4aHeadTracker> Create(const Eigen::Vector3f& pos_device, const Eigen::Matrix3f& rot_device);
	void Open();
	void Close();
	bool TransformGlobal2Head(const Eigen::Vector3f& pos_in_global, Eigen::Vector3f& pos_in_head);
	bool TransformHead2Global(const Eigen::Vector3f& pos_in_head, Eigen::Vector3f& pos_in_global);
	bool GetHeadGeometry(Eigen::Vector3f& pos, Eigen::Quaternionf& quo);
private:
	Eigen::Affine3f m_affine_device;
	k4a::device m_device;
	k4abt::tracker m_tracker;
};