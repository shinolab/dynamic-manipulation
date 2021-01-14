#ifndef _DYNAMAN_HANDSTATE_READER_HPP
#define _DYNAMAN_HANDSTATE_READER_HPP

#include <utility>
#include "FloatingObject.hpp"
#include "pcl_grabber.hpp"
#include "pcl_util.hpp"
#include "state_type.hpp"

namespace dynaman {

	class PclHandStateReader {
	public:
		PclHandStateReader(float radius);

		~PclHandStateReader();

		static std::shared_ptr<PclHandStateReader> Create(float radius);

		pcl_util::pcl_ptr DefaultPreprocess(pcl_util::pcl_ptr pCloud, FloatingObjectPtr pObject);

		//Estimate the radius of a sphere based on its center and a point cloud.
		//The radius is estimated using the distance between the center and the furthest point of the nearest cluster
		bool EstimateSphereRadius(pcl_util::pcl_ptr pCloud, const Eigen::Vector3f& centerSphere);
		bool EstimateHandState(
			dynaman::HandState& state,
			const Eigen::Vector3f& center, 
			pcl_util::pcl_ptr pCloud
		);

		pcl_util::pcl_ptr ExtractPointsInsideColliderClick(pcl_util::pcl_ptr pCloud, const Eigen::Vector3f& center);

		float RadiusObject();
		float RadiusColliderContact();
		float RadiusColliderClick();
		bool Read(dynaman::HandState &state, pcl_util::pcl_ptr pCloud, const Eigen::Vector3f& center);

	private:
		float m_radiusObject;
		float m_radiusColliderContact;
		float m_radiusColliderClick;
	};

}
#endif