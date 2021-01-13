#ifndef _DYNAMAN_HANDSTATE_READER_HPP
#define _DYNAMAN_HANDSTATE_READER_HPP

#include <utility>
#include "FloatingObject.hpp"
#include "pcl_grabber.hpp"
#include "pcl_util.hpp"
#include "pcl_viewer.hpp"
#include "state_type.hpp"

namespace dynaman {

	class HandStateReader {
	public:
		virtual ~HandStateReader() {}
		virtual bool initialize() = 0;
		virtual bool Read(dynaman::HandState &state) = 0;
	};

	class PclHandStateReader : public HandStateReader {
	public:
		PclHandStateReader(
			dynaman::FloatingObjectPtr pObject,
			std::shared_ptr<pcl_grabber> pPclGrabber
		);

		~PclHandStateReader();

		static std::shared_ptr<HandStateReader> Create(
			dynaman::FloatingObjectPtr pObject,
			std::shared_ptr<pcl_grabber> pPclGrabber
		);

		pcl_util::pcl_ptr DefaultPreprocess(pcl_util::pcl_ptr pCloud);

		bool initialize() override;

		//Estimate the radius of a sphere based on its center and a point cloud.
		//The radius is estimated using the distance between the center and the furthest point of the nearest cluster
		bool EstimateSphereRadius(const Eigen::Vector3f& centerSphere, pcl_util::pcl_ptr pCloud, float &radius);
		bool EstimateHandState(
			dynaman::HandState& state,
			const Eigen::Vector3f& center, 
			pcl_util::pcl_ptr pCloud
		);
		float RadiusObject();
		float RadiusColliderContact();
		float RadiusColliderClick();
		bool Read(dynaman::HandState &state) override;

	private:
		dynaman::FloatingObjectPtr m_pObject;
		std::shared_ptr<pcl_grabber> m_pPclGrabber;
		float m_radiusObject;
		float m_radiusColliderContact;
		float m_radiusColliderClick;
	};

}
#endif