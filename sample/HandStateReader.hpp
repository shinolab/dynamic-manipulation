#ifndef _DYNAMAN_HANDSTATE_READER_HPP
#define _DYNAMAN_HANDSTATE_READER_HPP

#include <utility>
#include "pcl_grabber.hpp"
#include "FloatingObject.hpp"

namespace dynaman {
	enum class HandState {
		NONCONTACT, TOUCH, HOLD_FINGER_UP, HOLD_FINGER_DOWN
	};

	class HandStateReader {
	public:
		virtual ~HandStateReader() {}
		virtual bool Read(dynaman::HandState &state) = 0;
	};

	class PclHandStateReader : public HandStateReader {
	public:
		PclHandStateReader(
			dynaman::FloatingObjectPtr pObject,
			std::shared_ptr<pcl_grabber> pPclGrabber
		);

		~PclHandStateReader();

		std::shared_ptr<HandStateReader> Create(
			dynaman::FloatingObjectPtr pObject,
			std::shared_ptr<pcl_grabber> pPclGrabber
		);

		void Initialize();
		bool Read(dynaman::HandState &state) override;

	private:
		dynaman::FloatingObjectPtr m_pObject;
		std::shared_ptr<pcl_grabber> m_PclGrabber;
		float m_radiusObject;
		float m_radiusColliderContact;
		float m_radiusColliderClick;
	};

}
#endif