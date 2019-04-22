#include "odcs.hpp"
#include <memory>

extern "C" {

	__declspec(dllexport) void* InitializeDynaman() {
		auto pOdcs = new odcs;
		pOdcs->Initialize();
		return (void*)pOdcs;
	}

	__declspec(dllexport) void DeleteDynaman(void* dynamanPtr) {
		delete (odcs*)dynamanPtr;
	}

	__declspec(dllexport) void StartControl(void* dynamanPtr) {
		((odcs*)dynamanPtr)->StartControl();
	}

	__declspec(dllexport) void CloseDynaman(void* dynamanPtr) {
		((odcs*)dynamanPtr)->Close();
	}

	__declspec(dllexport) void AddDevice(void* dynamanPtr, float x, float y, float z, float angle_z1, float angle_y2, float angle_z3) {
		((odcs*)dynamanPtr)->AddDevice(Eigen::Vector3f(x, y, z), Eigen::Vector3f(angle_z1, angle_y2, angle_z3));
	}

	__declspec(dllexport) void SetSensorGeometry(void* dynamanPtr, float x, float y, float z, float angle_z1, float angle_y2, float angle_z3) {
		((odcs*)dynamanPtr)->odsPtr->SetSensorGeometry(Eigen::Vector3f(x,y,z), Eigen::Vector3f(angle_z1, angle_y2, angle_z3));
	}

	__declspec(dllexport) void* CreateFloatingObject(float x, float y, float z, float weight, float radius) {
		auto pObjPtr = new FloatingObjectPtr(new FloatingObject(Eigen::Vector3f(x, y, z), weight, radius));
		return (void*)pObjPtr;
	}

	__declspec(dllexport) bool DeleteFloatingObject(void* objPtr) {
		if (objPtr == nullptr)
			return false;
		delete (FloatingObjectPtr*)objPtr;
		return true;
	}

	__declspec(dllexport) void RegisterFloatingObject(void* dynamanPtr, void* objPtr) {
		((odcs*)dynamanPtr)->RegisterObject(*(FloatingObjectPtr*)objPtr);
	}

	__declspec(dllexport) void MoveObjectFromTo(void* objPtr, float x0, float y0, float z0,
		float x1, float y1, float z1, float timeToGo, float timeInit) {
		(*(FloatingObjectPtr*)objPtr)->SetTrajectory(TrajectoryBangBang::Create(timeToGo, timeInit, Eigen::Vector3f(x0, y0, z0), Eigen::Vector3f(x1, y1, z1)));
	}

	__declspec(dllexport) void KeepObjectAt(void* objPtr, float x, float y, float z) {
		(*(FloatingObjectPtr*)objPtr)->updateStatesTarget(Eigen::Vector3f(x, y, z), Eigen::Vector3f(0.f, 0.f, 0.f));
	}

	__declspec(dllexport) void GetObjectPosition(void* objPtr, float* x, float* y, float* z) {
		Eigen::Vector3f pos = (*(FloatingObjectPtr*)objPtr)->getPosition();
		*x = pos.x();
		*y = pos.y();
		*z = pos.z();
	}
	
	__declspec(dllexport) void GetObjectPositionTarget(void* objPtr, float* x, float* y, float* z) {
		Eigen::Vector3f posTgt = (*(FloatingObjectPtr*)objPtr)->getPositionTarget();
		*x = posTgt.x();
		*y = posTgt.y();
		*z = posTgt.z();
	}
}