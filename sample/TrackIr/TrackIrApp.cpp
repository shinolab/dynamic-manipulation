#include <Windows.h>
#include "resource.h"
#include "NPClient.h"
#include "NPClientWraps.h"
#include "glRenderer.h"
#include <tchar.h>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

#define NP_DEVELOPER_ID 1000
const float np_max_rotation = 180.0f;
const float np_max_translation = 50.0f;
const float np_max_value = 16383;
const float np_min_value = -16383;

#define LATENCY_SAMPLES 0
#define LATENCY_SAMPLES 100
unsigned long NPFrameSignature = 0;
unsigned long NPStaleFrames = 0;
double dSampleDeltaAverage;
char szVersion[MAX_PATH] = {""};
char szStatus[MAX_PATH] = {""};
double NPLatencyAverage = 0.0;
double NPLatencySamples[LATENCY_SAMPLES];
int iLatencyIndex = 0;
double fLast = 0.0;
int index;
long iMemTest = 0;
//application global parameters
HWND hWnd;
HINSTANCE hInst;
glRenderer* g_pRenderer;
int LastMouseDownX = 0;
int LastMouseDownY = 0;
const int Timer_ID = 100;

bool TrackIrInit() {
	NPRESULT res;
	NPFrameSignature = 0;
	NPStaleFrames = 0;

	TCHAR szPath[2 * MAX_PATH];
	HKEY pKey = NULL;
	LPTSTR pszValue;
	DWORD dwSize;
	if (::RegOpenKeyEx(HKEY_CURRENT_USER, _T("Software\\NaturalPoint\\NATURALPOINT\\NPClient Location"), 0, KEY_READ, &pKey) != ERROR_SUCCESS) {
		std::cerr << "ERROR: DLL Location Key not present." << std::endl;
		return false;
	}
	if (RegQueryValueEx(pKey, _T("Path"), NULL, NULL, NULL, &dwSize)) {
		std::cerr << "Path value not present." << std::endl;
		return false;
	}
	pszValue = (LPTSTR)malloc(dwSize);
	if (pszValue == NULL) {
		std::cerr << "Insufficient memory." << std::endl;
		return false;
	}
	if (RegQueryValueEx(pKey, _T("Path"), NULL, NULL, (LPBYTE)pszValue, &dwSize) != ERROR_SUCCESS) {
		::RegCloseKey(pKey);
		std::cerr << "Error: cannot read location key" << std::endl;
		return false;
	}
	else {
		::RegCloseKey(pKey);
		_tcscpy_s(szPath, pszValue);
		free(pszValue);
	}
	res = NPClient_Init(szPath);
	if (res != NP_OK) {
		std::cerr << "Error: failed to initialize NPClient interface. (Error code: " << res << ")" << std::endl;
		return false;
	}
	res = NP_RegisterWindowHandle(hWnd);
	if (res != NP_OK) {
		std::cerr << "Error: failed to register window handle" << std::endl;
		return false;
	}
	unsigned short wNpClientVer;
	res = NP_QueryVersion(&wNpClientVer);
	if (res == NP_OK) {
		std::cout << "VERSION: " << (wNpClientVer >> 8) << ", " << (wNpClientVer & 0x00FF) << std::endl;
	}
	else {
		std::cerr << "NPClient : Failed to get Np version." << std::endl;
	}

	unsigned int dataFields = 0;
	dataFields |= NPPitch;
	dataFields |= NPYaw;
	dataFields |= NPRoll;
	dataFields |= NPX;
	dataFields |= NPY;
	dataFields |= NPZ;
	NP_RequestData(dataFields);
	NP_RegisterProgramProfileID(NP_DEVELOPER_ID);
	res = NP_StopCursor();
	if (res != NP_OK) {
		std::cerr << "Failed to stop cursor." << std::endl;
	}
	res = NP_StartDataTransmission();
	if (res != NP_OK) {
		std::cerr << "Failed to start data transmission." << std::endl;
	}
	return true;
}

float ConvertTrackIrTranslationToFloat(float trackIrValue) {
	return trackIrValue * np_max_translation / np_max_value;
}

float ConvertTrackIrRotationToFloat(float trackIrValue) {
	return trackIrValue * np_max_rotation / np_max_value;
}

void TrackIrShutdown() {
	NP_StopDataTransmission();
	NP_UnregisterWindowHandle();
}

NPRESULT TrackIrHandleTrackData() {
	TRACKIRDATA trackIrData;
	NPRESULT res = NP_GetData(&trackIrData);

	if (res == NP_OK) {
		if (trackIrData.wNPStatus == NPSTATUS_REMOTEACTIVE) {
			if (NPFrameSignature != trackIrData.wPFrameSignature) {
				if (g_pRenderer) {
					float x = trackIrData.fNPX * np_max_translation / np_max_value;
					float y = trackIrData.fNPY * np_max_translation / np_max_value;
					float z = trackIrData.fNPZ * np_max_translation / np_max_value;

					float yaw = ConvertTrackIrRotationToFloat(trackIrData.fNPYaw);
					float pitch = ConvertTrackIrRotationToFloat(trackIrData.fNPPitch);
					float roll = ConvertTrackIrRotationToFloat(trackIrData.fNPRoll);

					g_pRenderer->tirData.x = x;
					g_pRenderer->tirData.y = y;
					g_pRenderer->tirData.z = z;
					g_pRenderer->tirData.yaw = yaw;
					g_pRenderer->tirData.pitch = pitch;
					g_pRenderer->tirData.roll = roll;
				}
				NPFrameSignature = trackIrData.wPFrameSignature;
				NPStaleFrames = 0;

			}
			else {
				if (NPStaleFrames > 30) {
					std::cout << "No new data." << std::endl;
				}
				else {
					NPStaleFrames++;
					std::cout << "Not tracking for " << NPStaleFrames << "Frames" << std::endl;
				}
				res = NP_ERR_NO_DATA;
			}
		}
		else {
			res = NP_ERR_NO_DATA;
		}
	}
	return res;
}

LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY _tWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPTSTR lpCmdLine, int cCmdshow) {
	
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);
	MSG msg;
	hInst = hInstance;
	WNDCLASSEX wcex;
	wcex.cbSize = sizeof(WNDCLASSEX);
	wcex.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
	wcex.lpfnWndProc = WndProc;
	wcex.cbClsExtra = 0;
	wcex.cbWndExtra = 0;
	wcex.hInstance = hInstance;
	wcex.hIcon = LoadIcon(NULL, IDC_ARROW);
	wcex.hCursor = LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
	wcex.lpszMenuName = MAKEINTRESOURCE(IDC_TRACKIRCLIENTOGL);
	wcex.lpszClassName = _T("TrackIrTest");
	wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));
	RegisterClassEx(&wcex);
	hWnd = CreateWindow(
		_T("TrackIrTest"), _T("TrackIrTest"), WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, 0,
		CW_USEDEFAULT, 0, NULL, NULL, hInstance, NULL);
	if (!hWnd) {
		return 0;
	}
	SetTimer(hWnd, Timer_ID, 10, (TIMERPROC)NULL);

	g_pRenderer = new glRenderer();
	auto bSuccess = g_pRenderer->Initialize(hWnd);
	glSphere* pSphere = new glSphere();
	g_pRenderer->m_RenderList3D.push_back(pSphere);

	bSuccess = TrackIrInit();
	if (!bSuccess) {
		std::cerr << "Failed to initialize Track IR." << std::endl;
	}
	ShowWindow(hWnd, cCmdshow);
	UpdateWindow(hWnd);
	
	HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_TRACKIRCLIENTOGL));
	while (GetMessage(&msg, NULL, 0, 0)) {
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg)) {
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	return (int) msg.wParam;
}

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam) {
	int wmId, wmEvent;
	PAINTSTRUCT ps;
	HDC hdc;
	switch (message) {
	case WM_COMMAND:
		wmId = LOWORD(wParam);
		wmEvent = HIWORD(wParam);
		switch (wmId) {
		case IDM_ABOUT:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
			break;
		case IDM_EXIT:
			DestroyWindow(hWnd);
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
		break;
	case WM_TIMER:
		if (wParam == Timer_ID) {
			NPRESULT result = TrackIrHandleTrackData();
			g_pRenderer->Render();
		}
		break;
	case WM_PAINT:
		hdc = BeginPaint(hWnd, &ps);
		if (g_pRenderer) {
			g_pRenderer->Render();
		}
		EndPaint(hWnd, &ps);
		break;
	case WM_ERASEBKGND:
		break;
	case WM_SIZE:
		if (g_pRenderer)
			g_pRenderer->ResizeWindow(LOWORD(lParam), HIWORD(lParam));
		break;
	case WM_LBUTTONDOWN:
		LastMouseDownX = LOWORD(lParam);
		LastMouseDownY = HIWORD(lParam);
		break;
	case WM_MOUSEWHEEL:
		if(g_pRenderer){
			float zDelta = GET_WHEEL_DELTA_WPARAM(wParam);
			float fNearLimitZ = -50.0;
			float fFarLimitZ = 2000.0;
			float fScale = 10.0f;
			float nNotches = -1.0f * (float)zDelta / (float)WHEEL_DELTA;
			float fNewZ = (float)g_pRenderer->m_pView->EyeDistance + (nNotches * fScale);
			if (fNewZ <= fFarLimitZ && fNewZ >= fNearLimitZ) {
				g_pRenderer->m_pView->EyeDistance = fNewZ;
				g_pRenderer->m_bTrackingView = false;
				g_pRenderer->Render();
			}
		}
		break;
	case WM_MOUSEMOVE:
		if (wParam & MK_LBUTTON) {
			// orbit the viewpoint
			int x = LOWORD(lParam);
			int y = HIWORD(lParam);
			float fYawScale = 0.3f;
			float fPitchScale = 0.3f;
			float fYawChange = (x - LastMouseDownX) * fYawScale;
			float fPitchChange = (y - LastMouseDownY) * fPitchScale;
			g_pRenderer->m_pView->EyeAzimuth += fYawChange;
			g_pRenderer->m_pView->EyeElevation += fPitchChange;
			g_pRenderer->Render();
			LastMouseDownX = x;
			LastMouseDownY = y;
		}
		break;
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}