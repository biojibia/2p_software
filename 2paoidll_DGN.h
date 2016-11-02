extern "C" {
BOOL WINAPI DllMain(HINSTANCE hinstDLL, DWORD,LPVOID);
//_declspec (dllexport) int32 backscan(int32 xPixels, int32 yPixels, double xZoom, double yZoom, int32 AIrate, int32 AOrate);
//_declspec (dllexport) void backscan_bin(int xPixels, int yPixels, int T, int bin, int manualShift, int16 *data0, int16 *data1, int16 *frame0, int16 *frame1);
//_declspec (dllexport) int32 backscan_raw(int32 xPixels, int32 yPixels, double xZoom, double yZoom, int32 AIrate, int32 AOrate);
//_declspec (dllexport) int32 backscan_mirror(int32 xPixels, int32 yPixels, double xZoom, double yZoom, int32 AIrate, int32 AOrate);
//_declspec (dllexport) int32 scan(int32 xPixels, int32 yPixels, double xZoom, double yZoom, int32 AIrate, int32 AOrate, int numChannels);
//_declspec (dllexport) void undistort(int xPixels, int yPixels, int T, int bin, int manualShift, int16 *data0, int16 *data1, int16 *xMirror, int16 *yMirror, int16 *frame0, int16 *frame1);


//_declspec (dllexport) long  add_num(long, long);
}
// NI Macros
#define DAQmxErrChk(functionCall) { if( DAQmxFailed(error=(functionCall)) ) { goto Error; } }

// Settable program constants
#define PI						3.14159

