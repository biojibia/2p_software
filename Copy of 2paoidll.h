BOOL WINAPI DllMain(HINSTANCE hinstDLL, DWORD,LPVOID);
_declspec (dllexport) long  add_num(long, long);

// NI Macros
#define DAQmxErrChk(functionCall) { if( DAQmxFailed(error=(functionCall)) ) { goto Error; } }

// Settable program constants
#define PI						3.14159