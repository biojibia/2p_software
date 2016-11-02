/* 2paoi.c source code */

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

// National Instruments Headers
#include "NIDAQmx.h"

#include "2paoidll_DGN.h"


BOOL WINAPI DllMain(
    HINSTANCE hinstDLL,  // handle to DLL module
    DWORD fdwReason,     // reason for calling function
    LPVOID lpReserved )  // reserved
{
    // Perform actions based on the reason for calling.
    switch( fdwReason ) 
    {
        case DLL_PROCESS_ATTACH:
         // Initialize once for each new process.
         // Return FALSE to fail DLL load.
            break;

        case DLL_THREAD_ATTACH:
         // Do thread-specific initialization.
            break;

        case DLL_THREAD_DETACH:
         // Do thread-specific cleanup.
            break;

        case DLL_PROCESS_DETACH:
         // Perform any necessary cleanup.
            break;
    }
    return TRUE;  // Successful DLL_PROCESS_ATTACH.
}



	/* Scan */
	_declspec (dllexport) int32 scan(int32 xPixels, int32 yPixels, double xZoom, double yZoom, int32 AIrate, int32 AOrate, int numChannels)
	{
		// DAQ Handles
		TaskHandle AItaskHandle = 0;
		TaskHandle AOtaskHandle = 0;

		// Local Variables 
		int32			error = 0;
		char			AIchannels[100] = "Dev1/ai0:1\0";				// Always two channels
		double			*waveform = NULL;
		char			errBuff[2048] = { '\0' };

		// Scan Parameters
		int32		numPixels = 0, numScans = 0;
		int32		written = 0;
		int32		read = 0;
		int32		bin = 1;

		// Waveform Variables
		int		i = 0, j = 0;
		double	Tmin = .0007;   	// Minimum Slew Speed (700us)
		double	S, Sy, C;
		int		P;

		// Set channel Number
		switch (numChannels)
		{
		case 1:
			sprintf_s(AIchannels, "Dev1/ai0\0");
			break;
		case 2:
			sprintf_s(AIchannels, "Dev1/ai0:1\0");
			break;
		case 3:
			sprintf_s(AIchannels, "Dev1/ai0:2\0");
			break;
		case 4:
			sprintf_s(AIchannels, "Dev1/ai0:3\0");
			break;
		default:
			sprintf_s(AIchannels, "Dev1/ai0\0:1");
			break;
		}

		// Set SCAN parameters
		bin = (int)AIrate / AOrate;
		P = (int)((double)Tmin*AOrate);	// Number of pixels in fastest flyback
		numPixels = (xPixels + P)*yPixels;
		numScans = bin*numPixels;
		S = 2.0 / xPixels;	// Slope
		Sy = 2.0 / numPixels;	// Yslope
		C = -2.0*(S*(P / 2.0) + 1.0) / ((P / 2.0)*(P / 2.0));		// Acceleration (Phase 2)

		// allocate memory
		waveform = (double *)malloc(sizeof(double)*numPixels * 2);

		// AI
		DAQmxErrChk(DAQmxCreateTask("AI", &AItaskHandle));
		DAQmxErrChk(DAQmxCreateAIVoltageChan(AItaskHandle, AIchannels, "AI", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(AItaskHandle, "", (float)AIrate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, numScans));
		DAQmxErrChk(DAQmxSetAIDataXferMech(AItaskHandle, AIchannels, DAQmx_Val_DMA));
		DAQmxErrChk(DAQmxCfgDigEdgeStartTrig(AItaskHandle, "ao/StartTrigger", DAQmx_Val_Rising));

		// AO
		DAQmxErrChk(DAQmxCreateTask("AO", &AOtaskHandle));
		DAQmxErrChk(DAQmxCreateAOVoltageChan(AOtaskHandle, "Dev1/ao0:1", "AO", -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(AOtaskHandle, "", (float)AOrate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, numPixels));

		// Make X Waveform
		for (j = 0; j < numPixels; j += (xPixels + P))
		{
			//Phase 1: X
			for (i = 0; i < xPixels; i++)
			{
				waveform[i + j] = (S*i - 1.0);
			}

			//Phase 2: X
			for (i = 0; i < (P / 2); i++)
			{
				waveform[i + xPixels + j] = (((C / 2)*i*i) + S*i + 1.0);
			}

			// Phase 3: X
			for (i = 0; i < (P / 2); i++)
			{
				waveform[i + xPixels + (P / 2) + j] = (-1.0*waveform[xPixels + (P / 2) - i - 1 + j]);
			}
		}

		// Make Y Waveform
		// Phase 1: Y						// Y mirror follows a sawtooth
		for (i = 0; i < numPixels; i++)
		{
			waveform[i + numPixels] = (Sy*i - 1.0);
		}

		// Set ZoomX,Y
		for (i = 0; i < numPixels; i++)
		{
			waveform[i] *= xZoom;
			waveform[i + numPixels] *= yZoom;

		}

		// Write Scan Waveform
		DAQmxErrChk(DAQmxWriteAnalogF64(AOtaskHandle, numPixels, 0, 10.0, DAQmx_Val_GroupByChannel, waveform, &written, NULL));


	Error:
		if (DAQmxFailed(error))
		{
			DAQmxGetExtendedErrorInfo(errBuff, 2048);
			printf("DAQmx Error: %s\n", errBuff);
		}

		free(waveform);
		return numPixels;
	}
	/* Forward Scan Only */
	_declspec (dllexport) int32 forscan(int32 xPixels, int32 yPixels, double xZoom, double yZoom, int32 AIrate, int32 AOrate)
	{
		// DAQ Handles
		TaskHandle AItaskHandle = 0;
		TaskHandle AOtaskHandle = 0;

		// Local Variables 
		int32			error = 0;
		char			AIchannels[100] = "Dev1/ai0:1\0";				// Always two channels
		double			*waveform = NULL;
		char			errBuff[2048] = { '\0' };

		// Scan Parameters
		int32		numPixels = 0, numScans = 0;
		int32		written = 0;
		int32		read = 0;
		int32		bin = 1;

		// Waveform Variables
		int		i = 0, j = 0;
		double	Tmin = .0007;   	// Minimum Slew Speed (700us)
		double	S, Sy, C;
		int		P;
			

		// Set SCAN parameters
		bin = (int)AIrate / AOrate;
		P = (int)((double)Tmin*AOrate);	// Number of pixels in fastest flyback
		numPixels = (xPixels + P)*yPixels;
		numScans = bin*numPixels;
		S = 2.0 / xPixels;	// Slope
		Sy = 2.0 / numPixels;	// Yslope
		C = -2.0*(S*(P / 2.0) + 1.0) / ((P / 2.0)*(P / 2.0));		// Acceleration (Phase 2)

																	// allocate memory
		waveform = (double *)malloc(sizeof(double)*numPixels * 2);

		// AI
		DAQmxErrChk(DAQmxCreateTask("AI", &AItaskHandle));
		DAQmxErrChk(DAQmxCreateAIVoltageChan(AItaskHandle, AIchannels, "AI", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(AItaskHandle, "", (float)AIrate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, numScans));
		DAQmxErrChk(DAQmxSetAIDataXferMech(AItaskHandle, AIchannels, DAQmx_Val_DMA));
		DAQmxErrChk(DAQmxCfgDigEdgeStartTrig(AItaskHandle, "ao/StartTrigger", DAQmx_Val_Rising));

		// AO
		DAQmxErrChk(DAQmxCreateTask("AO", &AOtaskHandle));
		DAQmxErrChk(DAQmxCreateAOVoltageChan(AOtaskHandle, "Dev1/ao0:1", "AO", -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(AOtaskHandle, "", (float)AOrate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, numPixels));

		// Make X Waveform
		for (j = 0; j < numPixels; j += (xPixels + P))
		{
			//Phase 1: X
			for (i = 0; i < xPixels; i++)
			{
				waveform[i + j] = (S*i - 1.0);
			}

			//Phase 2: X
			for (i = 0; i < (P / 2); i++)
			{
				waveform[i + xPixels + j] = (((C / 2)*i*i) + S*i + 1.0);
			}

			// Phase 3: X
			for (i = 0; i < (P / 2); i++)
			{
				waveform[i + xPixels + (P / 2) + j] = (-1.0*waveform[xPixels + (P / 2) - i - 1 + j]);
			}
		}

		// Make Y Waveform
		// Phase 1: Y						// Y mirror follows a sawtooth
		for (i = 0; i < numPixels; i++)
		{
			waveform[i + numPixels] = (Sy*i - 1.0);
		}

		// Set ZoomX,Y
		for (i = 0; i < numPixels; i++)
		{
			waveform[i] *= xZoom;
			waveform[i + numPixels] *= yZoom;

		}


		//FILE *debug = NULL;
		//debug = fopen("E:\\Documents\\Drago_VIs\\waveform.txt", "w");
		//for (i = 0; i < numPixels; i++)
		//{
		//	fprintf(debug, "%f %f\n", waveform[i], waveform[i + numPixels]);
		//}
		//fclose(debug);


		// Write Scan Waveform
		DAQmxErrChk(DAQmxWriteAnalogF64(AOtaskHandle, numPixels, 0, 10.0, DAQmx_Val_GroupByChannel, waveform, &written, NULL));


	Error:
		if (DAQmxFailed(error))
		{
			DAQmxGetExtendedErrorInfo(errBuff, 2048);
			printf("DAQmx Error: %s\n", errBuff);
		}

		free(waveform);
		return P;
	}
	/* Forward Scan with gating */
	_declspec (dllexport) int32 forscan_gate(int32 xPixels, int32 yPixels, double xZoom, double yZoom, int32 AIrate, int32 AOrate, int32 delay)
	{
		// DAQ Handles
		TaskHandle AItaskHandle = 0;
		TaskHandle AOtaskHandle = 0;
		//TaskHandle DOtaskHandle = 0;
		TaskHandle COtaskHandle = 0;
		TaskHandle CO2taskHandle = 0;

		// Local Variables 
		int32			error = 0;
		char			AIchannels[100] = "Dev1/ai0:1\0";				// Always two channels
		double			*waveform = NULL;
		//uInt8			*DO_gate = NULL;
		char			errBuff[2048] = { '\0' };

		// Scan Parameters
		int32		numPixels = 0, numScans = 0;
		int32		written = 0;
		//int32		written2 = 0;
		int32		read = 0;
		int32		bin = 1;

		// Waveform Variables
		int		i = 0, j = 0;
		double	Tmin = .0007;   	// Minimum Slew Speed (700us)
		double	S, Sy, C;
		int		P;


		// Set SCAN parameters
		bin = (int)AIrate / AOrate;
		P = (int)((double)Tmin*AOrate);	// Number of pixels in fastest flyback
		numPixels = (xPixels + P)*yPixels;
		numScans = bin*numPixels;
		S = 2.0 / xPixels;	// Slope
		Sy = 2.0 / numPixels;	// Yslope
		C = -2.0*(S*(P / 2.0) + 1.0) / ((P / 2.0)*(P / 2.0));		// Acceleration (Phase 2)

																	// allocate memory
		waveform = (double *)malloc(sizeof(double)*numPixels * 2);

		//CO, gate for the actual PMT, normally OFF
		DAQmxErrChk(DAQmxCreateTask("CO", &COtaskHandle));
		//DAQmxErrChk(DAQmxCreateCOPulseChanTicks(COtaskHandle, "Dev1/ctr0", "CO", "/Dev1/ao/SampleClock", DAQmx_Val_Low, 0, P, xPixels));
		DAQmxErrChk(DAQmxCreateCOPulseChanTicks(COtaskHandle, "Dev1/ctr0", "CO", "/Dev1/ao/SampleClock", DAQmx_Val_Low, xPixels, xPixels, P));
		DAQmxErrChk(DAQmxCfgImplicitTiming(COtaskHandle, DAQmx_Val_ContSamps, numPixels));

		//CO2, gate for the projector
		DAQmxErrChk(DAQmxCreateTask("CO2", &CO2taskHandle));
		//Delay centered always at the middle of the interval
		//DAQmxErrChk(DAQmxCreateCOPulseChanTicks(CO2taskHandle, "Dev1/ctr1", "CO2", "/Dev1/ao/SampleClock", DAQmx_Val_Low, xPixels + (delay / 2), xPixels + delay, P - delay));
		//Delay starting always at the beginning of the interval
		DAQmxErrChk(DAQmxCreateCOPulseChanTicks(CO2taskHandle, "Dev1/ctr1", "CO2", "/Dev1/ao/SampleClock", DAQmx_Val_Low, xPixels, xPixels + delay, P - delay));
		//Inverted gating (for use with NAND gate)
		//DAQmxErrChk(DAQmxCreateCOPulseChanTicks(CO2taskHandle, "Dev1/ctr1", "CO2", "/Dev1/ao/SampleClock", DAQmx_Val_Low, P + xPixels - delay, P - delay, xPixels + delay));
		DAQmxErrChk(DAQmxCfgImplicitTiming(CO2taskHandle, DAQmx_Val_ContSamps, numPixels));

		// AI
		DAQmxErrChk(DAQmxCreateTask("AI", &AItaskHandle));
		DAQmxErrChk(DAQmxCreateAIVoltageChan(AItaskHandle, AIchannels, "AI", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(AItaskHandle, "", (float)AIrate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, numScans));
		DAQmxErrChk(DAQmxSetAIDataXferMech(AItaskHandle, AIchannels, DAQmx_Val_DMA));
		DAQmxErrChk(DAQmxCfgDigEdgeStartTrig(AItaskHandle, "ao/StartTrigger", DAQmx_Val_Rising));

		// AO
		DAQmxErrChk(DAQmxCreateTask("AO", &AOtaskHandle));
		DAQmxErrChk(DAQmxCreateAOVoltageChan(AOtaskHandle, "Dev1/ao0:1", "AO", -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(AOtaskHandle, "", (float)AOrate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, numPixels));

		
		// Make X Waveform
		for (j = 0; j < numPixels; j += (xPixels + P))
		{
			//Phase 1: X
			for (i = 0; i < xPixels; i++)
			{
				waveform[i + j] = (S*i - 1.0);
			}

			//Phase 2: X
			for (i = 0; i < (P / 2); i++)
			{
				waveform[i + xPixels + j] = (((C / 2)*i*i) + S*i + 1.0);
			}

			// Phase 3: X
			for (i = 0; i < (P / 2); i++)
			{
				waveform[i + xPixels + (P / 2) + j] = (-1.0*waveform[xPixels + (P / 2) - i - 1 + j]);
			}
		}

		// Make Y Waveform
		// Phase 1: Y						// Y mirror follows a sawtooth
		for (i = 0; i < numPixels; i++)
		{
			waveform[i + numPixels] = (Sy*i - 1.0);
		}

		// Set ZoomX,Y
		for (i = 0; i < numPixels; i++)
		{
			waveform[i] *= xZoom;
			waveform[i + numPixels] *= yZoom;

		}

		// Save Waveform (for debugging purposes)
			//FILE *debug = NULL;
			//debug = fopen("E:\\Documents\\Drago_VIs\\gating.txt", "w");
			//for(i = 0; i < numPixels; i ++)
			//{
			//	fprintf(debug, "%f %f %u\n", waveform[i], waveform[i + numPixels], DO_gate[i]);
			//	//fprintf(debug, "%u\n", DO_gate[i]);
			//}
			//fclose(debug);
		
		// Write Scan Waveform
		//DAQmxErrChk(DAQmxWriteDigitalLines(DOtaskHandle, numPixels, 1, 10.0, DAQmx_Val_GroupByChannel, DO_gate, &written2, NULL));
		DAQmxErrChk(DAQmxWriteAnalogF64(AOtaskHandle, numPixels, 0, 10.0, DAQmx_Val_GroupByChannel, waveform, &written, NULL));


	Error:
		if (DAQmxFailed(error))
		{
			DAQmxGetExtendedErrorInfo(errBuff, 2048);
			printf("DAQmx Error: %s\n", errBuff);
		}

		free(waveform);
		//free(DO_gate);
		return P;
	}
	/* Forward Scan with gating and  imp waveform*/
	_declspec (dllexport) int32 forscan_gate2(int32 xPixels, int32 yPixels, double xZoom, double yZoom, 
		int32 AIrate, int32 AOrate, int32 delay, double Tprop, int32 manShift)
	{
		// DAQ Handles
		TaskHandle AItaskHandle = 0;
		TaskHandle AOtaskHandle = 0;
		//TaskHandle DOtaskHandle = 0;
		TaskHandle COtaskHandle = 0;
		TaskHandle CO2taskHandle = 0;

		// Local Variables 
		int32			error = 0;
		char			AIchannels[100] = "Dev1/ai0:1\0";				// Always two channels
		double			*waveform = NULL;
		char			errBuff[2048] = { '\0' };

		// Scan Parameters
		int32		numPixels = 0, numScans = 0;
		int32		written = 0;
		int32		read = 0;
		int32		bin = 1;

		// Waveform Variables
		int		i = 0, j = 0;
		double	S, Sy, Sf;
		int		P;


		// Set SCAN parameters
		bin = (int)AIrate/AOrate;
		P = (int)ceil(Tprop*xPixels);
		numPixels = (xPixels + P)*yPixels;
		numScans = bin*numPixels;
		S = 2.0 / xPixels;	// Slope
		Sy = 2.0 / numPixels;	// Yslope
		Sf = -2.0 / P;
																	// allocate memory
		waveform = (double *)malloc(sizeof(double)*numPixels * 2);

		//CO, gate for the actual PMT, normally OFF
		DAQmxErrChk(DAQmxCreateTask("CO", &COtaskHandle));
		//DAQmxErrChk(DAQmxCreateCOPulseChanTicks(COtaskHandle, "Dev1/ctr0", "CO", "/Dev1/ao/SampleClock", DAQmx_Val_Low, 0, P, xPixels));
		DAQmxErrChk(DAQmxCreateCOPulseChanTicks(COtaskHandle, "Dev1/ctr0", "CO", "/Dev1/ao/SampleClock", DAQmx_Val_Low, xPixels + manShift, xPixels, P));
		DAQmxErrChk(DAQmxCfgImplicitTiming(COtaskHandle, DAQmx_Val_ContSamps, numPixels));

		//CO2, gate for the projector
		DAQmxErrChk(DAQmxCreateTask("CO2", &CO2taskHandle));
		//Delay centered always at the middle of the interval
		//DAQmxErrChk(DAQmxCreateCOPulseChanTicks(CO2taskHandle, "Dev1/ctr1", "CO2", "/Dev1/ao/SampleClock", DAQmx_Val_Low, xPixels + (delay / 2), xPixels + delay, P - delay));
		//Delay starting always at the beginning of the interval
		DAQmxErrChk(DAQmxCreateCOPulseChanTicks(CO2taskHandle, "Dev1/ctr1", "CO2", "/Dev1/ao/SampleClock", DAQmx_Val_Low, xPixels + manShift, xPixels + delay, P - delay));
		//Inverted gating (for use with NAND gate)
		//DAQmxErrChk(DAQmxCreateCOPulseChanTicks(CO2taskHandle, "Dev1/ctr1", "CO2", "/Dev1/ao/SampleClock", DAQmx_Val_Low, P + xPixels - delay, P - delay, xPixels + delay));
		DAQmxErrChk(DAQmxCfgImplicitTiming(CO2taskHandle, DAQmx_Val_ContSamps, numPixels));

		// AI
		DAQmxErrChk(DAQmxCreateTask("AI", &AItaskHandle));
		DAQmxErrChk(DAQmxCreateAIVoltageChan(AItaskHandle, AIchannels, "AI", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(AItaskHandle, "", (float)AIrate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, numScans));
		DAQmxErrChk(DAQmxSetAIDataXferMech(AItaskHandle, AIchannels, DAQmx_Val_DMA));
		DAQmxErrChk(DAQmxCfgDigEdgeStartTrig(AItaskHandle, "ao/StartTrigger", DAQmx_Val_Rising));

		// AO
		DAQmxErrChk(DAQmxCreateTask("AO", &AOtaskHandle));
		DAQmxErrChk(DAQmxCreateAOVoltageChan(AOtaskHandle, "Dev1/ao0:1", "AO", -10.0, 10.0, DAQmx_Val_Volts, NULL));
		//DAQmxErrChk(DAQmxSetAODataXferMech(AOtaskHandle, "Dev1/ao0:1", DAQmx_Val_DMA));
		DAQmxErrChk(DAQmxCfgSampClkTiming(AOtaskHandle, "", (float)AOrate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, numPixels));


		// Make X Waveform
		for (j = 0; j < numPixels; j += (xPixels + P))
		{
			//Phase 1: X
			for (i = 0; i < xPixels; i++)
			{
				waveform[i + j] = (S*i - 1.0);
			}

			//Phase 2: X
			for (i = 0; i < P; i++)
			{
				waveform[i + xPixels + j] = (Sf*i + 1.0);
			}

		}
		// Make Y Waveform
		// Phase 1: Y						// Y mirror follows a sawtooth
		for (i = 0; i < numPixels; i++)
		{
			waveform[i + numPixels] = (Sy*i - 1.0);
		}

		// Set ZoomX,Y
		for (i = 0; i < numPixels; i++)
		{
			waveform[i] *= xZoom;
			waveform[i + numPixels] *= yZoom;

		}

		// Save Waveform (for debugging purposes)
		//FILE *debug = NULL;
		//debug = fopen("E:\\Documents\\Drago_VIs\\gating.txt", "w");
		//for(i = 0; i < numPixels; i ++)
		//{
		//	fprintf(debug, "%f %f %u\n", waveform[i], waveform[i + numPixels], DO_gate[i]);
		//	//fprintf(debug, "%u\n", DO_gate[i]);
		//}
		//fclose(debug);

		// Write Scan Waveform
		//DAQmxErrChk(DAQmxWriteDigitalLines(DOtaskHandle, numPixels, 1, 10.0, DAQmx_Val_GroupByChannel, DO_gate, &written2, NULL));
		DAQmxErrChk(DAQmxWriteAnalogF64(AOtaskHandle, numPixels, 0, 10.0, DAQmx_Val_GroupByChannel, waveform, &written, NULL));


	Error:
		if (DAQmxFailed(error))
		{
			DAQmxGetExtendedErrorInfo(errBuff, 2048);
			printf("DAQmx Error: %s\n", errBuff);
		}

		free(waveform);
		//free(DO_gate);
		return P;
	}
	/* Forward Scan with gating and  imp waveform 1 chan*/
	_declspec (dllexport) int32 forscan_gate1chan(int32 xPixels, int32 yPixels, double xZoom, double yZoom,
		int32 AIrate, int32 AOrate, int32 delay, double Tprop, int32 manShift)
	{
		// DAQ Handles
		TaskHandle AItaskHandle = 0;
		TaskHandle AOtaskHandle = 0;
		//TaskHandle DOtaskHandle = 0;
		TaskHandle COtaskHandle = 0;
		TaskHandle CO2taskHandle = 0;

		// Local Variables 
		int32			error = 0;
		char			AIchannels[100] = "Dev1/ai0\0";				// Always two channels
		double			*waveform = NULL;
		char			errBuff[2048] = { '\0' };

		// Scan Parameters
		int32		numPixels = 0, numScans = 0;
		int32		written = 0;
		int32		read = 0;
		int32		bin = 1;

		// Waveform Variables
		int		i = 0, j = 0;
		double	S, Sy, Sf;
		int		P;


		// Set SCAN parameters
		bin = (int)AIrate / AOrate;
		P = (int)ceil(Tprop*xPixels);
		numPixels = (xPixels + P)*yPixels;
		numScans = bin*numPixels;
		S = 2.0 / xPixels;	// Slope
		Sy = 2.0 / numPixels;	// Yslope
		Sf = -2.0 / P;
		// allocate memory
		waveform = (double *)malloc(sizeof(double)*numPixels * 2);

		//CO, gate for the actual PMT, normally OFF
		DAQmxErrChk(DAQmxCreateTask("CO", &COtaskHandle));
		//DAQmxErrChk(DAQmxCreateCOPulseChanTicks(COtaskHandle, "Dev1/ctr0", "CO", "/Dev1/ao/SampleClock", DAQmx_Val_Low, 0, P, xPixels));
		DAQmxErrChk(DAQmxCreateCOPulseChanTicks(COtaskHandle, "Dev1/ctr0", "CO", "/Dev1/ao/SampleClock", DAQmx_Val_Low, xPixels + manShift, xPixels, P));
		DAQmxErrChk(DAQmxCfgImplicitTiming(COtaskHandle, DAQmx_Val_ContSamps, numPixels));

		//CO2, gate for the projector
		DAQmxErrChk(DAQmxCreateTask("CO2", &CO2taskHandle));
		//Delay centered always at the middle of the interval
		//DAQmxErrChk(DAQmxCreateCOPulseChanTicks(CO2taskHandle, "Dev1/ctr1", "CO2", "/Dev1/ao/SampleClock", DAQmx_Val_Low, xPixels + (delay / 2), xPixels + delay, P - delay));
		//Delay starting always at the beginning of the interval
		DAQmxErrChk(DAQmxCreateCOPulseChanTicks(CO2taskHandle, "Dev1/ctr1", "CO2", "/Dev1/ao/SampleClock", DAQmx_Val_Low, xPixels + manShift, xPixels + delay, P - delay));
		//Inverted gating (for use with NAND gate)
		//DAQmxErrChk(DAQmxCreateCOPulseChanTicks(CO2taskHandle, "Dev1/ctr1", "CO2", "/Dev1/ao/SampleClock", DAQmx_Val_Low, P + xPixels - delay, P - delay, xPixels + delay));
		DAQmxErrChk(DAQmxCfgImplicitTiming(CO2taskHandle, DAQmx_Val_ContSamps, numPixels));

		// AI
		DAQmxErrChk(DAQmxCreateTask("AI", &AItaskHandle));
		DAQmxErrChk(DAQmxCreateAIVoltageChan(AItaskHandle, AIchannels, "AI", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(AItaskHandle, "", (float)AIrate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, numScans));
		DAQmxErrChk(DAQmxSetAIDataXferMech(AItaskHandle, AIchannels, DAQmx_Val_DMA));
		DAQmxErrChk(DAQmxCfgDigEdgeStartTrig(AItaskHandle, "ao/StartTrigger", DAQmx_Val_Rising));

		// AO
		DAQmxErrChk(DAQmxCreateTask("AO", &AOtaskHandle));
		DAQmxErrChk(DAQmxCreateAOVoltageChan(AOtaskHandle, "Dev1/ao0:1", "AO", -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxSetAODataXferMech(AOtaskHandle, "Dev1/ao0:1", DAQmx_Val_DMA));
		DAQmxErrChk(DAQmxCfgSampClkTiming(AOtaskHandle, "", (float)AOrate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, numPixels));


		// Make X Waveform
		for (j = 0; j < numPixels; j += (xPixels + P))
		{
			//Phase 1: X
			for (i = 0; i < xPixels; i++)
			{
				waveform[i + j] = (S*i - 1.0);
			}

			//Phase 2: X
			for (i = 0; i < P; i++)
			{
				waveform[i + xPixels + j] = (Sf*i + 1.0);
			}

		}
		// Make Y Waveform
		// Phase 1: Y						// Y mirror follows a sawtooth
		for (i = 0; i < numPixels; i++)
		{
			waveform[i + numPixels] = (Sy*i - 1.0);
		}

		// Set ZoomX,Y
		for (i = 0; i < numPixels; i++)
		{
			waveform[i] *= xZoom;
			waveform[i + numPixels] *= yZoom;

		}

		// Save Waveform (for debugging purposes)
		//FILE *debug = NULL;
		//debug = fopen("E:\\Documents\\Drago_VIs\\gating.txt", "w");
		//for(i = 0; i < numPixels; i ++)
		//{
		//	fprintf(debug, "%f %f %u\n", waveform[i], waveform[i + numPixels], DO_gate[i]);
		//	//fprintf(debug, "%u\n", DO_gate[i]);
		//}
		//fclose(debug);

		// Write Scan Waveform
		//DAQmxErrChk(DAQmxWriteDigitalLines(DOtaskHandle, numPixels, 1, 10.0, DAQmx_Val_GroupByChannel, DO_gate, &written2, NULL));
		DAQmxErrChk(DAQmxWriteAnalogF64(AOtaskHandle, numPixels, 0, 10.0, DAQmx_Val_GroupByChannel, waveform, &written, NULL));


	Error:
		if (DAQmxFailed(error))
		{
			DAQmxGetExtendedErrorInfo(errBuff, 2048);
			printf("DAQmx Error: %s\n", errBuff);
		}

		free(waveform);
		//free(DO_gate);
		return P;
	}
	/* Forward Scan 1 channel test*/
	_declspec (dllexport) int32 forscan_chan(int32 xPixels, int32 yPixels, double xZoom,
		double yZoom, int32 LinespSec, double *waveform, int32 tar_val, double Tprop)
	{
		// DAQ Handles
		TaskHandle AItaskHandle = 0;
		TaskHandle AOtaskHandle = 0;

		// Local Variables 
		int32			error = 0;
		char			AIchannels[100] = "Dev1/ai2";				// Always two channels
		char			errBuff[2048] = { '\0' };

		// Scan Parameters
		int32		numPixels = 0;
		int32		written = 0;
		int32		read = 0;
		double		AIrate;
		double		AOrate;
		int32		numScans = 0;

		// Waveform Variables
		int		i = 0, j = 0;
		double	S, Sy, Sf;
		int		P;

		if (tar_val == 10) {
		// Set SCAN parameters
		//Tprop = 1;
		P = (int)ceil(Tprop*xPixels);
		numPixels = (xPixels + P)*yPixels;
		numScans = numPixels;
		S = 2.0 / xPixels;	// Slope
		Sy = 2.0 / numPixels;	// Yslope
		Sf = -2.0 / P;

			// Make X Waveform
			for (j = 0; j < numPixels; j += (xPixels + P))
			{
				//Phase 1: X
				for (i = 0; i < xPixels; i++)
				{
					waveform[i + j] = (S*i - 1.0);
				}

				//Phase 2: X
				for (i = 0; i < P; i++)
				{
					waveform[i + xPixels + j] = (Sf*i + 1.0);
				}

			}
		}
		else { //if it's in the calibration phase
			numPixels = xPixels *yPixels;
			numScans = numPixels;
			for (i = 0; i < numPixels; i++)
			{
				waveform[i] = tar_val;
			}
		}

		// Set ZoomX
		for (i = 0; i < numPixels; i++)
		{
			waveform[i] *= xZoom;
		}

		// Save Waveform (for debugging purposes)
		//FILE *debug = NULL;
		//debug = fopen("E:\\Documents\\Drago_VIs\\gating.txt", "w");
		//for(i = 0; i < numPixels; i ++)
		//{
		//	fprintf(debug, "%f %f %u\n", waveform[i], waveform[i + numPixels], DO_gate[i]);
		//	//fprintf(debug, "%u\n", DO_gate[i]);
		//}
		//fclose(debug);

		// Calculate frame rate and AO/AI rates
		AOrate = (double)LinespSec;
		AIrate = AOrate;

		// AI
		DAQmxErrChk(DAQmxCreateTask("AI", &AItaskHandle));
		DAQmxErrChk(DAQmxCreateAIVoltageChan(AItaskHandle, AIchannels, "AI", DAQmx_Val_Cfg_Default, -5.0, 5.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(AItaskHandle, NULL, (float)AIrate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, numScans));
		DAQmxErrChk(DAQmxSetAIDataXferMech(AItaskHandle, AIchannels, DAQmx_Val_DMA));
		DAQmxErrChk(DAQmxCfgDigEdgeStartTrig(AItaskHandle, "ao/StartTrigger", DAQmx_Val_Rising));

		// AO
		DAQmxErrChk(DAQmxCreateTask("AO", &AOtaskHandle));
		DAQmxErrChk(DAQmxCreateAOVoltageChan(AOtaskHandle, "Dev1/ao0", "AO", -5.0, 5.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(AOtaskHandle, NULL, (float)AOrate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, numPixels));

		// Write Scan Waveform
		DAQmxErrChk(DAQmxWriteAnalogF64(AOtaskHandle, numPixels, 0, 10.0, DAQmx_Val_GroupByChannel, waveform, &written, NULL));


	Error:
		if (DAQmxFailed(error))
		{
			DAQmxGetExtendedErrorInfo(errBuff, 2048);
			printf("DAQmx Error: %s\n", errBuff);
		}


		return numScans;
	}
	/* Forward Scan 2 channel test*/
	_declspec (dllexport) int32 forscan_2chan(int32 xPixels, int32 yPixels, double xZoom,
		double yZoom, int32 LinespSec, double *waveform, int32 tar_val, double Tprop)
	{
		// DAQ Handles
		TaskHandle AItaskHandle = 0;
		TaskHandle AOtaskHandle = 0;

		// Local Variables 
		int32			error = 0;
		char			AIchannels[100] = "Dev1/ai2:3";				// Always two channels
		char			errBuff[2048] = { '\0' };

		// Scan Parameters
		int32		numPixels = 0;
		int32		written = 0;
		int32		read = 0;
		double		AIrate;
		double		AOrate;
		int32		numScans = 0;

		// Waveform Variables
		int		i = 0, j = 0;
		double	S, Sy, Sf;
		int		P;

		if (tar_val == 10) {
			// Set SCAN parameters
			P = (int)ceil(Tprop*xPixels);
			numPixels = (xPixels + P)*yPixels;
			numScans = numPixels;
			S = 2.0 / xPixels;	// Slope
			Sf = -2.0 / P;

			// Make X Waveform
			for (j = 0; j < numPixels; j += (xPixels + P))
			{
				//Phase 1: X
				for (i = 0; i < xPixels; i++)
				{
					waveform[i + j] = (S*i - 1.0);
				}

				//Phase 2: X
				for (i = 0; i < P; i++)
				{
					waveform[i + xPixels + j] = (Sf*i + 1.0);
				}

			}
			// Make Y Waveform
			// Phase 1: Y						// Y mirror follows a sawtooth
			Sy = 2.0 / numPixels;	// Yslope

			for (i = 0; i < numPixels; i++)
			{
				waveform[i + numPixels] = (Sy*i - 1.0);
			}
		}
		else { //if it's in the calibration phase
			numPixels = xPixels*yPixels;
			numScans = numPixels;
			for (i = 0; i < numPixels; i++)
			{
				waveform[i] = tar_val;
				waveform[i + numPixels] = tar_val;

			}
		}

		

		// Set ZoomX,Y
		for (i = 0; i < numPixels; i++)
		{
			waveform[i] *= xZoom;
			waveform[i + numPixels] *= yZoom;

		}

		// Save Waveform (for debugging purposes)
		//FILE *debug = NULL;
		//debug = fopen("E:\\Documents\\Drago_VIs\\gating.txt", "w");
		//for(i = 0; i < numPixels; i ++)
		//{
		//	fprintf(debug, "%f %f %u\n", waveform[i], waveform[i + numPixels], DO_gate[i]);
		//	//fprintf(debug, "%u\n", DO_gate[i]);
		//}
		//fclose(debug);

		// Define frame rate and AO/AI rates
		AOrate = (double)LinespSec;
		AIrate = AOrate;

		// AI
		DAQmxErrChk(DAQmxCreateTask("AI", &AItaskHandle));
		DAQmxErrChk(DAQmxCreateAIVoltageChan(AItaskHandle, AIchannels, "AI", DAQmx_Val_Cfg_Default, -5.0, 5.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(AItaskHandle, NULL, (float)AIrate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, numScans));
		DAQmxErrChk(DAQmxSetAIDataXferMech(AItaskHandle, AIchannels, DAQmx_Val_DMA));
		DAQmxErrChk(DAQmxCfgDigEdgeStartTrig(AItaskHandle, "ao/StartTrigger", DAQmx_Val_Rising));

		// AO
		DAQmxErrChk(DAQmxCreateTask("AO", &AOtaskHandle));
		DAQmxErrChk(DAQmxCreateAOVoltageChan(AOtaskHandle, "Dev1/ao0:1", "AO", -5.0, 5.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(AOtaskHandle, NULL, (float)AOrate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, numPixels));

		// Write Scan Waveform
		DAQmxErrChk(DAQmxWriteAnalogF64(AOtaskHandle, numPixels, 0, 10.0, DAQmx_Val_GroupByChannel, waveform, &written, NULL));


	Error:
		if (DAQmxFailed(error))
		{
			DAQmxGetExtendedErrorInfo(errBuff, 2048);
			printf("DAQmx Error: %s\n", errBuff);
		}

		return numScans;
	}
	/* Back Scan */
	_declspec (dllexport) int32 backscan(int32 xPixels, int32 yPixels, double xZoom, double yZoom, int32 AIrate, int32 AOrate)
	{
		// DAQ Handles
		TaskHandle AItaskHandle = 0;
		TaskHandle AOtaskHandle = 0;

		// Local Variables 
		int32			error = 0;
		char			AIchannels[100] = "Dev1/ai0:3\0";				// Always four channels
		double			*waveform = NULL;
		char			errBuff[2048] = { '\0' };

		// Scan Parameters
		int32		numPixels = 0, numScans = 0;
		int32		written = 0;
		int32		read = 0;
		int32		bin = 1;

		// Waveform Variables
		int		i = 0, j = 0;
		double	S, Sy, Tscale;
		int		T;

		// Set SCAN parameters
		bin = (int)(AIrate / AOrate);
		T = int(xPixels*0.10);									// Turn Around Pixels (10% of scan pixels)
		Tscale = (20.0*(T*T) / 4.0);
		numPixels = (xPixels + T)*yPixels;
		numScans = bin*numPixels;
		S = 2.0 / xPixels;		// XSlope
		Sy = 2.0 / numPixels;	// Yslope

		// allocate memory
		waveform = (double *)malloc(sizeof(double)*numPixels * 2);

		// AI
		DAQmxErrChk(DAQmxCreateTask("AI", &AItaskHandle));
		DAQmxErrChk(DAQmxCreateAIVoltageChan(AItaskHandle, AIchannels, "AI", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(AItaskHandle, "", (float)AIrate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, numScans));
		DAQmxErrChk(DAQmxSetAIDataXferMech(AItaskHandle, AIchannels, DAQmx_Val_DMA));
		DAQmxErrChk(DAQmxCfgDigEdgeStartTrig(AItaskHandle, "ao/StartTrigger", DAQmx_Val_Rising));

		// AO
		DAQmxErrChk(DAQmxCreateTask("AO", &AOtaskHandle));
		DAQmxErrChk(DAQmxCreateAOVoltageChan(AOtaskHandle, "Dev1/ao0:1", "AO", -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(AOtaskHandle, "", (float)AOrate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, numPixels));

		// Make X Waveform (linear ramp in both directions)
		for (j = 0; j < numPixels; j += (2 * (xPixels + T)))
		{
			//Phase 1: X
			for (i = 0; i < xPixels; i++)
			{
				waveform[i + j] = (S*i - 1.0);
			}

			//Phase 2: X turn around 1
			for (i = 0; i < T; i++)
			{
				waveform[i + xPixels + j] = ((T*T) / (4.0*Tscale)) + 1.0 + (-1.0 * ((i - T / 2.0)*(i - T / 2.0)) / Tscale);
			}

			// Phase 3: X back scan
			for (i = 0; i < xPixels; i++)
			{
				waveform[i + xPixels + T + j] = (1.0 - S*i);
			}

			//Phase 4: X turn around 2
			for (i = 0; i < T; i++)
			{
				waveform[i + xPixels + T + xPixels + j] = -1.0*(((T*T) / (4.0*Tscale)) + 1.0 + (-1.0 * ((i - T / 2.0)*(i - T / 2.0)) / Tscale));
			}
		}

		// Make Y Waveform
		// Phase 1: Y						// Y mirror follows a sawtooth
		for (i = 0; i < numPixels; i++)
		{
			waveform[i + numPixels] = (Sy*i - 1.0);
		}

		// Set ZoomX,Y
		for (i = 0; i < numPixels; i++)
		{
			waveform[i] *= xZoom;
			waveform[i + numPixels] *= yZoom;

		}

		// Save Waveform (for debugging purposes)
		//	FILE *debug = NULL;
		//	debug = fopen("C:\\waveform.txt", "w");
		//	for(i = 0; i < numPixels; i ++)
		//	{
		//		fprintf(debug, "%f %f\n", waveform[i], waveform[i+numPixels]);
		//	}
		//	fclose(debug);

		// Write Scan Waveform
		DAQmxErrChk(DAQmxWriteAnalogF64(AOtaskHandle, numPixels, 0, 10.0, DAQmx_Val_GroupByChannel, waveform, &written, NULL));


	Error:
		if (DAQmxFailed(error))
		{
			DAQmxGetExtendedErrorInfo(errBuff, 2048);
			printf("DAQmx Error: %s\n", errBuff);
		}
		free(waveform);

		return T;
	}

	/* Back Scan RAW - no encoder readings */
	_declspec (dllexport) int32 backscan_raw(int32 xPixels, int32 yPixels, double xZoom, double yZoom, int32 AIrate, int32 AOrate)
	{
		// DAQ Handles
		TaskHandle AItaskHandle = 0;
		TaskHandle AOtaskHandle = 0;

		// Local Variables 
		int32			error = 0;
		char			AIchannels[100] = "Dev1/ai0:1\0";				// Always four channels
		double			*waveform = NULL;
		char			errBuff[2048] = { '\0' };

		// Scan Parameters
		int32		numPixels = 0, numScans = 0;
		int32		written = 0;
		int32		read = 0;
		int32		bin = 1;

		// Waveform Variables
		int		i = 0, j = 0;
		double	S, Sy, Tscale;
		int		T;

		// Set SCAN parameters
		bin = (int)(AIrate / AOrate);
		T = int(xPixels*0.10);									// Turn Around Pixels (10% of scan pixels)
		Tscale = (20.0*(T*T) / 4.0);
		numPixels = (xPixels + T)*yPixels;
		numScans = bin*numPixels;
		S = 2.0 / xPixels;		// XSlope
		Sy = 2.0 / numPixels;	// Yslope

		// allocate memory
		waveform = (double *)malloc(sizeof(double)*numPixels * 2);

		// AI
		DAQmxErrChk(DAQmxCreateTask("AI", &AItaskHandle));
		DAQmxErrChk(DAQmxCreateAIVoltageChan(AItaskHandle, AIchannels, "AI", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(AItaskHandle, "", (float)AIrate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, numScans));
		DAQmxErrChk(DAQmxSetAIDataXferMech(AItaskHandle, AIchannels, DAQmx_Val_DMA));
		DAQmxErrChk(DAQmxCfgDigEdgeStartTrig(AItaskHandle, "ao/StartTrigger", DAQmx_Val_Rising));

		// AO
		DAQmxErrChk(DAQmxCreateTask("AO", &AOtaskHandle));
		DAQmxErrChk(DAQmxCreateAOVoltageChan(AOtaskHandle, "Dev1/ao0:1", "AO", -10.0, 10.0, DAQmx_Val_Volts, NULL));
		DAQmxErrChk(DAQmxCfgSampClkTiming(AOtaskHandle, "", (float)AOrate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, numPixels));

		// Make X Waveform (linear ramp in both directions)
		for (j = 0; j < numPixels; j += (2 * (xPixels + T)))
		{
			//Phase 1: X
			for (i = 0; i < xPixels; i++)
			{
				waveform[i + j] = (S*i - 1.0);
			}

			//Phase 2: X turn around 1
			for (i = 0; i < T; i++)
			{
				waveform[i + xPixels + j] = ((T*T) / (4.0*Tscale)) + 1.0 + (-1.0 * ((i - T / 2.0)*(i - T / 2.0)) / Tscale);
			}

			// Phase 3: X back scan
			for (i = 0; i < xPixels; i++)
			{
				waveform[i + xPixels + T + j] = (1.0 - S*i);
			}

			//Phase 4: X turn around 2
			for (i = 0; i < T; i++)
			{
				waveform[i + xPixels + T + xPixels + j] = -1.0*(((T*T) / (4.0*Tscale)) + 1.0 + (-1.0 * ((i - T / 2.0)*(i - T / 2.0)) / Tscale));
			}
		}

		// Make Y Waveform
		// Phase 1: Y						// Y mirror follows a sawtooth
		for (i = 0; i < numPixels; i++)
		{
			waveform[i + numPixels] = (Sy*i - 1.0);
		}

		// Set ZoomX,Y
		for (i = 0; i < numPixels; i++)
		{
			waveform[i] *= xZoom;
			waveform[i + numPixels] *= yZoom;

		}

		// Save Waveform (for debugging purposes)
		//	FILE *debug = NULL;
		//	debug = fopen("C:\\waveform.txt", "w");
		//	for(i = 0; i < numPixels; i ++)
		//	{
		//		fprintf(debug, "%f %f\n", waveform[i], waveform[i+numPixels]);
		//	}
		//	fclose(debug);

		// Write Scan Waveform
		DAQmxErrChk(DAQmxWriteAnalogF64(AOtaskHandle, numPixels, 0, 10.0, DAQmx_Val_GroupByChannel, waveform, &written, NULL));


	Error:
		if (DAQmxFailed(error))
		{
			DAQmxGetExtendedErrorInfo(errBuff, 2048);
			printf("DAQmx Error: %s\n", errBuff);
		}
		free(waveform);

		return T;
	}

	/* Distortion Correct and BIN Back Scan */
	_declspec (dllexport) void undistort(int xPixels, int yPixels, int T, int bin, int manualShift,
		int16 *data0, int16 *data1,
		int16 *xMirror, int16 *yMirror,
		int16 *frame0, int16 *frame1)
	{
		int x = 0, y = 0, c = 0, f = 0, b = 0, offset = 0, start = 0;
		int xRes = 0;
		int yRes = 0;
		float	*xAVGforward = NULL, *xAVGbackward = NULL;
		float	currentThreshold = 0.0f;
		int		numSteps = 10;

		int		ForMaxInd = 0, ForMinInd = 0;
		int		BackMaxInd = 0, BackMinInd = 0;
		float	ForMaxVal = 0.0f, ForMinVal = 0.0f;
		float	BackMaxVal = 0.0f, BackMinVal = 0.0f;
		float	MaxVal = 0.0f, MinVal = 0.0f;
		float	BinRes = 0.0f;
		int		*ForBins = NULL;
		int		*BackBins = NULL;

		// Scale Element Number by Bin Factor
		xRes = xPixels;
		yRes = yPixels;
		xPixels = xPixels*bin;
		T = T*bin;

		// Allocate Memory
		xAVGforward = (float *)malloc(sizeof(float)*(T + T + T + xPixels));
		xAVGbackward = (float *)malloc(sizeof(float)*(T + T + T + xPixels));
		ForBins = (int *)malloc(sizeof(int)*(xRes));
		BackBins = (int *)malloc(sizeof(int)*(xRes));

		// Initialize Memory to zeros
		for (c = 0; c < xPixels + T + T + T; c++)
		{
			xAVGforward[c] = 0.0f;
			xAVGbackward[c] = 0.0f;
		}
		for (c = 0; c < xRes; c++)
		{
			ForBins[c] = 0;
			BackBins[c] = 0;
		}

		// Average Xmirror Data (ignoring first and last line) - turn-line-turn-(turn distance)
		f = 0;
		b = 0;
		for (y = 1; y < yPixels - 1; y++)
		{
			if (y % 2 == 0) // Forward Scan
			{
				c = 0;
				for (x = -T; x < xPixels + T + T; x++)
				{
					xAVGforward[c] = xAVGforward[c] + (float)xMirror[x + (y*(xPixels + T))];
					c++;
				}
				f++;
			}
			else { // backward scan
				c = 0;
				for (x = -T; x < xPixels + T + T; x++)
				{
					xAVGbackward[c] = xAVGbackward[c] + (float)xMirror[x + (y*(xPixels + T))];
					c++;
				}
				b++;
			}
		}

		// Initialize Min/Max Values
		ForMinVal = xAVGforward[0] / f;
		BackMinVal = xAVGbackward[0] / b;
		ForMaxVal = xAVGforward[0] / f;
		BackMaxVal = xAVGbackward[0] / b;

		// Average Waveform and find forward min/max
		for (c = 0; c < xPixels + T + T + T; c++)
		{
			xAVGforward[c] = xAVGforward[c] / f;
			if (xAVGforward[c] > ForMaxVal)
			{
				ForMaxVal = xAVGforward[c];
				ForMaxInd = c;
			}
			if (xAVGforward[c] < ForMinVal)
			{
				ForMinVal = xAVGforward[c];
				ForMinInd = c;
			}

			xAVGbackward[c] = xAVGbackward[c] / b;
			if (xAVGbackward[c] > BackMaxVal)
			{
				BackMaxVal = xAVGbackward[c];
				BackMaxInd = c;
			}
			if (xAVGbackward[c] < BackMinVal)
			{
				BackMinVal = xAVGbackward[c];
				BackMinInd = c;
			}
		}

		// Constrain Min/Max
		MaxVal = min(ForMaxVal, BackMaxVal);
		MinVal = max(ForMinVal, BackMinVal);

		// Compute Ranges and Bin Resolutions
		BinRes = (MaxVal - MinVal) / (float)xRes;

		// Construct the binsize array (forward and backward)
		currentThreshold = MinVal + BinRes;
		offset = 0;
		for (c = 0; c < xRes; c++)
		{
			//MIKE ADDED
			if (!((xAVGforward[ForMinInd + offset] < currentThreshold) && ((ForMinInd + offset) < ForMaxInd)))
			{
				ForBins[c] = ForMinInd + offset;
			}

			while ((xAVGforward[ForMinInd + offset] < currentThreshold) && ((ForMinInd + offset) < ForMaxInd))
			{
				offset++;
				ForBins[c] = ForMinInd + offset;
			}
			currentThreshold += BinRes;
		}
		currentThreshold = MinVal + BinRes;
		offset = 0;
		for (c = 0; c < xRes; c++)
		{
			//MIKE ADDED
			if (!((xAVGbackward[BackMinInd - offset] < currentThreshold) && ((BackMinInd - offset) > BackMaxInd)))
			{
				BackBins[c] = BackMinInd - offset;
			}

			while ((xAVGbackward[BackMinInd - offset] < currentThreshold) && ((BackMinInd - offset) > BackMaxInd))
			{
				offset++;
				BackBins[c] = BackMinInd - offset;
			}
			currentThreshold += BinRes;
		}


		// Bin Undistorted Frames (first and last line = crap)
		for (y = 1; y < yPixels - 1; y++)
		{
			if (y % 2 == 0) // Forward Scan
			{
				start = ForMinInd;
				for (x = 0; x < xRes; x++)
				{
					frame0[x + (y*xRes)] = 0;
					frame1[x + (y*xRes)] = 0;
					f = 0;
					//MIKE ADDED
					if (start >= ForBins[x]) { start = ForBins[x] - 1; }
					for (c = start; c < ForBins[x]; c++)
					{
						frame0[x + (y*xRes)] += data0[c - T + (y*(xPixels + T))];
						frame1[x + (y*xRes)] += data1[c - T + (y*(xPixels + T))];
						f++;
					}
					frame0[x + (y*xRes)] = 100 + (frame0[x + (y*xRes)] / f);
					frame1[x + (y*xRes)] = 100 + (frame1[x + (y*xRes)] / f);
					start = ForBins[x];
				}
			}
			else { // backward scan
				start = BackMinInd + manualShift;
				for (x = 0; x < xRes; x++)
				{
					frame0[x + (y*xRes)] = 0;
					frame1[x + (y*xRes)] = 0;
					b = 0;
					//MIKE ADDED
					if (start <= BackBins[x]) { start = BackBins[x] + 1; }
					for (c = start; c > BackBins[x] + manualShift; c--)
					{
						frame0[x + (y*xRes)] += data0[c - T + (y*(xPixels + T))];
						frame1[x + (y*xRes)] += data1[c - T + (y*(xPixels + T))];
						b++;
					}
					frame0[x + (y*xRes)] = 100 + (frame0[x + (y*xRes)] / b);
					frame1[x + (y*xRes)] = 100 + (frame1[x + (y*xRes)] / b);
					start = BackBins[x] + manualShift;
				}
			}
		}
		// Duplicate First and Last lines (from neighbors)
		for (x = 0; x < xRes; x++)
		{
			frame0[x] = frame0[x + xRes];
			frame1[x] = frame1[x + xRes];
		}
		for (x = 0; x < xRes; x++)
		{
			frame0[x + ((yRes - 1)*xRes)] = frame0[x + ((yRes - 2)*xRes)];
			frame1[x + ((yRes - 1)*xRes)] = frame1[x + ((yRes - 2)*xRes)];
		}

		// Cleanup
		free(xAVGforward);
		free(xAVGbackward);
		free(ForBins);
		free(BackBins);

		return;
	}


	/* Binning for RAW Back Scan */
	_declspec (dllexport) void backscan_bin(int xPixels, int yPixels, int T, int bin, int manualShift,
		int16 *data0, int16 *data1,
		int16 *frame0, int16 *frame1)
	{
		int x = 0, y = 0, c = 0, f = 0, b = 0, offset = 0, start = 0;
		int xRes = 0;
		int yRes = 0;

		// Scale Element Number by Bin Factor
		xRes = xPixels;
		yRes = yPixels;
		xPixels = xPixels*bin;
		T = T*bin;

		// Bin Undistorted Frames (first and last line = crap)
		for (y = 1; y < yPixels - 1; y++)
		{
			if (y % 2 == 0) // Forward Scan
			{
				for (x = 0; x < xRes; x++)
				{
					frame0[x + (y*xRes)] = 0;
					frame1[x + (y*xRes)] = 0;
					f = 0;
					for (c = 0; c < bin; c++)
					{
						frame0[x + (y*xRes)] += data0[c + (x*bin) + (y*(xPixels + T))];
						frame1[x + (y*xRes)] += data1[c + (x*bin) + (y*(xPixels + T))];
						f++;
					}
					frame0[x + (y*xRes)] = 100 + (frame0[x + (y*xRes)] / f);
					frame1[x + (y*xRes)] = 100 + (frame1[x + (y*xRes)] / f);
				}
			}
			else { // backward scan
				for (x = 0; x < xRes; x++)
				{
					frame0[x + (y*xRes)] = 0;
					frame1[x + (y*xRes)] = 0;
					b = 0;
					for (c = 0; c < bin; c++)
					{
						frame0[x + (y*xRes)] += data0[xPixels - (c + manualShift + (x*bin)) + (y*(xPixels + T))];
						frame1[x + (y*xRes)] += data1[xPixels - (c + manualShift + (x*bin)) + (y*(xPixels + T))];
						b++;
					}
					frame0[x + (y*xRes)] = 100 + (frame0[x + (y*xRes)] / b);
					frame1[x + (y*xRes)] = 100 + (frame1[x + (y*xRes)] / b);
				}
			}
		}
		// Duplicate First and Last lines (from neighbors)
		for (x = 0; x < xRes; x++)
		{
			frame0[x] = frame0[x + xRes];
			frame1[x] = frame1[x + xRes];
		}
		for (x = 0; x < xRes; x++)
		{
			frame0[x + ((yRes - 1)*xRes)] = frame0[x + ((yRes - 2)*xRes)];
			frame1[x + ((yRes - 1)*xRes)] = frame1[x + ((yRes - 2)*xRes)];
		}

		return;
	}

	/* Binning for Forward Scan Only */
	_declspec (dllexport) void forscan_bin(int xPixels, int yPixels, int P, int bin,
		int16 *data0, int16 *data1,
		int16 *frame0, int16 *frame1)
	{
		int x = 0, y = 0, c = 0, f = 0, b = 0, offset = 0, start = 0;
		int xRes = 0;
		int yRes = 0;
		int P_prebin = 0;

		// Scale Element Number by Bin Factor
		xRes = xPixels;
		yRes = yPixels;
		xPixels = xPixels*bin;
		P_prebin = P;
		P = P*bin;

		// Bin Undistorted Frames (first and last line = crap)
		for (y = 1; y < yPixels - 1; y++)
		{
			
			for (x = 0; x < xRes; x++)
			{
				frame0[x + (y*xRes)] = 0;
				//frame1[x + (y*xRes)] = 0;
				f = 0;
				for (c = 0; c < bin; c++)
				{
					frame0[x + (y*xRes)] += data0[c + (x*bin) + (y*(xPixels + P))];
					//frame1[x + (y*xRes)] += data1[c + (x*bin) + (y*(xPixels + P))];
					f++;
				}
				frame0[x + (y*xRes)] = 100 + (frame0[x + (y*xRes)] / f);
				//frame1[x + (y*xRes)] = 100 + (frame1[x + (y*xRes)] / f);
			}
			
		}
		// Duplicate First and Last lines (from neighbors)
		for (x = 0; x < xRes; x++)
		{
			frame0[x] = frame0[x + xRes];
			//frame1[x] = frame1[x + xRes];
		}
		for (x = 0; x < xRes; x++)
		{
			frame0[x + ((yRes - 1)*xRes)] = frame0[x + ((yRes - 2)*xRes)];
			//frame1[x + ((yRes - 1)*xRes)] = frame1[x + ((yRes - 2)*xRes)];
		}

		return;
	}
	/* Binning for Forward Scan plus shift */
	_declspec (dllexport) void forscan_shift(int xPixels, int yPixels, int P, int bin, int manShift,
		int16 *data0, int16 *data1,
		int16 *frame0, int16 *frame1)
	{
		int x = 0, y = 0, c = 0, f = 0, b = 0, offset = 0, start = 0;
		int xRes = 0;
		int yRes = 0;
		int P_prebin = 0;

		// Scale Element Number by Bin Factor
		xRes = xPixels;
		yRes = yPixels;
		xPixels = xPixels*bin;
		P_prebin = P;
		P = P*bin;

		// Bin Undistorted Frames (first and last line = crap)
		for (y = 1; y < yPixels - 1; y++)
		{

			for (x = 0; x < xRes; x++)
			{
				frame0[x + (y*xRes)] = 0;
				//frame1[x + (y*xRes)] = 0;
				//f = 0;
				for (c = 0; c < bin; c++)
				{
					frame0[x + (y*xRes)] += data0[c + manShift + (x*bin) + (y*(xPixels + P))];
					//frame1[x + (y*xRes)] += data1[c + (x*bin) + (y*(xPixels + P))];
					//f++;
				}
				frame0[x + (y*xRes)] = 100 + (frame0[x + (y*xRes)] / bin);
				//frame0[x + (y*xRes)] = 100 + (frame0[x + (y*xRes)] / f);
				//frame1[x + (y*xRes)] = 100 + (frame1[x + (y*xRes)] / f);
			}

		}
		// Duplicate First and Last lines (from neighbors)
		for (x = 0; x < xRes; x++)
		{
			frame0[x] = frame0[x + xRes];
			//frame1[x] = frame1[x + xRes];
		}
		for (x = 0; x < xRes; x++)
		{
			frame0[x + ((yRes - 1)*xRes)] = frame0[x + ((yRes - 2)*xRes)];
			//frame1[x + ((yRes - 1)*xRes)] = frame1[x + ((yRes - 2)*xRes)];
		}

		return;
	}
	/* Binning for Forward Scan plus shift and 1 chan */
	_declspec (dllexport) void forscan_shift1chan(int xPixels, int yPixels, int P, int bin, int manShift,
		int16 *data0, int16 *frame0)
	{
		int x = 0, y = 0, c = 0, f = 0, b = 0, offset = 0, start = 0;
		int xRes = 0;
		int yRes = 0;
		int P_prebin = 0;

		// Scale Element Number by Bin Factor
		xRes = xPixels;
		yRes = yPixels;
		xPixels = xPixels*bin;
		P_prebin = P;
		P = P*bin;

		// Bin Undistorted Frames (first and last line = crap)
		for (y = 1; y < yPixels - 1; y++)
		{

			for (x = 0; x < xRes; x++)
			{
				frame0[x + (y*xRes)] = 0;

				for (c = 0; c < bin; c++)
				{
					frame0[x + (y*xRes)] += data0[c + manShift + (x*bin) + (y*(xPixels + P))];
					
				}
				frame0[x + (y*xRes)] = 100 + (frame0[x + (y*xRes)] / bin);
				
			}

		}
		// Duplicate First and Last lines (from neighbors)
		for (x = 0; x < xRes; x++)
		{
			frame0[x] = frame0[x + xRes];
		}
		for (x = 0; x < xRes; x++)
		{
			frame0[x + ((yRes - 1)*xRes)] = frame0[x + ((yRes - 2)*xRes)];
		}

		return;
	}
	/* Binning for Debugging Forward Scan + Gate */
	_declspec (dllexport) void forscan_debug(int xPixels, int yPixels, int P, int bin,
		int16 *data0, int16 *data1,
		int16 *frame0, int16 *frame1)
	{
		int x = 0, y = 0, c = 0, f = 0, b = 0, offset = 0, start = 0;
		int xRes = 0;
		int yRes = 0;
		int P_prebin = 0;

		// Scale Element Number by Bin Factor
		xRes = xPixels;
		yRes = yPixels;
		xPixels = xPixels*bin;
		P_prebin = P;
		P = P*bin;

		// Bin Undistorted Frames (first and last line = crap)
		for (y = 1; y < yPixels - 1; y++)
		{

			for (x = 0; x < xRes + P_prebin; x++)
			{
				frame0[x + (y*(xRes + P_prebin))] = 0;
				//frame1[x + (y*xRes)] = 0;
				f = 0;
				for (c = 0; c < bin; c++)
				{
					frame0[x + (y*(xRes + P_prebin))] += data0[c + (x*bin) + (y*(xPixels + P))];
					//frame1[x + (y*xRes)] += data1[c + (x*bin) + (y*(xPixels + P))];
					f++;
				}
				frame0[x + (y*(xRes + P_prebin))] = 100 + (frame0[x + (y*(xRes + P_prebin))] / f);
				//frame1[x + (y*xRes)] = 100 + (frame1[x + (y*xRes)] / f);
			}

		}
		// Duplicate First and Last lines (from neighbors)
		for (x = 0; x < (xRes + P_prebin); x++)
		{
			frame0[x] = frame0[x + (xRes + P_prebin)];
			//frame1[x] = frame1[x + xRes];
		}
		for (x = 0; x < (xRes + P_prebin); x++)
		{
			frame0[x + ((yRes - 1)*(xRes + P_prebin))] = frame0[x + ((yRes - 2)*(xRes + P_prebin))];
			//frame1[x + ((yRes - 1)*xRes)] = frame1[x + ((yRes - 2)*xRes)];
		}

		return;
	}