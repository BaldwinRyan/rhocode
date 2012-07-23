#ifndef PMD_Diagnostics
#define PMD_Diagnostics

//  PMDDiagnostics.h -- diagnostic functions
//
//  Performance Motion Devices, Inc.
//

#if defined(__cplusplus)
extern "C" {
#endif
    
    
    struct tagPMDOpcodeText {
        PMDuint16 opcode;
        char*     text;
        char*     pad;
    } PMDOpcodeText[] = {
        {PMDOPNoOperation,              "Nop","                   "},
        {0x01,                          "Invalid opcode!","       "},
        {0x02,                          "Invalid opcode!","       "},
        {0x03,                          "Invalid opcode!","       "},
        {0x04,                          "Invalid opcode!","       "},
        {0x05,                          "Invalid opcode!","       "},
        {PMDOPSetMotorLimit,            "SetMotorLimit","         "},
        {PMDOPGetMotorLimit,            "GetMotorLimit","         "},
        {0x08,                          "Invalid opcode!","       "},
        {0x09,                          "Invalid opcode!","       "},
        {0x0A,                          "Invalid opcode!","       "},
        {0x0B,                          "Invalid opcode!","       "},
        {0x0C,                          "Invalid opcode!","       "},
        {0x0D,                          "Invalid opcode!","       "},
        {0x0E,                          "Invalid opcode!","       "},
        {PMDOPSetMotorBias,             "SetMotorBias","          "},
        
        {PMDOPSetPosition,              "SetPosition","           "},
        {PMDOPSetVelocity,              "SetVelocity","           "},
        {0x12,                          "Invalid opcode!","       "},
        {PMDOPSetJerk,                  "SetJerk","               "},
        {PMDOPSetGearRatio,             "SetGearRatio","          "},
        {0x15,                          "Invalid opcode!","       "},
        {0x16,                          "Invalid opcode!","       "},
        {0x17,                          "Invalid opcode!","       "},
        {0x18,                          "Invalid opcode!","       "},
        {0x19,                          "Invalid opcode!","       "},
        {PMDOPUpdate,                   "Update","                "},
        {0x1B,                          "Invalid opcode!","       "},
        {0x1C,                          "Invalid opcode!","       "},
        {PMDOPGetCommandedPosition,     "GetCommandedPosition","  "},
        {PMDOPGetCommandedVelocity,     "GetCommandedVelocity","  "},
        {0x1F,                          "Invalid opcode!","       "},

        {0x20,                          "Invalid opcode!","       "},
        {0x21,                          "Invalid opcode!","       "},
        {0x22,                          "Invalid opcode!","       "},
        {0x23,                          "Invalid opcode!","       "},
        {0x24,                          "Invalid opcode!","       "},
        {PMDOPSetKp,                    "SetKp","                 "},
        {PMDOPSetKi,                    "SetKi","                 "},
        {PMDOPSetKd,                    "SetKd","                 "},
        {0x28,                          "Invalid opcode!","       "},
        {0x29,                          "Invalid opcode!","       "},
        {0x2A,                          "Invalid opcode!","       "},
        {PMDOPSetKvff,                  "SetKvff","               "},
        {PMDOPGetPhaseAngle,            "GetPhaseAngle","         "},
        {PMDOPGetMotorBias,             "GetMotorBias","          "},
        {0x2E,                          "Invalid opcode!","       "},
        {PMDOPSetInterruptMask,         "SetInterruptMask","      "},
        
        {0x30,                          "Invalid opcode!","       "},
        {PMDOPGetEventStatus,           "GetEventStatus","        "},
        {0x32,                          "Invalid opcode!","       "},
        {0x33,                          "Invalid opcode!","       "},
        {PMDOPResetEventStatus,         "ResetEventStatus","      "},
        {0x35,                          "Invalid opcode!","       "},
        {PMDOPGetCaptureValue,          "GetCaptureValue","       "},
        {PMDOPGetActualPosition,        "GetActualPosition","     "},
        {PMDOPSetSampleTime,            "SetSampleTime","         "},
        {PMDOPReset,                    "Reset","                 "},
        {PMDOPGetCurrentMotorCommand,   "GetCurrentMotorCommand",""},
        {0x3B,                          "Invalid opcode!","       "},
        {0x3C,                          "Invalid opcode!","       "},
        {0x3D,                          "Invalid opcode!","       "},
        {PMDOPGetTime,                  "GetTime","               "},
        {0x3F,                          "Invalid opcode!","       "},
        
        {0x40,                          "Invalid opcode!","       "},
        {0x41,                          "Invalid opcode!","       "},
        {0x42,                          "Invalid opcode!","       "},
        {0x43,                          "Invalid opcode!","       "},
        {0x44,                          "Invalid opcode!","       "},
        {0x45,                          "Invalid opcode!","       "},
        {0x46,                          "Invalid opcode!","       "},
        {PMDOPClearPositionError,       "ClearPositionError","    "},
        {0x48,                          "Invalid opcode!","       "},
        {0x49,                          "Invalid opcode!","       "},
        {PMDOPGetPosition,              "GetPosition","           "},
        {PMDOPGetVelocity,              "GetVelocity","           "},
        {PMDOPGetAcceleration,          "GetAcceleration","       "},
        {PMDOPSetActualPosition,        "SetActualPosition","     "},
        {0x4E,                          "Invalid opcode!","       "},
        {0x4F,                          "Invalid opcode!","       "},
        
        {PMDOPGetKp,                    "GetKp","                 "},
        {PMDOPGetKi,                    "GetKi","                 "},
        {PMDOPGetKd,                    "GetKd","                 "},
        {0x53,                          "Invalid opcode!","       "},
        {PMDOPGetKvff,                  "GetKvff","               "},
        {0x55,                          "Invalid opcode!","       "},
        {PMDOPGetInterruptMask,         "GetInterruptMask","      "},
        {0x57,                          "Invalid opcode!","       "},
        {PMDOPGetJerk,                  "GetJerk","               "},
        {PMDOPGetGearRatio,             "GetGearRatio","          "},
        {0x5A,                          "Invalid opcode!","       "},
        {PMDOPMultiUpdate,              "MultiUpdate","           "},
        {0x5C,                          "Invalid opcode!","       "},
        {0x5D,                          "Invalid opcode!","       "},
        {0x5E,                          "Invalid opcode!","       "},
        {0x5F,                          "Invalid opcode!","       "},
        
        {0x60,                          "Invalid opcode!","       "},
        {PMDOPGetSampleTime,            "GetSampleTime","         "},
        {0x62,                          "Invalid opcode!","       "},
        {0x63,                          "Invalid opcode!","       "},
        {0x64,                          "Invalid opcode!","       "},
        {0x65,                          "Invalid opcode!","       "},
        {0x66,                          "Invalid opcode!","       "},
        {0x67,                          "Invalid opcode!","       "},
        {0x68,                          "Invalid opcode!","       "},
        {PMDOPGetMotorCommand,          "GetMotorCommand","       "},
        {PMDOPSetStartVelocity,         "SetStartVelocity","      "},
        {PMDOPGetStartVelocity,         "GetStartVelocity","      "},
        {0x6C,                          "Invalid opcode!","       "},
        {0x6D,                          "Invalid opcode!","       "},
        {PMDOPGetOutputMode,            "GetOutputMode","         "},
        {0x6F,                          "Invalid opcode!","       "},
        
        {0x70,                          "Invalid opcode!","       "},
        {0x71,                          "Invalid opcode!","       "},
        {PMDOPSetPhaseInitializeTime,   "SetPhaseInitializeTime",""},
        {0x73,                          "Invalid opcode!","       "},
        {0x74,                          "Invalid opcode!","       "},
        {PMDOPSetPhaseCounts,           "SetPhaseCounts","        "},
        {PMDOPSetPhaseOffset,           "SetPhaseOffset","        "},
        {PMDOPSetMotorCommand,          "SetMotorCommand","       "},
        {0x78,                          "Invalid opcode!","       "},
        {0x79,                          "Invalid opcode!","       "},
        {PMDOPInitializePhase,          "InitializePhase","       "},
        {PMDOPGetPhaseOffset,           "GetPhaseOffset","        "},
        {PMDOPGetPhaseInitializeTime,   "GetPhaseInitializeTime",""},
        {PMDOPGetPhaseCounts,           "GetPhaseCounts","        "},
        {0x7E,                          "Invalid opcode!","       "},
        {0x7F,                          "Invalid opcode!","       "},
        
        {PMDOPSetLimitSwitchMode,       "SetLimitSwitchMode","    "},
        {PMDOPGetLimitSwitchMode,       "GetLimitSwitchMode","    "},
        {PMDOPWriteIO,                  "WriteIO","               "},
        {PMDOPReadIO,                   "ReadIO","                "},
        {PMDOPSetPhaseAngle,            "SetPhaseAngle","         "},
        {PMDOPSetNumberPhases,          "SetNumberPhases","       "},
        {PMDOPGetNumberPhases,          "GetNumberPhases","       "},
        {PMDOPSetAxisMode,              "SetAxisMode","           "},
        {PMDOPGetAxisMode,              "GetAxisMode","           "},
        {PMDOPSetDiagnosticPortMode,    "SetDiagnosticPortMode"," "},
        {PMDOPGetDiagnosticPortMode,    "GetDiagnosticPortMode"," "},
        {PMDOPSetSerialPortMode,        "SetSerialPortMode","     "},
        {PMDOPGetSerialPortMode,        "GetSerialPortMode","     "},
        {PMDOPSetEncoderModulus,        "SetEncoderModulus","     "},
        {PMDOPGetEncoderModulus,        "GetEncoderModulus","     "},
        {PMDOPGetVersion,               "GetVersion","            "},
        
        {PMDOPSetAcceleration,          "SetAcceleration","       "},
        {PMDOPSetDeceleration,          "SetDeceleration","       "},
        {PMDOPGetDeceleration,          "GetDeceleration","       "},
        {PMDOPSetKaff,                  "SetKaff","               "},
        {PMDOPGetKaff,                  "GetKaff","               "},
        {PMDOPSetIntegrationLimit,      "SetIntegrationLimit","   "},
        {PMDOPGetIntegrationLimit,      "GetIntegrationLimit","   "},
        {PMDOPSetPositionErrorLimit,    "SetPositionErrorLimit"," "},
        {PMDOPGetPositionErrorLimit,    "GetPositionErrorLimit"," "},
        {PMDOPGetPositionError,         "GetPositionError","      "},
        {PMDOPGetIntegral,              "GetIntegral","           "},
        {PMDOPGetDerivative,            "GetDerivative","         "},
        {PMDOPSetDerivativeTime,        "SetDerivativeTime","     "},
        {PMDOPGetDerivativeTime,        "GetDerivativeTime","     "},
        {PMDOPSetKout,                  "SetKout","               "},
        {PMDOPGetKout,                  "GetKout","               "},
        
        {PMDOPSetProfileMode,           "SetProfileMode","        "},
        {PMDOPGetProfileMode,           "GetProfileMode","        "},
        {PMDOPSetSignalSense,           "SetSignalSense","        "},
        {PMDOPGetSignalSense,           "GetSignalSense","        "},
        {PMDOPGetSignalStatus,          "GetSignalStatus","       "},
        {PMDOPGetHostIOError,           "GetHostIOError","        "},
        {PMDOPGetActivityStatus,        "GetActivityStatus","     "},
        {PMDOPGetCommandedAcceleration, "GetCommandedAcceleration",""},
        {PMDOPSetTrackingWindow,        "SetTrackingWindow","     "},
        {PMDOPGetTrackingWindow,        "GetTrackingWindow","     "},
        {PMDOPSetSettleTime,            "SetSettleTime","         "},
        {PMDOPGetSettleTime,            "GetSettleTime","         "},
        {PMDOPClearInterrupt,           "ClearInterrupt","        "},
        {PMDOPGetActualVelocity,        "GetActualVelocity","     "},
        {PMDOPSetGearMaster,            "SetGearMaster","         "},
        {PMDOPGetGearMaster,            "GetGearMaster","         "},
        
        {PMDOPSetTraceMode,             "SetTraceMode","          "},
        {PMDOPGetTraceMode,             "GetTraceMode","          "},
        {PMDOPSetTraceStart,            "SetTraceStart","         "},
        {PMDOPGetTraceStart,            "GetTraceStart","         "},
        {PMDOPSetTraceStop,             "SetTraceStop","          "},
        {PMDOPGetTraceStop,             "GetTraceStop","          "},
        {PMDOPSetTraceVariable,         "SetTraceVariable","      "},
        {PMDOPGetTraceVariable,         "GetTraceVariable","      "},
        {PMDOPSetTracePeriod,           "SetTracePeriod","        "},
        {PMDOPGetTracePeriod,           "GetTracePeriod","        "},
        {PMDOPGetTraceStatus,           "GetTraceStatus","        "},
        {PMDOPGetTraceCount,            "GetTraceCount","         "},
        {PMDOPSetSettleWindow,          "SetSettleWindow","       "},
        {PMDOPGetSettleWindow,          "GetSettleWindow","       "},
        {PMDOPSetActualPositionUnits,   "SetActualPositionUnits",""},
        {PMDOPGetActualPositionUnits,   "GetActualPositionUnits",""},
        
        {PMDOPSetBufferStart,           "SetBufferStart","        "},
        {PMDOPGetBufferStart,           "GetBufferStart","        "},
        {PMDOPSetBufferLength,          "SetBufferLength","       "},
        {PMDOPGetBufferLength,          "GetBufferLength","       "},
        {PMDOPSetBufferWriteIndex,      "SetBufferWriteIndex","   "},
        {PMDOPGetBufferWriteIndex,      "GetBufferWriteIndex","   "},
        {PMDOPSetBufferReadIndex,       "SetBufferReadIndex","    "},
        {PMDOPGetBufferReadIndex,       "GetBufferReadIndex","    "},
        {PMDOPWriteBuffer,              "WriteBuffer","           "},
        {PMDOPReadBuffer,               "ReadBuffer","            "},
        {0xCA,                          "Invalid opcode!","       "},
        {0xCB,                          "Invalid opcode!","       "},
        {0xCC,                          "Invalid opcode!","       "},
        {0xCD,                          "Invalid opcode!","       "},
        {0xCE,                          "Invalid opcode!","       "},
        {0xCF,                          "Invalid opcode!","       "},
        
        {PMDOPSetStopMode,              "SetStopMode","           "},
        {PMDOPGetStopMode,              "GetStopMode","           "},
        {PMDOPSetAutoStopMode,          "SetAutoStopMode","       "},
        {PMDOPGetAutoStopMode,          "GetAutoStopMode","       "},
        {PMDOPSetBreakpoint,            "SetBreakpoint","         "},
        {PMDOPGetBreakpoint,            "GetBreakpoint","         "},
        {PMDOPSetBreakpointValue,       "SetBreakpointValue","    "},
        {PMDOPGetBreakpointValue,       "GetBreakpointValue","    "},
        {PMDOPSetCaptureSource,         "SetCaptureSource","      "},
        {PMDOPGetCaptureSource,         "GetCaptureSource","      "},
        {PMDOPSetEncoderSource,         "SetEncoderSource","      "},
        {PMDOPGetEncoderSource,         "GetEncoderSource","      "},
        {PMDOPSetMotorMode,             "SetMotorMode","          "},
        {PMDOPGetMotorMode,             "GetMotorMode","          "},
        {PMDOPSetEncoderToStepRatio,    "SetEncoderToStepRatio"," "},
        {PMDOPGetEncoderToStepRatio,    "GetEncoderToStepRatio"," "},
        
        {PMDOPSetOutputMode,            "SetOutputMode","         "},
        {PMDOPGetInterruptAxis,         "GetInterruptAxis","      "},
        {PMDOPSetCommutationMode,       "SetCommutationMode","    "},
        {PMDOPGetCommutationMode,       "GetCommutationMode","    "},
        {PMDOPSetPhaseInitializeMode,   "SetPhaseInitializeMode",""},
        {PMDOPGetPhaseInitializeMode,   "GetPhaseInitializeMode",""},
        {PMDOPSetPhasePrescale,         "SetPhasePrescale","      "},
        {PMDOPGetPhasePrescale,         "GetPhasePrescale","      "},
        {PMDOPSetPhaseCorrectionMode,   "SetPhaseCorrectionMode",""},
        {PMDOPGetPhaseCorrectionMode,   "GetPhaseCorrectionMode",""},
        {PMDOPGetPhaseCommand,          "GetPhaseCommand","       "},
        {PMDOPSetMotionCompleteMode,    "SetMotionCompleteMode"," "},
        {PMDOPGetMotionCompleteMode,    "GetMotionCompleteMode"," "},
        {PMDOPSetAxisOutSource,         "SetAxisOutSource","      "},
        {PMDOPGetAxisOutSource,         "GetAxisOutSource","      "},
        {PMDOPReadAnalog,               "ReadAnalog","            "},
        
        {0xF0,                          "Invalid opcode!","       "},
        {0xF1,                          "Invalid opcode!","       "},
        {0xF2,                          "Invalid opcode!","       "},
        {0xF3,                          "Invalid opcode!","       "},
        {0xF4,                          "Invalid opcode!","       "},
        {PMDOPAdjustActualPosition,     "AdjustActualPosition","  "},
        {0xF6,                          "Invalid opcode!","       "},
        {0xF7,                          "Invalid opcode!","       "},
        {PMDOPGetChecksum,              "GetChecksum","           "},
        {0xF9,                          "Invalid opcode!","       "},
        {0xFA,                          "Invalid opcode!","       "},
        {0xFB,                          "Invalid opcode!","       "},
        {0xFC,                          "Invalid opcode!","       "},
        {0xFD,                          "Invalid opcode!","       "},
        {0xFE,                          "Invalid opcode!","       "},
        {0xFF,                          "Invalid opcode!","       "}
};

#if defined(__cplusplus)
}
#endif

#endif

