#include <iostream>
#include <winsock2.h>
#include <cstring>
#include <cstdlib>
#include <iomanip> // 需要包含這個頭文件以使用 std::setprecision

#pragma comment(lib, "ws2_32.lib") // Winsock library

// 定義 Forza Horizon 4 的 telemetry 數據結構
struct TelemetryData {
    uint32_t IsRaceOn; // 1 when the race is on, 0 when in menus or race stopped
    uint32_t TimestampMS; // Timestamp in milliseconds, will eventually overflow to 0
    float EngineMaxRpm; // Maximum engine RPM
    float EngineIdleRpm; // Engine idle RPM
    float CurrentEngineRpm; // Current engine RPM
    float AccelerationX; // Acceleration in the car's local space; X = right, Y = up, Z = forward
    float AccelerationY; // Acceleration in the car's local space
    float AccelerationZ; // Acceleration in the car's local space
    float VelocityX; // Velocity in the car's local space; X = right, Y = up, Z = forward
    float VelocityY; // Velocity in the car's local space
    float VelocityZ; // Velocity in the car's local space
    float AngularVelocityX; // Angular velocity in the car's local space; X = pitch, Y = yaw, Z = roll
    float AngularVelocityY; // Angular velocity in the car's local space
    float AngularVelocityZ; // Angular velocity in the car's local space
    float Yaw; // Yaw angle
    float Pitch; // Pitch angle
    float Roll; // Roll angle
    float NormalizedSuspensionTravelFrontLeft; // Normalized suspension travel: 0.0f = max extension; 1.0 = max compression
    float NormalizedSuspensionTravelFrontRight; // Normalized suspension travel
    float NormalizedSuspensionTravelRearLeft; // Normalized suspension travel
    float NormalizedSuspensionTravelRearRight; // Normalized suspension travel
    float TireSlipRatioFrontLeft; // Normalized tire slip ratio, 0 means 100% grip, and |ratio| > 1.0 means loss of grip
    float TireSlipRatioFrontRight; // Normalized tire slip ratio
    float TireSlipRatioRearLeft; // Normalized tire slip ratio
    float TireSlipRatioRearRight; // Normalized tire slip ratio
    float WheelRotationSpeedFrontLeft; // Front left wheel rotation speed in radians/second
    float WheelRotationSpeedFrontRight; // Front right wheel rotation speed
    float WheelRotationSpeedRearLeft; // Rear left wheel rotation speed
    float WheelRotationSpeedRearRight; // Rear right wheel rotation speed
    int32_t WheelOnRumbleStripFrontLeft; // 1 when front left wheel is on the rumble strip, 0 when off
    int32_t WheelOnRumbleStripFrontRight; // 1 when front right wheel is on the rumble strip, 0 when off
    int32_t WheelOnRumbleStripRearLeft; // 1 when rear left wheel is on the rumble strip, 0 when off
    int32_t WheelOnRumbleStripRearRight; // 1 when rear right wheel is on the rumble strip, 0 when off
    float WheelInPuddleDepthFrontLeft; // Front left wheel's puddle depth, from 0 to 1, where 1 is the deepest puddle
    float WheelInPuddleDepthFrontRight; // Front right wheel's puddle depth
    float WheelInPuddleDepthRearLeft; // Rear left wheel's puddle depth
    float WheelInPuddleDepthRearRight; // Rear right wheel's puddle depth
    float SurfaceRumbleFrontLeft; // Non-dimensional surface rumble value passed to the controller for front left wheel
    float SurfaceRumbleFrontRight; // Non-dimensional surface rumble value passed to the controller for front right wheel
    float SurfaceRumbleRearLeft; // Non-dimensional surface rumble value passed to the controller for rear left wheel
    float SurfaceRumbleRearRight; // Non-dimensional surface rumble value passed to the controller for rear right wheel
    float TireSlipAngleFrontLeft; // Normalized tire slip angle, 0 means 100% grip, and |angle| > 1.0 means loss of grip
    float TireSlipAngleFrontRight; // Normalized tire slip angle
    float TireSlipAngleRearLeft; // Normalized tire slip angle
    float TireSlipAngleRearRight; // Normalized tire slip angle
    float TireCombinedSlipFrontLeft; // Normalized combined tire slip, 0 means 100% grip, and |slip| > 1.0 means loss of grip
    float TireCombinedSlipFrontRight; // Normalized combined tire slip
    float TireCombinedSlipRearLeft; // Normalized combined tire slip
    float TireCombinedSlipRearRight; // Normalized combined tire slip
    float SuspensionTravelMetersFrontLeft; // Actual suspension travel in meters for the front left wheel
    float SuspensionTravelMetersFrontRight; // Actual suspension travel in meters for the front right wheel
    float SuspensionTravelMetersRearLeft; // Actual suspension travel in meters for the rear left wheel
    float SuspensionTravelMetersRearRight; // Actual suspension travel in meters for the rear right wheel
    int32_t CarOrdinal; // Unique identifier for the car make/model
    int32_t CarClass; // Car class, between 0 (D - worst cars) and 7 (X class - best cars)
    int32_t CarPerformanceIndex; // Performance index of the car, between 100 (slowest) and 999 (fastest)
    int32_t DrivetrainType; // Corresponds to EDrivetrainType; 0 = FWD, 1 = RWD, 2 = AWD
    int32_t NumCylinders; // Number of cylinders in the engine
    float PositionX; // X position of the car in space
    float PositionY; // Y position of the car in space
    float PositionZ; // Z position of the car in space
    float Speed; // Speed in meters per second
    float Power; // Power in watts
    float Torque; // Torque in newton-meters
    float TireTempFrontLeft; // Tire temperature for front left wheel
    float TireTempFrontRight; // Tire temperature for front right wheel
    float TireTempRearLeft; // Tire temperature for rear left wheel
    float TireTempRearRight; // Tire temperature for rear right wheel
    float Boost; // Boost level
    float Fuel; // Fuel amount
    float DistanceTraveled; // Distance traveled
    float BestLap; // Best lap time
    float LastLap; // Last lap time
    float CurrentLap; // Current lap time
    float CurrentRaceTime; // Current race time
    uint16_t LapNumber; // Current lap number
    uint8_t RacePosition; // Position in the race
    uint8_t Accel; // Acceleration input value
    uint8_t Brake; // Brake input value
    uint8_t Clutch; // Clutch input value
    uint8_t HandBrake; // Handbrake input value
    uint8_t Gear; // Current gear
    uint8_t Steer; // Steering input value
    int8_t NormalizedDrivingLine; // Normalized driving line
    int8_t NormalizedAIBrakeDifference; // Normalized difference in AI braking

};

// 函數來解析數據包 , Function to parse the data packet
TelemetryData parseTelemetryData(const char* data) {
    TelemetryData telemetryData;

    // if FM please use 0.
    size_t offset = 12;

    // 使用 memcpy 複製每個字段, Use memcpy to copy each field.
    std::memcpy(&telemetryData.IsRaceOn, &data[0], sizeof(int32_t));
    std::memcpy(&telemetryData.TimestampMS, &data[4], sizeof(uint32_t));
    std::memcpy(&telemetryData.EngineMaxRpm, &data[8], sizeof(float));
    std::memcpy(&telemetryData.EngineIdleRpm, &data[12], sizeof(float));
    std::memcpy(&telemetryData.CurrentEngineRpm, &data[16], sizeof(float));
    std::memcpy(&telemetryData.AccelerationX, &data[20], sizeof(float));
    std::memcpy(&telemetryData.AccelerationY, &data[24], sizeof(float));
    std::memcpy(&telemetryData.AccelerationZ, &data[28], sizeof(float));
    std::memcpy(&telemetryData.VelocityX, &data[32], sizeof(float));
    std::memcpy(&telemetryData.VelocityY, &data[36], sizeof(float));
    std::memcpy(&telemetryData.VelocityZ, &data[40], sizeof(float));
    std::memcpy(&telemetryData.AngularVelocityX, &data[44], sizeof(float));
    std::memcpy(&telemetryData.AngularVelocityY, &data[48], sizeof(float));
    std::memcpy(&telemetryData.AngularVelocityZ, &data[52], sizeof(float));
    std::memcpy(&telemetryData.Yaw, &data[56], sizeof(float));
    std::memcpy(&telemetryData.Pitch, &data[60], sizeof(float));
    std::memcpy(&telemetryData.Roll, &data[64], sizeof(float));
    std::memcpy(&telemetryData.NormalizedSuspensionTravelFrontLeft, &data[68], sizeof(float));
    std::memcpy(&telemetryData.NormalizedSuspensionTravelFrontRight, &data[72], sizeof(float));
    std::memcpy(&telemetryData.NormalizedSuspensionTravelRearLeft, &data[76], sizeof(float));
    std::memcpy(&telemetryData.NormalizedSuspensionTravelRearRight, &data[80], sizeof(float));
    std::memcpy(&telemetryData.TireSlipRatioFrontLeft, &data[84], sizeof(float));
    std::memcpy(&telemetryData.TireSlipRatioFrontRight, &data[88], sizeof(float));
    std::memcpy(&telemetryData.TireSlipRatioRearLeft, &data[92], sizeof(float));
    std::memcpy(&telemetryData.TireSlipRatioRearRight, &data[96], sizeof(float));
    std::memcpy(&telemetryData.WheelRotationSpeedFrontLeft, &data[100], sizeof(float));
    std::memcpy(&telemetryData.WheelRotationSpeedFrontRight, &data[104], sizeof(float));
    std::memcpy(&telemetryData.WheelRotationSpeedRearLeft, &data[108], sizeof(float));
    std::memcpy(&telemetryData.WheelRotationSpeedRearRight, &data[112], sizeof(float));
    std::memcpy(&telemetryData.WheelOnRumbleStripFrontLeft, &data[116], sizeof(int32_t));
    std::memcpy(&telemetryData.WheelOnRumbleStripFrontRight, &data[120], sizeof(int32_t));
    std::memcpy(&telemetryData.WheelOnRumbleStripRearLeft, &data[124], sizeof(int32_t));
    std::memcpy(&telemetryData.WheelOnRumbleStripRearRight, &data[128], sizeof(int32_t));
    std::memcpy(&telemetryData.WheelInPuddleDepthFrontLeft, &data[132], sizeof(float));
    std::memcpy(&telemetryData.WheelInPuddleDepthFrontRight, &data[136], sizeof(float));
    std::memcpy(&telemetryData.WheelInPuddleDepthRearLeft, &data[140], sizeof(float));
    std::memcpy(&telemetryData.WheelInPuddleDepthRearRight, &data[144], sizeof(float));
    std::memcpy(&telemetryData.SurfaceRumbleFrontLeft, &data[148], sizeof(float));
    std::memcpy(&telemetryData.SurfaceRumbleFrontRight, &data[152], sizeof(float));
    std::memcpy(&telemetryData.SurfaceRumbleRearLeft, &data[156], sizeof(float));
    std::memcpy(&telemetryData.SurfaceRumbleRearRight, &data[160], sizeof(float));
    std::memcpy(&telemetryData.TireSlipAngleFrontLeft, &data[164], sizeof(float));
    std::memcpy(&telemetryData.TireSlipAngleFrontRight, &data[168], sizeof(float));
    std::memcpy(&telemetryData.TireSlipAngleRearLeft, &data[172], sizeof(float));
    std::memcpy(&telemetryData.TireSlipAngleRearRight, &data[176], sizeof(float));
    std::memcpy(&telemetryData.TireCombinedSlipFrontLeft, &data[180], sizeof(float));
    std::memcpy(&telemetryData.TireCombinedSlipFrontRight, &data[184], sizeof(float));
    std::memcpy(&telemetryData.TireCombinedSlipRearLeft, &data[188], sizeof(float));
    std::memcpy(&telemetryData.TireCombinedSlipRearRight, &data[192], sizeof(float));
    std::memcpy(&telemetryData.SuspensionTravelMetersFrontLeft, &data[196], sizeof(float));
    std::memcpy(&telemetryData.SuspensionTravelMetersFrontRight, &data[200], sizeof(float));
    std::memcpy(&telemetryData.SuspensionTravelMetersRearLeft, &data[204], sizeof(float));
    std::memcpy(&telemetryData.SuspensionTravelMetersRearRight, &data[208], sizeof(float));
    std::memcpy(&telemetryData.CarOrdinal, &data[212], sizeof(int32_t));
    std::memcpy(&telemetryData.CarClass, &data[216], sizeof(int32_t));
    std::memcpy(&telemetryData.CarPerformanceIndex, &data[220], sizeof(int32_t));
    std::memcpy(&telemetryData.DrivetrainType, &data[224], sizeof(int32_t));
    std::memcpy(&telemetryData.NumCylinders, &data[228], sizeof(int32_t));
    std::memcpy(&telemetryData.PositionX, &data[offset + 232], sizeof(float));
    std::memcpy(&telemetryData.PositionY, &data[offset + 236], sizeof(float));
    std::memcpy(&telemetryData.PositionZ, &data[offset + 240], sizeof(float));
    std::memcpy(&telemetryData.Speed, &data[offset + 244], sizeof(float));
    std::memcpy(&telemetryData.Power, &data[offset + 248], sizeof(float));
    std::memcpy(&telemetryData.Torque, &data[offset + 252], sizeof(float));
    std::memcpy(&telemetryData.TireTempFrontLeft, &data[offset + 256], sizeof(float));
    std::memcpy(&telemetryData.TireTempFrontRight, &data[offset + 260], sizeof(float));
    std::memcpy(&telemetryData.TireTempRearLeft, &data[offset + 264], sizeof(float));
    std::memcpy(&telemetryData.TireTempRearRight, &data[offset + 268], sizeof(float));
    std::memcpy(&telemetryData.Boost, &data[offset + 272], sizeof(float));
    std::memcpy(&telemetryData.Fuel, &data[offset + 276], sizeof(float));
    std::memcpy(&telemetryData.DistanceTraveled, &data[offset + 280], sizeof(float));
    std::memcpy(&telemetryData.BestLap, &data[offset + 284], sizeof(float));
    std::memcpy(&telemetryData.LastLap, &data[offset + 288], sizeof(float));
    std::memcpy(&telemetryData.CurrentLap, &data[offset + 292], sizeof(float));
    std::memcpy(&telemetryData.CurrentRaceTime, &data[offset + 296], sizeof(float));
    std::memcpy(&telemetryData.LapNumber, &data[offset + 300], sizeof(uint16_t));
    std::memcpy(&telemetryData.RacePosition, &data[offset + 302], sizeof(uint8_t));
    std::memcpy(&telemetryData.Accel, &data[offset + 303], sizeof(uint8_t));
    std::memcpy(&telemetryData.Brake, &data[offset + 304], sizeof(uint8_t));
    std::memcpy(&telemetryData.Clutch, &data[offset + 305], sizeof(uint8_t));
    std::memcpy(&telemetryData.HandBrake, &data[offset + 306], sizeof(uint8_t));
    std::memcpy(&telemetryData.Gear, &data[offset + 307], sizeof(uint8_t));
    std::memcpy(&telemetryData.Steer, &data[offset + 308], sizeof(uint8_t));
    std::memcpy(&telemetryData.NormalizedDrivingLine, &data[offset + 309], sizeof(uint8_t));
    std::memcpy(&telemetryData.NormalizedAIBrakeDifference, &data[offset + 310], sizeof(uint8_t));


    return telemetryData;
}

int main() {
    WSADATA wsaData;
    SOCKET sock;
    sockaddr_in server;
    const int bufferSize = 1024; // 假設數據包大小, Assume the size of the data packet.
    char buffer[bufferSize];

    // 初始化 Winsock, Initialize Winsock.
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed. Error Code: " << WSAGetLastError() << std::endl;
        return 1;
    }

    // 創建 socket, Create socket.
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        std::cerr << "Socket creation failed. Error Code: " << WSAGetLastError() << std::endl;
        WSACleanup();
        return 1;
    }

    // 設定伺服器地址, Set up the server address.
    server.sin_family = AF_INET;
    server.sin_port = htons(9999); // 設定為 Forza 的傳輸端口
    server.sin_addr.s_addr = INADDR_ANY;

    // 綁定 socket, Bind the socket.
    if (bind(sock, (struct sockaddr*)&server, sizeof(server)) == SOCKET_ERROR) {
        std::cerr << "Binding failed. Error Code: " << WSAGetLastError() << std::endl;
        closesocket(sock);
        WSACleanup();
        return 1;
    }

    std::cout << "Waiting for telemetry data..." << std::endl;

    while (true) {
        // 接收數據包, Receive the data packet.
        int recvLen = recv(sock, buffer, bufferSize, 0);
        if (recvLen == SOCKET_ERROR) {
            std::cerr << "Receiving failed. Error Code: " << WSAGetLastError() << std::endl;
            break;
        }

        // 解析數據包, Parse the data packet.
        TelemetryData telemetryData = parseTelemetryData(buffer);

        // 檢查 CurrentEngineRpm 是否為 0, Check if `CurrentEngineRpm` is 0.
        if (telemetryData.CurrentEngineRpm == 0) {
            // 如果為 0，則暫停讀取及列印, If it is 0, pause reading and printing.
            continue; // 跳過本次循環，回到 while 開頭, Skip this iteration and return to the start of the while loop.
        }
        int intGEAR = (int)telemetryData.Gear;
        // 輸出解析的數據, Output the parsed data.
        std::cout 
            << "Gear = " << intGEAR            
            << std::fixed << std::setprecision(1) // 設定固定格式與一位小數, Set a fixed format with one decimal place.
            << " Speed: "<< telemetryData.Speed * 2.237 * 1.609344 << " RPM: " << ((telemetryData.CurrentEngineRpm - telemetryData.EngineIdleRpm) / (telemetryData.EngineMaxRpm - telemetryData.EngineIdleRpm))
            << " Slip: " << sqrt(pow(telemetryData.TireSlipRatioFrontLeft,2)
                + pow(telemetryData.TireSlipRatioFrontRight,2) 
                + pow(telemetryData.TireSlipRatioRearLeft,2)
                + pow(telemetryData.TireSlipRatioRearRight,2) )
            << " Acceleration: " << 
                sqrt(pow(telemetryData.AccelerationZ, 2)
                + pow(telemetryData.AccelerationY, 2))
             << std::endl;
       
    }

    // 關閉 socket, close the socket.
    closesocket(sock);
    WSACleanup();
    return 0;
}
