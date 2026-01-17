//! Combined Aurora Tracker + STM32 Motor Controller - DUAL SENSOR VERSION
//! Features: Continuous CSV logging, threaded angle calculation, dual-angle transmission
//! WITH VALIDATION: Ignores invalid sensor readings with extreme values
//! UPDATED: Direct X/Y input with validation
//! INTEGRATED: Full Inverse Kinematics solver for 3-servo + linear motor system
//! COORDINATE SYSTEM: Sensor pad origin with X inverted, Y offset +280mm
//! FIXED: Z equation updated to use sin(theta2) instead of cos(theta2)
//! NEW: Main menu with Tracking and Moving modes
//! BUGFIX: Added proper thread joining in runTrackingMode() to prevent abort() crash
//----------------------------------------------------------------------------
#define NOMINMAX
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <algorithm>
#include "CombinedApi.h"
#include "PortHandleInfo.h"
#include "ToolData.h"

// Serial port handling
static HANDLE hSTM32Serial = INVALID_HANDLE_VALUE;

// Track actual y_linear position that has been sent to STM32
static float g_stm32_current_y_linear = 0.0f;

// System parameters
#define SAMPLING_RATE 1
#define REV_MM 0.25
#define SENSOR2PIVOT 39  // Distance between sensor 2 and pivot point (mm)
#define SENSOR1PIVOT 52  // Distance between sensor 1 and pivot point (mm)
#define STM32_PORT "\\\\.\\COM4"  // Change to your STM32 port
#define STM32_BAUDRATE CBR_9600
#define STM32_TIMEOUT 30  // 30 second timeout for motor movement

// Coordinate system transformation
#define Y_OFFSET 280.0   // Add 280mm to all Y coordinates

// System geometry constants
#define ARM_LENGTH 150           // Total arm length (mm)
#define SERVO_ANGLE_MIN 45.0     // Servo 1 minimum angle (degrees)
#define SERVO_ANGLE_MAX 135.0    // Servo 1 maximum angle (degrees)
#define SERVO2_ANGLE_MIN 45.0    // Servo 2 min angle
#define SERVO2_ANGLE_MAX 135.0   // Servo 2 max angle
#define SERVO3_ANGLE_MIN 0.0     // Servo 3 min angle
#define SERVO3_ANGLE_MAX 90.0    // Servo 3 max angle
#define ARM_INITIAL 170

// Link dimensions (mm) - ADJUST THESE TO MATCH YOUR ROBOT
#define LINK1 79.0       // Length of Link 1
#define LINK2Y 24.0      // Link between first and second servo (Y component)
#define LINK3Z 52.0      // Link 3 Z component
#define LINK3Y 63.0      // Link 3 Y component

// Computed constants
const double PI = 3.14159265358979323846;
const double THETA3Y = std::atan(LINK3Y / LINK3Z);
const double LINK3 = sqrt(LINK3Y * LINK3Y + LINK3Z * LINK3Z);

// Y_linear bounds (linear rail travel)
const double Y_LINEAR_MIN = 0.0;
const double Y_LINEAR_MAX = ARM_LENGTH * (3.0 / 4.0);  // 100mm

// Control parameters
#define CONTROL_UPDATE_INTERVAL_MS 2000  // Calculate and send angles every 2000ms
#define CSV_LOG_INTERVAL_MS 50           // Log to CSV every 50ms (20Hz)
#define THRESHOLD 1.5                      // Position threshold (mm)

// Validation parameters
#define MAX_VALID_COORDINATE 10000.0
#define MIN_VALID_COORDINATE -10000.0
#define MAX_VALID_QUATERNION 1.1
#define MIN_VALID_QUATERNION -1.1

// Position streaming interval for Moving mode
#define POSITION_STREAM_INTERVAL_MS 50

static CombinedApi capi = CombinedApi();
static bool apiSupportsBX2 = false;

// Thread-safe data structure for latest sensor data
struct SensorData {
    std::mutex mtx;
    Transform transform1;
    Transform transform2;
    bool valid1;
    bool valid2;
    long long timestamp;
};

// Shared control data
struct ControlData {
    std::mutex mtx;
    double x_ref;
    double y_ref;
    double z_ref;
    // Sensor 2 initial position (used for X and Z equations)
    double x0_initial;  // Sensor 2 initial X position (after transformation)
    double y0_initial;  // Sensor 2 initial Y position (after transformation) - NOT used in IK
    double z0_initial;  // Sensor 2 initial Z position (after transformation)
    // Sensor 1 initial position (used for Y equation)
    double y0_sensor1;  // Sensor 1 initial Y position (after transformation) - used in Y equation
    double current_servo_angle;    // Track current servo 1 angle
    double current_servo2_angle;   // Track current servo 2 angle
    double current_servo3_angle;   // Track current servo 3 angle
    double current_y_linear;       // Track current linear position
    // Commanded angles (adjusted by error feedback)
    double cmd_theta1;
    double cmd_theta2;
    double cmd_theta3;
    double cmd_y_linear;
    bool ik_computed;              // Flag: IK has been computed for current target
    bool target_active;
    bool target_reached;
    bool calibrated;  // Flag to indicate if initial position has been calibrated
};

// Waypoint structure
struct wayPoint {
    double x;
    double y;
    double z;
};

static std::vector<wayPoint> currentBatch;
static std::vector<wayPoint> allPoints;

// Global workspace bounds (calculated after calibration)
static double g_workspace_x_min = 0, g_workspace_x_max = 0;
static double g_workspace_y_min = 0, g_workspace_y_max = 0;
static double g_workspace_z_min = 0, g_workspace_z_max = 0;

static SensorData g_sensorData;
static ControlData g_controlData;
static std::atomic<bool> g_running(false);
static std::atomic<bool> g_pausePositionStream(false);
static std::atomic<bool> g_trackingModeRunning(false);  // BUGFIX: Added flag to control tracking thread

// Statistics
static std::atomic<int> g_frameCount(0);
static std::atomic<int> g_validFrames(0);
static std::atomic<int> g_missingFrames(0);
static std::atomic<int> g_invalidFrames(0);
static std::atomic<int> g_stm32CommandsSent(0);
static std::atomic<int> g_stm32CommandsCompleted(0);

// Track y_linear position for Moving mode (global so it can be reset from home)
static float g_moving_mode_y_linear = 0.0f;

//============================================================================
// UTILITY FUNCTIONS (FORWARD DECLARATIONS)
//============================================================================

void sleepMilliseconds(unsigned numMilliseconds);
void transformCoordinates(const Transform& t, double& x_out, double& y_out, double& z_out);
bool sendAnglesToSTM32AndWait(float y_linear_target, float angle_servo1, float angle_servo2, float angle_servo3, float* current_y_linear);

//============================================================================
// INVERSE KINEMATICS SOLVER
//============================================================================

/**
 * @brief Solve inverse kinematics to find servo angles and linear position
 */
bool solveInverseKinematics(
    double x_target, double y_target, double z_target,
    double x0, double y0_sensor1, double z0,
    double& theta1_out, double& theta2_out, double& theta3_out, double& y_linear_out,
    bool verbose = false)
{
    const double DEG_TO_RAD = PI / 180.0;
    const double RAD_TO_DEG = 180.0 / PI;
    const double THETA2_STEP = 1.0;

    int rejected_sin_theta2_zero = 0;
    int rejected_cos_theta3_invalid = 0;
    int rejected_theta3_out_of_range = 0;
    int rejected_cos_theta1_invalid = 0;
    int rejected_theta1_out_of_range = 0;
    int rejected_y_linear_out_of_range = 0;
    int valid_solutions = 0;

    bool solution_found = false;
    double best_y_linear_abs = 1e9;
    double best_theta1 = 0, best_theta2 = 0, best_theta3 = 0, best_y_linear = 0;

    for (double theta2_deg = SERVO2_ANGLE_MIN; theta2_deg <= SERVO2_ANGLE_MAX; theta2_deg += THETA2_STEP)
    {
        double theta2_rad = theta2_deg * DEG_TO_RAD;
        double sin_theta2 = sin(theta2_rad);
        double denominator = LINK3 * sin_theta2;

        if (fabs(denominator) < 0.0001)
        {
            rejected_sin_theta2_zero++;
            continue;
        }

        double cos_theta3_with_offset = (LINK2Y * sin_theta2 + z0 + LINK3Z - LINK2Y - z_target) / denominator;

        if (cos_theta3_with_offset < -1.0 || cos_theta3_with_offset > 1.0)
        {
            rejected_cos_theta3_invalid++;
            continue;
        }

        double theta3_with_offset_rad = acos(cos_theta3_with_offset);
        double theta3_rad = theta3_with_offset_rad - THETA3Y;
        double theta3_deg = theta3_rad * RAD_TO_DEG;

        if (theta3_deg < SERVO3_ANGLE_MIN || theta3_deg > SERVO3_ANGLE_MAX)
        {
            rejected_theta3_out_of_range++;
            continue;
        }

        double dist_cam_y = LINK3 * cos_theta3_with_offset - LINK2Y;
        double cos_theta2 = cos(theta2_rad);
        double cos_theta1 = (x_target - x0 + cos_theta2 * dist_cam_y) / LINK1;

        if (cos_theta1 < -1.0 || cos_theta1 > 1.0)
        {
            rejected_cos_theta1_invalid++;
            continue;
        }

        double theta1_rad = acos(cos_theta1);
        double theta1_deg = theta1_rad * RAD_TO_DEG;

        if (theta1_deg < SERVO_ANGLE_MIN || theta1_deg > SERVO_ANGLE_MAX)
        {
            rejected_theta1_out_of_range++;
            continue;
        }

        double y_linear = y_target - y0_sensor1 - SENSOR1PIVOT - LINK1 * sin(theta1_rad) - LINK3 * sin(theta3_with_offset_rad);

        if (y_linear < Y_LINEAR_MIN || y_linear > Y_LINEAR_MAX)
        {
            rejected_y_linear_out_of_range++;
            continue;
        }

        valid_solutions++;

        if (verbose)
        {
            std::cout << "    [VALID] S1=" << std::fixed << std::setprecision(2) << theta1_deg
                << " S2=" << theta2_deg
                << " S3=" << theta3_deg
                << " y_lin=" << y_linear << std::endl;
        }

        if (fabs(y_linear) < best_y_linear_abs)
        {
            solution_found = true;
            best_y_linear_abs = fabs(y_linear);
            best_theta1 = theta1_deg;
            best_theta2 = theta2_deg;
            best_theta3 = theta3_deg;
            best_y_linear = y_linear;
        }
    }

    if (verbose)
    {
        std::cout << "  [IK DEBUG] Valid solutions: " << valid_solutions << std::endl;
    }

    if (solution_found)
    {
        theta1_out = best_theta1;
        theta2_out = best_theta2;
        theta3_out = best_theta3;
        y_linear_out = best_y_linear;

        if (verbose)
        {
            std::cout << "  [IK] Solution: S1=" << std::fixed << std::setprecision(2) << best_theta1
                << " deg, S2=" << best_theta2
                << " deg, S3=" << best_theta3
                << " deg, y_lin=" << best_y_linear << "mm" << std::endl;
        }
        return true;
    }

    if (verbose)
    {
        std::cout << "  [IK ERROR] No valid solution found!" << std::endl;
    }
    return false;
}

/**
 * @brief Forward kinematics - calculate position from angles
 */
void forwardKinematics(
    double theta1_deg, double theta2_deg, double theta3_deg, double y_linear,
    double x0, double y0_sensor1, double z0,
    double& x_out, double& y_out, double& z_out)
{
    double theta1_rad = theta1_deg * PI / 180.0;
    double theta2_rad = theta2_deg * PI / 180.0;
    double theta3_rad = theta3_deg * PI / 180.0;
    double theta3_with_offset_rad = theta3_rad + THETA3Y;

    double cos_theta3_with_offset = cos(theta3_with_offset_rad);
    double sin_theta3_with_offset = sin(theta3_with_offset_rad);
    double cos_theta2 = cos(theta2_rad);
    double sin_theta2 = sin(theta2_rad);

    double dist_cam_y = LINK3 * cos_theta3_with_offset - LINK2Y;

    x_out = x0 + LINK1 * cos(theta1_rad) - cos_theta2 * dist_cam_y;
    y_out = y0_sensor1 + SENSOR1PIVOT + y_linear + LINK1 * sin(theta1_rad) + LINK3 * sin_theta3_with_offset;
    z_out = z0 + LINK3Z - LINK3 * cos_theta3_with_offset + (LINK3 * cos_theta3_with_offset - LINK2Y) * (1.0 - sin_theta2);
}

/**
 * @brief Analyze workspace bounds
 */
void analyzeWorkspace(double x0, double y0_sensor1, double z0,
    double& x_min_out, double& x_max_out,
    double& y_min_out, double& y_max_out,
    double& z_min_out, double& z_max_out)
{
    std::cout << "\n===========================================\n";
    std::cout << "  WORKSPACE ANALYSIS\n";
    std::cout << "===========================================\n";

    double x_min = 1e9, x_max = -1e9;
    double y_min = 1e9, y_max = -1e9;
    double z_min = 1e9, z_max = -1e9;

    const double ANGLE_STEP = 2.0;
    const double LINEAR_STEP = 10.0;
    int total_points = 0;

    for (double theta1 = SERVO_ANGLE_MIN; theta1 <= SERVO_ANGLE_MAX; theta1 += ANGLE_STEP)
    {
        for (double theta2 = SERVO2_ANGLE_MIN; theta2 <= SERVO2_ANGLE_MAX; theta2 += ANGLE_STEP)
        {
            for (double theta3 = SERVO3_ANGLE_MIN; theta3 <= SERVO3_ANGLE_MAX; theta3 += ANGLE_STEP)
            {
                for (double y_lin = Y_LINEAR_MIN; y_lin <= Y_LINEAR_MAX; y_lin += LINEAR_STEP)
                {
                    double x, y, z;
                    forwardKinematics(theta1, theta2, theta3, y_lin, x0, y0_sensor1, z0, x, y, z);
                    total_points++;

                    if (x < x_min) x_min = x;
                    if (x > x_max) x_max = x;
                    if (y < y_min) y_min = y;
                    if (y > y_max) y_max = y;
                    if (z < z_min) z_min = z;
                    if (z > z_max) z_max = z;
                }
            }
        }
    }

    //std::cout << "Scanned " << total_points << " configurations\n" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  X: [" << x_min << ", " << x_max << "] mm" << std::endl;
    std::cout << "  Y: [" << y_min << ", " << y_max << "] mm" << std::endl;
    std::cout << "  Z: [" << z_min << ", " << z_max << "] mm" << std::endl;

    x_min_out = x_min; x_max_out = x_max;
    y_min_out = y_min; y_max_out = y_max;
    z_min_out = z_min; z_max_out = z_max;
}

void displayWorkspaceBounds(double x_min, double x_max,
    double y_min, double y_max,
    double z_min, double z_max)
{
    std::cout << "\n--- Reachable Workspace Bounds ---" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  X: [" << x_min << ", " << x_max << "] mm" << std::endl;
    std::cout << "  Y: [" << y_min << ", " << y_max << "] mm" << std::endl;
    std::cout << "  Z: [" << z_min << ", " << z_max << "] mm" << std::endl;
}

//============================================================================
// VALIDATION AND UTILITY FUNCTIONS
//============================================================================

bool isTransformValid(const Transform& t)
{
    if (t.isMissing()) return false;
    if (std::isnan(t.tx) || std::isnan(t.ty) || std::isnan(t.tz)) return false;
    if (std::isinf(t.tx) || std::isinf(t.ty) || std::isinf(t.tz)) return false;

    if (t.tx < MIN_VALID_COORDINATE || t.tx > MAX_VALID_COORDINATE ||
        t.ty < MIN_VALID_COORDINATE || t.ty > MAX_VALID_COORDINATE ||
        t.tz < MIN_VALID_COORDINATE || t.tz > MAX_VALID_COORDINATE)
        return false;

    if (std::isnan(t.q0) || std::isnan(t.qx) || std::isnan(t.qy) || std::isnan(t.qz)) return false;
    if (std::isinf(t.q0) || std::isinf(t.qx) || std::isinf(t.qy) || std::isinf(t.qz)) return false;

    double quat_mag = sqrt(t.q0 * t.q0 + t.qx * t.qx + t.qy * t.qy + t.qz * t.qz);
    if (quat_mag < 0.9 || quat_mag > 1.1) return false;

    return true;
}

void transformCoordinates(const Transform& t, double& x_out, double& y_out, double& z_out)
{
    x_out = -t.tx;
    y_out = t.ty + Y_OFFSET;
    z_out = -t.tz;
}

bool validateTargetPosition(double x, double y, double z, double x0, double y0_sensor1, double z0)
{
    double theta1, theta2, theta3, y_linear;
    return solveInverseKinematics(x, y, z, x0, y0_sensor1, z0, theta1, theta2, theta3, y_linear, false);
}

void sleepMilliseconds(unsigned numMilliseconds)
{
#ifdef _WIN32
    Sleep((DWORD)numMilliseconds);
#else
    usleep(numMilliseconds * 1000);
#endif
}

long long getCurrentTimeMs()
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

void determineApiSupportForBX2()
{
    std::string response = capi.getApiRevision();
    char deviceFamily = response[0];
    int majorVersion = capi.stringToInt(response.substr(2, 3));

    if (deviceFamily == 'G' && majorVersion >= 3)
    {
        apiSupportsBX2 = true;
    }
}

//============================================================================
// SERIAL COMMUNICATION
//============================================================================

HANDLE openSerialPort(const char* port, DWORD baudrate)
{
    HANDLE hSerial = CreateFileA(port,
        GENERIC_READ | GENERIC_WRITE,
        0, NULL, OPEN_EXISTING, 0, NULL);

    if (hSerial == INVALID_HANDLE_VALUE)
    {
        std::cerr << "ERROR: Could not open serial port " << port << std::endl;
        return INVALID_HANDLE_VALUE;
    }

    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams))
    {
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    dcbSerialParams.BaudRate = baudrate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams))
    {
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(hSerial, &timeouts))
    {
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    return hSerial;
}

HANDLE openSTM32Serial(const char* port)
{
    return openSerialPort(port, STM32_BAUDRATE);
}

/**
 * @brief Send angles to STM32 and wait for completion
 */
bool sendAnglesToSTM32AndWait(float y_linear_target, float angle_servo1, float angle_servo2, float angle_servo3, float* current_y_linear)
{
    if (hSTM32Serial == INVALID_HANDLE_VALUE)
    {
        std::cerr << "ERROR: STM32 serial port not open!" << std::endl;
        return false;
    }

    float delta_y_linear = y_linear_target - (*current_y_linear);
    float theta_linear = -(delta_y_linear / REV_MM) * 360.0f;

    char buffer[128];
    snprintf(buffer, sizeof(buffer), "L:%.2f,S1:%.2f,S2:%.2f,S3:%.2f\n",
        theta_linear, angle_servo1, angle_servo2, angle_servo3);

    DWORD bytes_written;
    DWORD data_len = (DWORD)strlen(buffer);

    if (!WriteFile(hSTM32Serial, buffer, data_len, &bytes_written, NULL) ||
        bytes_written != data_len)
    {
        std::cerr << "ERROR: Failed to send data to STM32" << std::endl;
        return false;
    }

    std::cout << "  -> Sent to STM32: " << buffer;
    std::cout << "  -> Waiting for completion..." << std::flush;

    time_t start_time = time(NULL);
    char rx_buffer[256] = { 0 };
    int buffer_pos = 0;

    while (difftime(time(NULL), start_time) < STM32_TIMEOUT)
    {
        DWORD bytes_read;
        char temp_buf[64];

        if (ReadFile(hSTM32Serial, temp_buf, sizeof(temp_buf) - 1, &bytes_read, NULL))
        {
            if (bytes_read > 0)
            {
                temp_buf[bytes_read] = '\0';

                for (DWORD i = 0; i < bytes_read && buffer_pos < 255; i++)
                {
                    rx_buffer[buffer_pos++] = temp_buf[i];
                }
                rx_buffer[buffer_pos] = '\0';

                if (strchr(rx_buffer, '1') != NULL)
                {
                    std::cout << " OK" << std::endl;
                    *current_y_linear = y_linear_target;
                    return true;
                }
            }
        }

        Sleep(10);
    }

    std::cout << " TIMEOUT" << std::endl;
    return false;
}

/**
 * @brief Send angles to STM32 without waiting (for Moving mode)
 */
bool sendAnglesToSTM32NoWait(float theta_linear, float angle_servo1, float angle_servo2, float angle_servo3)
{
    if (hSTM32Serial == INVALID_HANDLE_VALUE)
    {
        std::cerr << "ERROR: STM32 serial port not open!" << std::endl;
        return false;
    }

    char buffer[128];
    snprintf(buffer, sizeof(buffer), "L:%.2f,S1:%.2f,S2:%.2f,S3:%.2f\n",
        theta_linear, angle_servo1, angle_servo2, angle_servo3);

    DWORD bytes_written;
    DWORD data_len = (DWORD)strlen(buffer);

    if (!WriteFile(hSTM32Serial, buffer, data_len, &bytes_written, NULL) ||
        bytes_written != data_len)
    {
        std::cerr << "ERROR: Failed to send data to STM32" << std::endl;
        return false;
    }

    std::cout << "  -> Sent to STM32: " << buffer;
    return true;
}

/**
 * @brief Send position data to OpenMV via STM32 UART
 */
bool sendPositionToOpenMV(double x, double y, double z, bool verbose = false)
{
    if (hSTM32Serial == INVALID_HANDLE_VALUE)
    {
        if (verbose) std::cerr << "[Position] ERROR: Serial port not open" << std::endl;
        return false;
    }

    char buffer[64];
    snprintf(buffer, sizeof(buffer), "P:%.1f,%.1f,%.1f\n", x, y, z);

    DWORD bytes_written;
    BOOL result = WriteFile(hSTM32Serial, buffer, (DWORD)strlen(buffer), &bytes_written, NULL);

    if (verbose)
    {
        if (result && bytes_written == strlen(buffer))
        {
            std::cout << "[Position] Sent: " << buffer;
        }
        else
        {
            std::cerr << "[Position] FAILED to send: " << buffer;
        }
    }

    return (result && bytes_written == strlen(buffer));
}

//============================================================================
// THREADS
//============================================================================

/**
 * @brief Thread: Stream position to STM32 for OpenMV
 */
void positionStreamingThread()
{
    //std::cout << "[Position Stream Thread] Started" << std::endl;

    while (g_running)
    {
        if (g_pausePositionStream)
        {
            sleepMilliseconds(10);
            continue;
        }

        Transform t2;
        bool valid2;

        {
            std::lock_guard<std::mutex> lock(g_sensorData.mtx);
            t2 = g_sensorData.transform2;
            valid2 = g_sensorData.valid2;
        }

        if (valid2 && hSTM32Serial != INVALID_HANDLE_VALUE)
        {
            double x, y, z;
            transformCoordinates(t2, x, y, z);
            sendPositionToOpenMV(x, y, z, false);
        }

        sleepMilliseconds(POSITION_STREAM_INTERVAL_MS);
    }

    std::cout << "[Position Stream Thread] Stopped" << std::endl;
}

/**
 * @brief Thread: Continuously read Aurora sensors
 */
void sensorReadingThread()
{
    //std::cout << "[Sensor Thread] Started" << std::endl;

    while (g_running)
    {
        std::vector<ToolData> toolData = capi.getTrackingDataBX(
            TrackingReplyOption::TransformData | TrackingReplyOption::AllTransforms);

        if (toolData.size() >= 2)
        {
            bool valid1 = isTransformValid(toolData[0].transform);
            bool valid2 = isTransformValid(toolData[1].transform);

            {
                std::lock_guard<std::mutex> lock(g_sensorData.mtx);
                g_sensorData.transform1 = toolData[0].transform;
                g_sensorData.transform2 = toolData[1].transform;
                g_sensorData.valid1 = valid1;
                g_sensorData.valid2 = valid2;
                g_sensorData.timestamp = getCurrentTimeMs();
            }

            if (!valid1 || !valid2)
            {
                if (toolData[0].transform.isMissing() || toolData[1].transform.isMissing())
                    g_missingFrames++;
                else
                    g_invalidFrames++;
            }
            else
            {
                g_validFrames++;
            }
        }

        g_frameCount++;
        sleepMilliseconds(10);
    }

    std::cout << "[Sensor Thread] Stopped" << std::endl;
}

/**
 * @brief Helper function to write CSV row
 */
void writeCSVRow(std::ofstream& outFile, double elapsedSeconds,
    const Transform& t1, const Transform& t2,
    bool v1, bool v2,
    double delta_x, double delta_y, double delta_z,
    double y_linear, double angle_servo1, double angle_servo2, double angle_servo3)
{
    outFile << std::fixed << std::setprecision(3) << elapsedSeconds << ",";
    outFile << g_frameCount.load() << ",";

    outFile << std::setprecision(6);
    if (v1)
    {
        double x1, y1, z1;
        transformCoordinates(t1, x1, y1, z1);
        outFile << x1 << "," << y1 << "," << z1 << ",";
        outFile << t1.q0 << "," << t1.qx << "," << t1.qy << "," << t1.qz << ",";
        outFile << "0" << ",";
    }
    else
    {
        outFile << "NaN,NaN,NaN,NaN,NaN,NaN,NaN,1,";
    }
    outFile << "0x" << std::hex << t1.getErrorCode() << std::dec << ",";

    if (v2)
    {
        double x2, y2, z2;
        transformCoordinates(t2, x2, y2, z2);
        outFile << x2 << "," << y2 << "," << z2 << ",";
        outFile << t2.q0 << "," << t2.qx << "," << t2.qy << "," << t2.qz << ",";
        outFile << "0" << ",";
    }
    else
    {
        outFile << "NaN,NaN,NaN,NaN,NaN,NaN,NaN,1,";
    }
    outFile << "0x" << std::hex << t2.getErrorCode() << std::dec << ",";

    outFile << std::setprecision(2);
    outFile << delta_x << "," << delta_y << "," << delta_z << ",";
    outFile << y_linear << "," << angle_servo1 << "," << angle_servo2 << "," << angle_servo3 << std::endl;
}

/**
 * @brief Thread: Log sensor data to CSV
 */
void csvLoggingThread(std::ofstream& outFileLocal, std::ofstream& outFileE,
    long long startTime, bool edriveAvailable)
{
    //std::cout << "[CSV Thread] Started" << std::endl;

    while (g_running)
    {
        Transform t1, t2;
        bool v1, v2;
        long long timestamp;

        {
            std::lock_guard<std::mutex> lock(g_sensorData.mtx);
            t1 = g_sensorData.transform1;
            t2 = g_sensorData.transform2;
            v1 = g_sensorData.valid1;
            v2 = g_sensorData.valid2;
            timestamp = g_sensorData.timestamp;
        }

        double elapsedSeconds = (timestamp - startTime) / 1000.0;

        double delta_x = 0.0, delta_y = 0.0, delta_z = 0.0;
        double y_linear = 0.0, angle_servo1 = 90.0, angle_servo2 = 90.0, angle_servo3 = 0.0;

        {
            std::lock_guard<std::mutex> lock(g_controlData.mtx);
            y_linear = g_controlData.current_y_linear;
            angle_servo1 = g_controlData.current_servo_angle;
            angle_servo2 = g_controlData.current_servo2_angle;
            angle_servo3 = g_controlData.current_servo3_angle;

            if (g_controlData.target_active && v2)
            {
                double x_current, y_current, z_current;
                transformCoordinates(t2, x_current, y_current, z_current);

                delta_x = g_controlData.x_ref - x_current;
                delta_y = g_controlData.y_ref - y_current;
                delta_z = g_controlData.z_ref - z_current;
            }
        }

        writeCSVRow(outFileLocal, elapsedSeconds, t1, t2, v1, v2,
            delta_x, delta_y, delta_z, y_linear, angle_servo1, angle_servo2, angle_servo3);
        outFileLocal.flush();

        if (edriveAvailable && outFileE.is_open())
        {
            writeCSVRow(outFileE, elapsedSeconds, t1, t2, v1, v2,
                delta_x, delta_y, delta_z, y_linear, angle_servo1, angle_servo2, angle_servo3);
            outFileE.flush();
        }

        sleepMilliseconds(CSV_LOG_INTERVAL_MS);
    }

    std::cout << "[CSV Thread] Stopped" << std::endl;
}

/**
 * @brief Thread: Control loop using inverse kinematics with error feedback
 *
 * Control Strategy:
 * 1. theta2 = FIXED (from IK, never changes)
 * 2. theta3 -> controls Z (adjust until delta_z ≈ 0)
 * 3. theta1 -> controls X (adjust until delta_x ≈ 0)
 * 4. y_linear -> controls Y (adjust until delta_y ≈ 0)
 */
void controlThread()
{
    //std::cout << "[Control Thread] Started (Sequential Mode: Z -> X -> Y)" << std::endl;

    const double Kp_theta3_z = 0.5;
    const double Kp_theta1_x = -0.3;
    const double Kp_ylin_y = 1;

    const double max_angle_adjust = 3.0;
    const double max_ylin_adjust = 2.0;

    // BUGFIX: Check both g_running AND g_trackingModeRunning
    while (g_running && g_trackingModeRunning)
    {
        bool should_send = false;
        double delta_x = 0.0, delta_y = 0.0, delta_z = 0.0;
        float y_linear = 0.0;
        float angle_servo1 = 90.0;
        float angle_servo2 = 90.0;
        float angle_servo3 = 0.0;

        {
            std::lock_guard<std::mutex> lock(g_controlData.mtx);

            if (g_controlData.target_active && !g_controlData.target_reached)
            {
                // Step 1: Compute IK ONCE for this target
                if (!g_controlData.ik_computed)
                {
                    double theta1, theta2, theta3, y_lin;
                    bool ik_success = solveInverseKinematics(
                        g_controlData.x_ref, g_controlData.y_ref, g_controlData.z_ref,
                        g_controlData.x0_initial, g_controlData.y0_sensor1, g_controlData.z0_initial,
                        theta1, theta2, theta3, y_lin, false);

                    if (ik_success) {
                        g_controlData.cmd_theta1 = theta1;
                        g_controlData.cmd_theta2 = theta2;  // STAYS FIXED
                        g_controlData.cmd_theta3 = theta3;
                        g_controlData.cmd_y_linear = y_lin;
                        g_controlData.ik_computed = true;
                    }
                    else {
                        std::cerr << "[IK] Failed to find solution!" << std::endl;
                        g_controlData.target_active = false;
                        continue;
                    }
                }

                // Step 2: Get current position error
                Transform t2;
                bool v2;
                {
                    std::lock_guard<std::mutex> sensorLock(g_sensorData.mtx);
                    t2 = g_sensorData.transform2;
                    v2 = g_sensorData.valid2;
                }

                if (v2)
                {
                    double x_cur, y_cur, z_cur;
                    transformCoordinates(t2, x_cur, y_cur, z_cur);

                    delta_x = g_controlData.x_ref - x_cur;
                    delta_y = g_controlData.y_ref - y_cur;
                    delta_z = g_controlData.z_ref - z_cur;

                    // ============================================
                    // SEQUENTIAL CONTROL STRATEGY:
                    // ============================================

                    // A. Priority 1: Adjust Z via theta3
                    if (std::abs(delta_z) >= THRESHOLD)
                    {
                        double adjust = Kp_theta3_z * delta_z;
                        adjust = std::max(-max_angle_adjust, std::min(max_angle_adjust, adjust));
                        g_controlData.cmd_theta3 += adjust;
                        std::cout << " [SEQ] Adjusting Z (Theta3): d=" << delta_z << " adj=" << adjust << std::endl;
                        should_send = true;
                    }
                    // B. Priority 2: Adjust X via theta1 (Only if Z is reached)
                    else if (std::abs(delta_x) >= THRESHOLD)
                    {
                        double adjust = Kp_theta1_x * delta_x;
                        adjust = std::max(-max_angle_adjust, std::min(max_angle_adjust, adjust));
                        g_controlData.cmd_theta1 += adjust;
                        std::cout << " [SEQ] Adjusting X (Theta1): d=" << delta_x << " adj=" << adjust << std::endl;
                        should_send = true;
                    }
                    // C. Priority 3: Adjust Y via y_linear (Only if Z and X are reached)
                    else if (std::abs(delta_y) >= THRESHOLD)
                    {
                        double adjust = Kp_ylin_y * delta_y;
                        adjust = std::max(-max_ylin_adjust, std::min(max_ylin_adjust, adjust));
                        g_controlData.cmd_y_linear += adjust;
                        std::cout << " [SEQ] Adjusting Y (Y-Lin): d=" << delta_y << " adj=" << adjust << std::endl;
                        should_send = true;
                    }
                    // D. All reached
                    else
                    {
                        g_controlData.target_reached = true;
                        std::cout << "\n*** SEQUENTIAL TARGET REACHED ***" << std::endl;
                        continue;
                    }

                    if (should_send) {
                        // Clamp commands to valid ranges
                        g_controlData.cmd_theta1 = std::max(SERVO_ANGLE_MIN, std::min(SERVO_ANGLE_MAX, g_controlData.cmd_theta1));
                        g_controlData.cmd_theta3 = std::max(SERVO3_ANGLE_MIN, std::min(SERVO3_ANGLE_MAX, g_controlData.cmd_theta3));
                        g_controlData.cmd_y_linear = std::max(Y_LINEAR_MIN, std::min(Y_LINEAR_MAX, g_controlData.cmd_y_linear));

                        // Prepare floats for transmission
                        angle_servo1 = static_cast<float>(g_controlData.cmd_theta1);
                        angle_servo2 = static_cast<float>(g_controlData.cmd_theta2); // STAYS FIXED
                        angle_servo3 = static_cast<float>(g_controlData.cmd_theta3);
                        y_linear = static_cast<float>(g_controlData.cmd_y_linear);

                        // Sync current status
                        g_controlData.current_servo_angle = g_controlData.cmd_theta1;
                        g_controlData.current_servo2_angle = g_controlData.cmd_theta2;
                        g_controlData.current_servo3_angle = g_controlData.cmd_theta3;
                        g_controlData.current_y_linear = g_controlData.cmd_y_linear;
                    }
                }
            }
        }

        if (should_send)
        {
            g_pausePositionStream = true;
            g_stm32CommandsSent++;

            bool success = sendAnglesToSTM32AndWait(y_linear, angle_servo1, angle_servo2, angle_servo3, &g_stm32_current_y_linear);

            if (success) g_stm32CommandsCompleted++;
            g_pausePositionStream = false;
        }

        sleepMilliseconds(CONTROL_UPDATE_INTERVAL_MS);
    }
    std::cout << "[Control Thread] Stopped" << std::endl;
}

//============================================================================
// CALIBRATION FUNCTION
//============================================================================

/**
 * @brief Calibrate initial position from sensors
 */
bool calibrateInitialPosition()
{
    //std::cout << "\nCalibrating initial position..." << std::endl;

    bool initial_pos_set = false;

    for (int attempts = 0; attempts < 50 && !initial_pos_set; attempts++)
    {
        std::lock_guard<std::mutex> lock(g_sensorData.mtx);
        if (g_sensorData.valid2 && g_sensorData.valid1)
        {
            double x0, y0_s2, z0;
            transformCoordinates(g_sensorData.transform2, x0, y0_s2, z0);

            double x1_temp, y0_s1, z1_temp;
            transformCoordinates(g_sensorData.transform1, x1_temp, y0_s1, z1_temp);

            {
                std::lock_guard<std::mutex> ctrl_lock(g_controlData.mtx);
                g_controlData.x0_initial = x0;
                g_controlData.y0_initial = y0_s2;
                g_controlData.z0_initial = z0;
                g_controlData.y0_sensor1 = y0_s1;
                g_controlData.calibrated = true;
            }

            /*std::cout << "\n[OK] Initial Position Set!" << std::endl;
            std::cout << "  Sensor 2: X0=" << std::fixed << std::setprecision(2) << x0
                << ", Y0=" << y0_s2 << ", Z0=" << z0 << " mm" << std::endl;
            std::cout << "  Sensor 1: Y0=" << y0_s1 << " mm" << std::endl;*/

            initial_pos_set = true;
        }
        else
        {
            if (attempts % 5 == 0)
            {
                std::cout << "Attempt " << (attempts + 1) << "/50..." << std::endl;
            }
            sleepMilliseconds(200);
        }
    }

    return initial_pos_set;
}

//============================================================================
// MODE FUNCTIONS
//============================================================================

/**
 * @brief Moving Mode - Manual servo angle input with position streaming
 */
void runMovingMode()
{
    std::cout << "\n===========================================\n";
    std::cout << "  MOVING MODE\n";
    std::cout << "  Enter servo angles manually\n";
    std::cout << "  Position data streamed to OpenMV\n";
    std::cout << "===========================================\n";

    std::cout << "\nInput Limits:" << std::endl;
    std::cout << "  Linear (L):   " << Y_LINEAR_MIN << " - " << Y_LINEAR_MAX << " mm (position)" << std::endl;
    std::cout << "  Servo 1 (S1): " << SERVO_ANGLE_MIN << " - " << SERVO_ANGLE_MAX << " degrees" << std::endl;
    std::cout << "  Servo 2 (S2): " << SERVO2_ANGLE_MIN << " - " << SERVO2_ANGLE_MAX << " degrees" << std::endl;
    std::cout << "  Servo 3 (S3): " << SERVO3_ANGLE_MIN << " - " << SERVO3_ANGLE_MAX << " degrees" << std::endl;

    std::cout << "\n[INFO] Current y_linear position: " << g_moving_mode_y_linear << " mm" << std::endl;

    bool moving_loop = true;

    while (moving_loop && g_running)
    {
        // Display current sensor position and send to STM32/OpenMV
        {
            std::lock_guard<std::mutex> lock(g_sensorData.mtx);
            if (g_sensorData.valid2)
            {
                double x, y, z;
                transformCoordinates(g_sensorData.transform2, x, y, z);
                std::cout << "\n--- Current Sensor 2 Position ---" << std::endl;
                std::cout << "  X=" << std::fixed << std::setprecision(2) << x
                    << " mm, Y=" << y << " mm, Z=" << z << " mm" << std::endl;

                // Send position to STM32 for OpenMV display
                sendPositionToOpenMV(x, y, z, true);
            }
            else
            {
                std::cout << "\n[WARNING] Sensor 2 data invalid" << std::endl;
            }
        }

        std::cout << "\n--- Enter Values ---" << std::endl;
        std::cout << "(Enter 'q' for any value to quit, 'p' to resend position)" << std::endl;
        std::cout << "[Current y_linear: " << g_moving_mode_y_linear << " mm]" << std::endl;

        float y_linear_target, angle_s1, angle_s2, angle_s3;
        std::string input;

        // Get Linear motor position in mm
        std::cout << "Linear position (" << Y_LINEAR_MIN << "-" << Y_LINEAR_MAX << " mm): ";
        std::cin >> input;
        if (input == "q" || input == "Q") { moving_loop = false; continue; }
        if (input == "p" || input == "P")
        {
            std::lock_guard<std::mutex> lock(g_sensorData.mtx);
            if (g_sensorData.valid2)
            {
                double x, y, z;
                transformCoordinates(g_sensorData.transform2, x, y, z);
                sendPositionToOpenMV(x, y, z, true);
            }
            continue;
        }
        y_linear_target = std::stof(input);

        // Validate y_linear
        if (y_linear_target < Y_LINEAR_MIN || y_linear_target > Y_LINEAR_MAX)
        {
            std::cout << "[WARNING] y_linear out of range, clamping..." << std::endl;
            y_linear_target = std::max((float)Y_LINEAR_MIN, std::min((float)Y_LINEAR_MAX, y_linear_target));
        }

        // Get Servo 1 angle
        std::cout << "Servo 1 angle (" << SERVO_ANGLE_MIN << "-" << SERVO_ANGLE_MAX << " deg): ";
        std::cin >> input;
        if (input == "q" || input == "Q") { moving_loop = false; continue; }
        angle_s1 = std::stof(input);

        // Validate S1
        if (angle_s1 < SERVO_ANGLE_MIN || angle_s1 > SERVO_ANGLE_MAX)
        {
            std::cout << "[WARNING] S1 out of range, clamping..." << std::endl;
            angle_s1 = std::max((float)SERVO_ANGLE_MIN, std::min((float)SERVO_ANGLE_MAX, angle_s1));
        }

        // Get Servo 2 angle
        std::cout << "Servo 2 angle (" << SERVO2_ANGLE_MIN << "-" << SERVO2_ANGLE_MAX << " deg): ";
        std::cin >> input;
        if (input == "q" || input == "Q") { moving_loop = false; continue; }
        angle_s2 = std::stof(input);

        // Validate S2
        if (angle_s2 < SERVO2_ANGLE_MIN || angle_s2 > SERVO2_ANGLE_MAX)
        {
            std::cout << "[WARNING] S2 out of range, clamping..." << std::endl;
            angle_s2 = std::max((float)SERVO2_ANGLE_MIN, std::min((float)SERVO2_ANGLE_MAX, angle_s2));
        }

        // Get Servo 3 angle
        std::cout << "Servo 3 angle (" << SERVO3_ANGLE_MIN << "-" << SERVO3_ANGLE_MAX << " deg): ";
        std::cin >> input;
        if (input == "q" || input == "Q") { moving_loop = false; continue; }
        angle_s3 = std::stof(input);

        // Validate S3
        if (angle_s3 < SERVO3_ANGLE_MIN || angle_s3 > SERVO3_ANGLE_MAX)
        {
            std::cout << "[WARNING] S3 out of range, clamping..." << std::endl;
            angle_s3 = std::max((float)SERVO3_ANGLE_MIN, std::min((float)SERVO3_ANGLE_MAX, angle_s3));
        }

        // Calculate theta_linear from delta y_linear
        float delta_y_linear = y_linear_target - g_moving_mode_y_linear;
        float theta_linear = -(delta_y_linear / REV_MM) * 360.0f;

        std::cout << "\n--- Calculated Values ---" << std::endl;
        std::cout << "  y_linear: " << g_moving_mode_y_linear << " mm -> " << y_linear_target << " mm" << std::endl;
        std::cout << "  delta_y_linear: " << delta_y_linear << " mm" << std::endl;
        std::cout << "  theta_linear: " << theta_linear << " degrees" << std::endl;

        // Update control data for CSV logging
        {
            std::lock_guard<std::mutex> lock(g_controlData.mtx);
            g_controlData.current_servo_angle = angle_s1;
            g_controlData.current_servo2_angle = angle_s2;
            g_controlData.current_servo3_angle = angle_s3;
            g_controlData.current_y_linear = y_linear_target;
        }

        // Send to STM32
        std::cout << "\nSending to STM32..." << std::endl;
        g_pausePositionStream = true;

        bool success = sendAnglesToSTM32NoWait(theta_linear, angle_s1, angle_s2, angle_s3);

        if (success)
        {
            std::cout << "[OK] Command sent successfully" << std::endl;
            std::cout << "Waiting for motor movement..." << std::flush;

            time_t start_time = time(NULL);
            char rx_buffer[256] = { 0 };
            int buffer_pos = 0;
            bool completed = false;

            while (!completed && difftime(time(NULL), start_time) < STM32_TIMEOUT)
            {
                DWORD bytes_read;
                char temp_buf[64];

                if (ReadFile(hSTM32Serial, temp_buf, sizeof(temp_buf) - 1, &bytes_read, NULL))
                {
                    if (bytes_read > 0)
                    {
                        temp_buf[bytes_read] = '\0';
                        for (DWORD i = 0; i < bytes_read && buffer_pos < 255; i++)
                        {
                            rx_buffer[buffer_pos++] = temp_buf[i];
                        }
                        rx_buffer[buffer_pos] = '\0';

                        if (strchr(rx_buffer, '1') != NULL)
                        {
                            completed = true;
                            std::cout << " Done!" << std::endl;
                            g_moving_mode_y_linear = y_linear_target;
                            std::cout << "[INFO] y_linear updated to: " << g_moving_mode_y_linear << " mm" << std::endl;
                        }
                    }
                }
                Sleep(10);
            }

            if (!completed)
            {
                std::cout << " Timeout!" << std::endl;
                std::cout << "[WARNING] y_linear NOT updated (movement may have failed)" << std::endl;
            }
        }
        else
        {
            std::cout << "[ERROR] Failed to send command" << std::endl;
        }

        g_pausePositionStream = false;

        // Show updated position after movement and send to OpenMV
        sleepMilliseconds(500);
        {
            std::lock_guard<std::mutex> lock(g_sensorData.mtx);
            if (g_sensorData.valid2)
            {
                double x, y, z;
                transformCoordinates(g_sensorData.transform2, x, y, z);
                std::cout << "\n--- New Sensor 2 Position ---" << std::endl;
                std::cout << "  X=" << std::fixed << std::setprecision(2) << x
                    << " mm, Y=" << y << " mm, Z=" << z << " mm" << std::endl;

                sendPositionToOpenMV(x, y, z, true);
            }
        }

        std::cout << "\nContinue moving? (y/n): ";
        char response;
        std::cin >> response;
        if (response != 'y' && response != 'Y')
        {
            moving_loop = false;
        }
    }

    std::cout << "\nExiting Moving Mode..." << std::endl;
    std::cout << "[INFO] Final y_linear position: " << g_moving_mode_y_linear << " mm" << std::endl;
}

/**
 * @brief Tracking Mode - Original IK-based tracking functionality
 */
//void runTrackingMode()
//{
//    std::cout << "\n===========================================\n";
//    std::cout << "  TRACKING MODE\n";
//    std::cout << "  IK-based position tracking\n";
//    std::cout << "===========================================\n";
//
//    // Calibrate initial position
//    if (!calibrateInitialPosition())
//    {
//        std::cerr << "\nERROR: Could not calibrate initial position!" << std::endl;
//        return;
//    }
//
//    // Analyze workspace
//    double x0, y0_sensor1, z0;
//    {
//        std::lock_guard<std::mutex> lock(g_controlData.mtx);
//        x0 = g_controlData.x0_initial;
//        y0_sensor1 = g_controlData.y0_sensor1;
//        z0 = g_controlData.z0_initial;
//    }
//
//    analyzeWorkspace(x0, y0_sensor1, z0,
//        g_workspace_x_min, g_workspace_x_max,
//        g_workspace_y_min, g_workspace_y_max,
//        g_workspace_z_min, g_workspace_z_max);
//
//    // BUGFIX: Set flag before starting control thread
//    g_trackingModeRunning = true;
//
//    // Start control thread for tracking
//    std::thread ctrlThread(controlThread);
//
//    bool tracking_loop = true;
//
//    while (tracking_loop && g_running)
//    {
//        // Get current position
//        std::cout << "\n--- Current Position ---" << std::endl;
//        bool valid = false;
//        double x_current = 0.0, y_current = 0.0, z_current = 0.0;
//
//        for (int attempts = 0; attempts < 50 && !valid; attempts++)
//        {
//            {
//                std::lock_guard<std::mutex> lock(g_sensorData.mtx);
//                if (g_sensorData.valid2)
//                {
//                    transformCoordinates(g_sensorData.transform2, x_current, y_current, z_current);
//                    valid = true;
//                    std::cout << "Sensor 2: X=" << std::fixed << std::setprecision(2)
//                        << x_current << ", Y=" << y_current << ", Z=" << z_current << " mm" << std::endl;
//                }
//            }
//            if (!valid) sleepMilliseconds(200);
//        }
//
//        if (!valid)
//        {
//            std::cerr << "ERROR: Could not get valid position" << std::endl;
//            break;
//        }
//
//        int numPoints = 0;
//        while (numPoints <= 0)
//        {
//            std::cout << "\nHow many target points? (0 to exit): ";
//            std::cin >> numPoints;
//            if (numPoints == 0) { tracking_loop = false; break; }
//        }
//
//        if (!tracking_loop) break;
//
//        displayWorkspaceBounds(g_workspace_x_min, g_workspace_x_max,
//            g_workspace_y_min, g_workspace_y_max,
//            g_workspace_z_min, g_workspace_z_max);
//
//        int numPointsCollected = 0;
//        while (numPointsCollected < numPoints)
//        {
//            double x1, y1, z1;
//            bool input_valid = false;
//
//            while (!input_valid)
//            {
//                std::cout << "\n--- Enter Point " << (numPointsCollected + 1)
//                    << " of " << numPoints << " ---" << std::endl;
//                std::cout << "Enter X (mm): ";
//                std::cin >> x1;
//                std::cout << "Enter Y (mm): ";
//                std::cin >> y1;
//                std::cout << "Enter Z (mm): ";
//                std::cin >> z1;
//
//                input_valid = validateTargetPosition(x1, y1, z1, x0, y0_sensor1, z0);
//
//                if (!input_valid)
//                    std::cout << "Position not reachable. Please try again." << std::endl;
//            }
//
//            wayPoint tempStack = { x1, y1, z1 };
//            currentBatch.push_back(tempStack);
//            allPoints.push_back(tempStack);
//            numPointsCollected++;
//        }
//
//        // Execute movement to each target
//        for (size_t i = 0; i < currentBatch.size(); i++)
//        {
//            {
//                std::lock_guard<std::mutex> lock(g_controlData.mtx);
//                g_controlData.x_ref = currentBatch[i].x;
//                g_controlData.y_ref = currentBatch[i].y;
//                g_controlData.z_ref = currentBatch[i].z;
//                g_controlData.target_active = true;
//                g_controlData.target_reached = false;
//                g_controlData.ik_computed = false;  // Reset so IK computes new initial angles
//            }
//
//            std::cout << "\n--- Moving to Target " << (i + 1) << " ---" << std::endl;
//            std::cout << "Target: X=" << currentBatch[i].x
//                << ", Y=" << currentBatch[i].y
//                << ", Z=" << currentBatch[i].z << std::endl;
//
//            bool reached = false;
//            while (!reached && g_running && g_trackingModeRunning)
//            {
//                {
//                    std::lock_guard<std::mutex> lock(g_controlData.mtx);
//                    reached = g_controlData.target_reached;
//                }
//                sleepMilliseconds(100);
//            }
//
//            {
//                std::lock_guard<std::mutex> lock(g_controlData.mtx);
//                g_controlData.target_active = false;
//            }
//        }
//
//        currentBatch.clear();
//
//        std::cout << "\nContinue tracking? (y/n): ";
//        char response;
//        std::cin >> response;
//
//        if (response != 'y' && response != 'Y')
//        {
//            tracking_loop = false;
//        }
//    }
//
//    // BUGFIX: Signal control thread to stop
//    g_trackingModeRunning = false;
//
//    // Stop control thread
//    {
//        std::lock_guard<std::mutex> lock(g_controlData.mtx);
//        g_controlData.target_active = false;
//    }
//
//    // BUGFIX: Wait for control thread to finish
//    ctrlThread.join();
//
//    std::cout << "\nExiting Tracking Mode..." << std::endl;
//}
/**
 * @brief Tracking Mode - Original IK-based tracking functionality
 * UPDATED: Now asks for endpoint and generates intermediate points via linear interpolation
 */
void runTrackingMode()
{
    std::cout << "\n===========================================\n";
    std::cout << "  TRACKING MODE\n";
    std::cout << "  IK-based position tracking\n";
    std::cout << "  With Linear Interpolation\n";
    std::cout << "===========================================\n";

    // Calibrate initial position
    if (!calibrateInitialPosition())
    {
        std::cerr << "\nERROR: Could not calibrate initial position!" << std::endl;
        return;
    }

    // Analyze workspace
    double x0, y0_sensor1, z0;
    {
        std::lock_guard<std::mutex> lock(g_controlData.mtx);
        x0 = g_controlData.x0_initial;
        y0_sensor1 = g_controlData.y0_sensor1;
        z0 = g_controlData.z0_initial;
    }

    analyzeWorkspace(x0, y0_sensor1, z0,
        g_workspace_x_min, g_workspace_x_max,
        g_workspace_y_min, g_workspace_y_max,
        g_workspace_z_min, g_workspace_z_max);

    // BUGFIX: Set flag before starting control thread
    g_trackingModeRunning = true;

    // Start control thread for tracking
    std::thread ctrlThread(controlThread);

    bool tracking_loop = true;

    while (tracking_loop && g_running)
    {
        // Get current position (START POINT)
        std::cout << "\n--- Current Position (Start Point) ---" << std::endl;
        bool valid = false;
        double x_start = 0.0, y_start = 0.0, z_start = 0.0;

        for (int attempts = 0; attempts < 50 && !valid; attempts++)
        {
            {
                std::lock_guard<std::mutex> lock(g_sensorData.mtx);
                if (g_sensorData.valid2)
                {
                    transformCoordinates(g_sensorData.transform2, x_start, y_start, z_start);
                    valid = true;
                    std::cout << "Start: X=" << std::fixed << std::setprecision(2)
                        << x_start << ", Y=" << y_start << ", Z=" << z_start << " mm" << std::endl;
                }
            }
            if (!valid) sleepMilliseconds(200);
        }

        if (!valid)
        {
            std::cerr << "ERROR: Could not get valid position" << std::endl;
            break;
        }

        // Display workspace bounds
        displayWorkspaceBounds(g_workspace_x_min, g_workspace_x_max,
            g_workspace_y_min, g_workspace_y_max,
            g_workspace_z_min, g_workspace_z_max);

        // Ask for END POINT
        double x_end, y_end, z_end;
        bool endpoint_valid = false;

        while (!endpoint_valid)
        {
            std::cout << "\n--- Enter End Point ---" << std::endl;
            std::cout << "(Enter 0 0 0 to exit tracking mode)" << std::endl;
            std::cout << "Enter X (mm): ";
            std::cin >> x_end;
            std::cout << "Enter Y (mm): ";
            std::cin >> y_end;
            std::cout << "Enter Z (mm): ";
            std::cin >> z_end;

            // Check for exit condition
            if (x_end == 0 && y_end == 0 && z_end == 0)
            {
                tracking_loop = false;
                break;
            }

            // Validate endpoint
            endpoint_valid = validateTargetPosition(x_end, y_end, z_end, x0, y0_sensor1, z0);

            if (!endpoint_valid)
            {
                std::cout << "[ERROR] End position not reachable. Please try again." << std::endl;
            }
        }

        if (!tracking_loop) break;

        // Ask for number of intermediate points
        int numIntermediatePoints = 0;
        std::cout << "\nHow many intermediate points? (0 for direct movement): ";
        std::cin >> numIntermediatePoints;

        if (numIntermediatePoints < 0) numIntermediatePoints = 0;

        // Clear previous batch
        currentBatch.clear();

        // Generate intermediate points using linear interpolation
        // Total segments = numIntermediatePoints + 1
        // We generate numIntermediatePoints points BETWEEN start and end
        int totalSegments = numIntermediatePoints + 1;

        std::cout << "\n--- Generated Waypoints ---" << std::endl;
        std::cout << "Start:  X=" << std::fixed << std::setprecision(2)
            << x_start << ", Y=" << y_start << ", Z=" << z_start << std::endl;

        for (int i = 1; i <= numIntermediatePoints; i++)
        {
            double t = static_cast<double>(i) / static_cast<double>(totalSegments);

            double x_interp = x_start + t * (x_end - x_start);
            double y_interp = y_start + t * (y_end - y_start);
            double z_interp = z_start + t * (z_end - z_start);

            // Validate intermediate point
            bool interp_valid = validateTargetPosition(x_interp, y_interp, z_interp, x0, y0_sensor1, z0);

            if (interp_valid)
            {
                wayPoint wp = { x_interp, y_interp, z_interp };
                currentBatch.push_back(wp);
                allPoints.push_back(wp);

                std::cout << "Point " << i << ": X=" << x_interp
                    << ", Y=" << y_interp << ", Z=" << z_interp << " [OK]" << std::endl;
            }
            else
            {
                std::cout << "Point " << i << ": X=" << x_interp
                    << ", Y=" << y_interp << ", Z=" << z_interp << " [SKIPPED - unreachable]" << std::endl;
            }
        }

        // Add the end point
        wayPoint endWp = { x_end, y_end, z_end };
        currentBatch.push_back(endWp);
        allPoints.push_back(endWp);
        std::cout << "End:    X=" << x_end << ", Y=" << y_end << ", Z=" << z_end << " [OK]" << std::endl;

        std::cout << "\nTotal waypoints to execute: " << currentBatch.size() << std::endl;

        // Confirm before executing
        std::cout << "\nExecute movement? (y/n): ";
        char confirm;
        std::cin >> confirm;

        if (confirm != 'y' && confirm != 'Y')
        {
            std::cout << "Movement cancelled." << std::endl;
            currentBatch.clear();
            continue;
        }

        // Execute movement to each target
        for (size_t i = 0; i < currentBatch.size(); i++)
        {
            {
                std::lock_guard<std::mutex> lock(g_controlData.mtx);
                g_controlData.x_ref = currentBatch[i].x;
                g_controlData.y_ref = currentBatch[i].y;
                g_controlData.z_ref = currentBatch[i].z;
                g_controlData.target_active = true;
                g_controlData.target_reached = false;
                g_controlData.ik_computed = false;  // Reset so IK computes new initial angles
            }

            std::cout << "\n--- Moving to Waypoint " << (i + 1) << "/" << currentBatch.size() << " ---" << std::endl;
            std::cout << "Target: X=" << std::fixed << std::setprecision(2) << currentBatch[i].x
                << ", Y=" << currentBatch[i].y
                << ", Z=" << currentBatch[i].z << std::endl;

            bool reached = false;
            while (!reached && g_running && g_trackingModeRunning)
            {
                {
                    std::lock_guard<std::mutex> lock(g_controlData.mtx);
                    reached = g_controlData.target_reached;
                }
                sleepMilliseconds(100);
            }

            // Brief status update
            if (reached)
            {
                std::cout << "Waypoint " << (i + 1) << " reached!" << std::endl;
            }

            {
                std::lock_guard<std::mutex> lock(g_controlData.mtx);
                g_controlData.target_active = false;
            }
        }

        std::cout << "\n*** All waypoints completed! ***" << std::endl;
        currentBatch.clear();

        std::cout << "\nContinue tracking? (y/n): ";
        char response;
        std::cin >> response;

        if (response != 'y' && response != 'Y')
        {
            tracking_loop = false;
        }
    }

    // BUGFIX: Signal control thread to stop
    g_trackingModeRunning = false;

    // Stop control thread
    {
        std::lock_guard<std::mutex> lock(g_controlData.mtx);
        g_controlData.target_active = false;
    }

    // BUGFIX: Wait for control thread to finish
    ctrlThread.join();

    std::cout << "\nExiting Tracking Mode..." << std::endl;
}

//============================================================================
// MAIN
//============================================================================

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cout << "Aurora + STM32 Tracker (Menu Version) Ver " << capi.getVersion() << std::endl
            << "usage: ./combined_tracker <aurora_port>" << std::endl
            << "example: ./combined_tracker COM3" << std::endl
            << "\nNote: STM32 port configured as: " << STM32_PORT << std::endl;
        return -1;
    }

    std::string hostname = std::string(argv[1]);

    // ========================================
    // CONNECT TO AURORA
    // ========================================
   /* std::cout << "\n===========================================\n";
    std::cout << "  AURORA + STM32 TRACKER\n";
    std::cout << "  Menu Version with Tracking & Moving Modes\n";
    std::cout << "===========================================\n" << std::endl;

    std::cout << "Connecting to Aurora on " << hostname << "..." << std::endl*/;

    if (capi.connect(hostname) != 0)
    {
        std::cout << "ERROR: Aurora connection failed!" << std::endl;
        return -1;
    }
    /* std::cout << "Aurora connected" << std::endl;*/

    std::string apiRev = capi.getApiRevision();
    determineApiSupportForBX2();
    /*std::cout << "API Revision: " << apiRev << std::endl;*/

    if (capi.initialize() < 0)
    {
        std::cout << "ERROR: Aurora initialization failed!" << std::endl;
        return -1;
    }
    /*std::cout << "Aurora initialized" << std::endl;*/

    // ========================================
    // CONNECT TO STM32
    // ========================================
    /*std::cout << "\nConnecting to STM32 on " << STM32_PORT << "..." << std::endl;*/
    hSTM32Serial = openSTM32Serial(STM32_PORT);

    if (hSTM32Serial == INVALID_HANDLE_VALUE)
    {
        /* std::cout << "ERROR: STM32 connection failed!" << std::endl;*/
        return -1;
    }

    /* std::cout << "STM32 connected" << std::endl;*/
    Sleep(2000);

    PurgeComm(hSTM32Serial, PURGE_RXCLEAR | PURGE_TXCLEAR);
    /*std::cout << "STM32 ready" << std::endl;*/

    // ========================================
    // INITIALIZE AURORA SENSORS
    // ========================================
    std::vector<PortHandleInfo> allPorts = capi.portHandleSearchRequest(
        PortHandleSearchRequestOption::All);
    /*std::cout << "\nFound " << allPorts.size() << " Aurora sensor(s)" << std::endl;*/

    if (allPorts.size() < 2)
    {
        std::cout << "ERROR: Need at least 2 sensors, found only " << allPorts.size() << std::endl;
        CloseHandle(hSTM32Serial);
        return -1;
    }

    /* std::cout << "\nInitializing ports..." << std::endl;*/
    int successCount = 0;
    for (size_t i = 0; i < allPorts.size(); i++)
    {
        if (capi.portHandleInitialize(allPorts[i].getPortHandle()) >= 0 &&
            capi.portHandleEnable(allPorts[i].getPortHandle()) >= 0)
        {
            /*std::cout << "  Port " << allPorts[i].getPortHandle() << " ready" << std::endl;*/
            successCount++;
        }
    }

    if (successCount < 2)
    {
        std::cout << "ERROR: Could not initialize at least 2 ports!" << std::endl;
        CloseHandle(hSTM32Serial);
        return -1;
    }

    /* std::cout << "\nStarting tracking..." << std::endl;*/
    if (capi.startTracking() < 0)
    {
        std::cout << "ERROR: Could not start tracking!" << std::endl;
        CloseHandle(hSTM32Serial);
        return -1;
    }
    /*std::cout << "Tracking started" << std::endl;*/

    // ========================================
    // SETUP DATA LOGGING
    // ========================================
    std::remove("tracking_data.csv");
    std::remove("E:\\tracking_data.csv");

    std::ofstream outFileLocal("tracking_data.csv");
    if (!outFileLocal.is_open())
    {
        std::cerr << "ERROR: Could not open tracking_data.csv" << std::endl;
        CloseHandle(hSTM32Serial);
        return -1;
    }

    std::ofstream outFileE("E:\\tracking_data.csv");
    bool edriveAvailable = outFileE.is_open();

    std::string csvHeader = "Timestamp,Frame,"
        "X1,Y1,Z1,Q01,QX1,QY1,QZ1,IsMissing1,ErrorCode1,"
        "X2,Y2,Z2,Q02,QX2,QY2,QZ2,IsMissing2,ErrorCode2,"
        "DeltaX,DeltaY,DeltaZ,Y_Linear,AngleServo1,AngleServo2,AngleServo3";

    outFileLocal << csvHeader << std::endl;
    if (edriveAvailable) outFileE << csvHeader << std::endl;

    long long startTime = getCurrentTimeMs();

    // Initialize control data
    {
        std::lock_guard<std::mutex> lock(g_controlData.mtx);
        g_controlData.target_active = false;
        g_controlData.target_reached = false;
        g_controlData.calibrated = false;
        g_controlData.ik_computed = false;
        g_controlData.x0_initial = 0.0;
        g_controlData.y0_initial = 0.0;
        g_controlData.z0_initial = 0.0;
        g_controlData.y0_sensor1 = 0.0;
        g_controlData.current_servo_angle = 90.0;
        g_controlData.current_servo2_angle = 90.0;
        g_controlData.current_servo3_angle = 0.0;
        g_controlData.current_y_linear = 0.0;
        g_controlData.cmd_theta1 = 90.0;
        g_controlData.cmd_theta2 = 90.0;
        g_controlData.cmd_theta3 = 0.0;
        g_controlData.cmd_y_linear = 0.0;
    }

    // ========================================
    // START BACKGROUND THREADS
    // ========================================
    g_running = true;

    std::thread sensorThread(sensorReadingThread);
    std::thread csvThread(csvLoggingThread, std::ref(outFileLocal),
        std::ref(outFileE), startTime, edriveAvailable);
    std::thread posStreamThread(positionStreamingThread);

    //std::cout << "\nBackground threads started. Waiting for sensor data..." << std::endl;
    sleepMilliseconds(500);

    // ========================================
    // MAIN MENU LOOP
    // ========================================
    bool main_loop = true;

    while (main_loop)
    {
        std::cout << "\n===========================================\n";
        std::cout << "              MAIN MENU\n";
        std::cout << "===========================================\n";
        std::cout << "  1. Tracking Mode (IK-based position control)\n";
        std::cout << "  2. Moving Mode (Manual servo angle input)\n";
        std::cout << "  3. Home Position (Reset all servos)\n";
        std::cout << "  4. Exit\n";
        std::cout << "===========================================\n";
        std::cout << "Enter choice: ";

        int choice;
        std::cin >> choice;

        switch (choice)
        {
        case 1:
            runTrackingMode();
            break;

        case 2:
            runMovingMode();
            break;

        case 3:
            std::cout << "\nResetting to home position..." << std::endl;
            g_pausePositionStream = true;
            {
                float current_y_lin = 0.0f;
                {
                    std::lock_guard<std::mutex> lock(g_controlData.mtx);
                    current_y_lin = static_cast<float>(g_controlData.current_y_linear);
                }

                float theta_linear = -(0.0f - current_y_lin) * 360.0f / REV_MM;

                std::cout << "  Current y_linear: " << current_y_lin << " mm" << std::endl;
                std::cout << "  theta_linear to send: " << theta_linear << " degrees" << std::endl;

                bool success = sendAnglesToSTM32NoWait(theta_linear, 90.0f, 90.0f, 0.0f);

                if (success)
                {
                    std::cout << "  Waiting for motor movement..." << std::flush;

                    time_t start_time = time(NULL);
                    char rx_buffer[256] = { 0 };
                    int buffer_pos = 0;
                    bool completed = false;

                    while (!completed && difftime(time(NULL), start_time) < STM32_TIMEOUT)
                    {
                        DWORD bytes_read;
                        char temp_buf[64];

                        if (ReadFile(hSTM32Serial, temp_buf, sizeof(temp_buf) - 1, &bytes_read, NULL))
                        {
                            if (bytes_read > 0)
                            {
                                temp_buf[bytes_read] = '\0';
                                for (DWORD i = 0; i < bytes_read && buffer_pos < 255; i++)
                                {
                                    rx_buffer[buffer_pos++] = temp_buf[i];
                                }
                                rx_buffer[buffer_pos] = '\0';

                                if (strchr(rx_buffer, '1') != NULL)
                                {
                                    completed = true;
                                    std::cout << " Done!" << std::endl;
                                }
                            }
                        }
                        Sleep(10);
                    }

                    if (completed)
                    {
                        std::lock_guard<std::mutex> lock(g_controlData.mtx);
                        g_controlData.current_servo_angle = 90.0;
                        g_controlData.current_servo2_angle = 90.0;
                        g_controlData.current_servo3_angle = 0.0;
                        g_controlData.current_y_linear = 0.0;
                        g_stm32_current_y_linear = 0.0f;
                        g_moving_mode_y_linear = 0.0f;
                        std::cout << "Home position reached." << std::endl;
                    }
                    else
                    {
                        std::cout << " Timeout!" << std::endl;
                        std::cout << "Failed to reach home position." << std::endl;
                    }
                }
                else
                {
                    std::cout << "Failed to send home command." << std::endl;
                }
            }
            g_pausePositionStream = false;
            break;

        case 4:
            main_loop = false;
            break;

        default:
            std::cout << "Invalid choice. Please try again." << std::endl;
            break;
        }
    }

    // ========================================
    // CLEANUP
    // ========================================
    std::cout << "\n===========================================\n";
    std::cout << "  SHUTTING DOWN\n";
    std::cout << "===========================================\n";

    g_running = false;

    sensorThread.join();
    csvThread.join();
    posStreamThread.join();
    std::cout << "All threads stopped" << std::endl;

    std::cout << "\n--- Statistics ---" << std::endl;
    std::cout << "Total Frames: " << g_frameCount.load() << std::endl;
    std::cout << "Valid Frames: " << g_validFrames.load() << std::endl;
    std::cout << "Missing Frames: " << g_missingFrames.load() << std::endl;
    std::cout << "Invalid Frames: " << g_invalidFrames.load() << std::endl;
    std::cout << "STM32 Commands Sent: " << g_stm32CommandsSent.load() << std::endl;
    std::cout << "STM32 Commands Completed: " << g_stm32CommandsCompleted.load() << std::endl;

    outFileLocal.close();
    if (edriveAvailable) outFileE.close();

    capi.stopTracking();
    CloseHandle(hSTM32Serial);

    std::cout << "\nDone!" << std::endl;

    return 0;
}
