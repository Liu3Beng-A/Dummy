#ifndef DOF6_KINEMATIC_SOLVER_H
#define DOF6_KINEMATIC_SOLVER_H

// No MCU state: this module is also compiled directly by the native oracle.
class DOF6Kinematic
{
private:
    struct ArmConfig_t { float L_BASE, D_BASE, L_ARM, L_FOREARM, D_ELBOW, L_WRIST; };
    ArmConfig_t armConfig;
    float l_se_2{}, l_se{}, l_ew_2{}, l_ew{}, atan_e{};
    bool geometryValid{};

public:
    struct Joint6D_t
    {
        // Public joints, seed and IK results are degrees, including multiple turns.
        Joint6D_t() = default;
        Joint6D_t(float a1, float a2, float a3, float a4, float a5, float a6)
            : a{a1, a2, a3, a4, a5, a6} {}
        float a[6]{};
        friend Joint6D_t operator-(const Joint6D_t&, const Joint6D_t&);
    };
    struct Pose6D_t
    {
        Pose6D_t() = default;
        Pose6D_t(float x, float y, float z, float a, float b, float c)
            : X(x), Y(y), Z(z), A(a), B(b), C(c), hasR(false) {}
        // FK/SolveIKMeters: XYZ metres. Legacy SolveIK: XYZ millimetres.
        // ABC always degrees, roll/pitch/yaw; R row-major supersedes ABC if hasR.
        float X{}, Y{}, Z{}, A{}, B{}, C{}, R[9]{};
        bool hasR{};
    };
    enum SolutionFlag { INVALID = 0, REGULAR = 1, SINGULAR = 2 };
    struct IKSolves_t
    {
        Joint6D_t config[8]{};
        // Shoulder/elbow/wrist. A singular solution is valid only after FK residual.
        char solFlag[8][3]{};
    };
    // Numerical model tolerances, not assertions of mechanical/TCP accuracy.
    static constexpr float PositionToleranceM = 0.0001f;
    static constexpr float RotationElementTolerance = 0.001f;

    DOF6Kinematic(float L_BS, float D_BS, float L_AM, float L_FA, float D_EW, float L_WT);
    bool SolveFK(const Joint6D_t&, Pose6D_t&);
    bool SolveIK(const Pose6D_t& millimetres, const Joint6D_t& seedDegrees, IKSolves_t&);
    bool SolveIKMeters(const Pose6D_t& metres, const Joint6D_t& seedDegrees, IKSolves_t&);
    static bool IsSolutionValid(const IKSolves_t&, unsigned index);
    // Solver has no mechanical joint limits. Pick a legal 360-degree equivalent
    // before checking/connecting branches; never clamp an angle into the limits.
    static bool NearestInLimits(float angleDegrees, float seedDegrees,
                                float minimumDegrees, float maximumDegrees, float& result);
};
#endif
