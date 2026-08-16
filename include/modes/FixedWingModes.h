#pragma once

#include "modes/Mode.h"
#include "modes/VehicleContext.h"

namespace fc {

/**
 * FW mode implementations (legacy Mode_Fixedwing.h). The legacy FBWA path
 * used FW_CONTROL's own simplified stick-driven P+D controller (a SECOND,
 * redundant attitude-control implementation alongside the "real" one that
 * AUTO used via Attitude.h::stabilize()  --  see docs/attitude-lqr.md). This
 * refactor has exactly one attitude controller (AttitudeController, LQR),
 * used by both FBWA (stick-derived roll/pitch setpoints) and AUTO/GUIDED
 * (L1/TECS-derived setpoints)  --  no more parallel/redundant stack.
 *
 * Yaw in FBWA is fully automatic (AttitudeController's coordinated-turn
 * feedforward from the roll setpoint) rather than legacy's manual-stick +
 * auto-roll-mix blend  --  an intentional simplification once yaw is under
 * LQR rather than a hand-tuned rudder mix. See docs/modes.md.
 */

class ModeManual final : public ModeBase {
public:
    explicit ModeManual(VehicleContext& ctx) : ctx_(ctx) {}
    ModeId id() const override { return ModeId::Manual; }

protected:
    void _update() override;

private:
    VehicleContext& ctx_;
};

class ModeFbwa final : public ModeBase {
public:
    explicit ModeFbwa(VehicleContext& ctx) : ctx_(ctx) {}
    ModeId id() const override { return ModeId::Fbwa; }

protected:
    bool _enter() override;
    void _update() override;

private:
    static float mapStickToDeg(uint16_t channel_pwm, float max_deg);

    VehicleContext& ctx_;
};

class ModeAuto final : public ModeBase {
public:
    explicit ModeAuto(VehicleContext& ctx) : ctx_(ctx) {}
    ModeId id() const override { return ModeId::Auto; }

protected:
    bool _enter() override;
    void _update() override;
    void _exit() override;

private:
    VehicleContext& ctx_;
};

class ModeGuided final : public ModeBase {
public:
    explicit ModeGuided(VehicleContext& ctx) : ctx_(ctx) {}
    ModeId id() const override { return ModeId::Guided; }

protected:
    bool _enter() override;
    void _update() override;
    void _exit() override;

private:
    VehicleContext& ctx_;
};

}  // namespace fc
