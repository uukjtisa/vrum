#ifndef VRUM_HERO_CLUSTER_H
#define VRUM_HERO_CLUSTER_H

#include "ui_element.h"

class Simulator;

// VRUM: the driver's panel. Big digital tach with a shift-light strip, the gear
// (with the Shift Assist queue and paddle feedback), the Shift Advisor, road
// speed, status chips (TCU / ASSIST / LAUNCH / ISR / LIMP) and temperatures.
// Everything it shows comes from DriveController::Status, the same snapshot the
// Android cockpit reads.
class HeroCluster : public UiElement {
    public:
        HeroCluster();
        virtual ~HeroCluster();

        virtual void update(float dt);
        virtual void render();

        Simulator *m_simulator = nullptr;

    protected:
        void drawShiftLights(const Bounds &bounds, double rpm, double limit);
        void drawGear(const Bounds &bounds);
        void drawChips(const Bounds &bounds);
        void drawTemps(const Bounds &bounds);
        void drawChip(const Bounds &bounds, const std::string &label,
                      bool on, const ysVector &onColor);

        float m_time = 0.0f;
        double m_displayRpm = 0.0;
};

#endif /* VRUM_HERO_CLUSTER_H */
