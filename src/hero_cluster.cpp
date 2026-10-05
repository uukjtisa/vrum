#include "../include/hero_cluster.h"

#include "../include/engine_sim_application.h"
#include "../include/drive_controller.h"
#include "../include/ui_utilities.h"
#include "../include/units.h"

#include <cmath>
#include <iomanip>
#include <sstream>

HeroCluster::HeroCluster() { /* void */ }
HeroCluster::~HeroCluster() { /* void */ }

void HeroCluster::update(float dt) {
    m_time += dt;

    if (m_simulator != nullptr && m_simulator->getEngine() != nullptr) {
        const double rpm = units::toRpm(std::abs(m_simulator->getEngine()->getSpeed()));
        // Light smoothing so the digits are readable at idle without lagging a rev.
        m_displayRpm += (rpm - m_displayRpm) * (dt / (dt + 0.05));
    }

    UiElement::update(dt);
}

void HeroCluster::render() {
    drawFrame(m_bounds, 1.0f, m_app->getForegroundColor(), m_app->getBackgroundColor());

    if (m_simulator == nullptr || m_simulator->getEngine() == nullptr) {
        UiElement::render();
        return;
    }

    Engine *engine = m_simulator->getEngine();
    const double limit = units::toRpm(engine->getIgnitionModule()->getRevLimit());

    const Bounds inner = m_bounds.inset(10.0f);
    const Bounds lights = inner.verticalSplit(0.88f, 1.0f);
    const Bounds main = inner.verticalSplit(0.30f, 0.86f);
    const Bounds chips = inner.verticalSplit(0.14f, 0.28f);
    const Bounds temps = inner.verticalSplit(0.0f, 0.12f);

    drawShiftLights(lights, m_displayRpm, limit);

    // Left: rpm + speed. Right: gear.
    const Bounds left = main.horizontalSplit(0.0f, 0.58f);
    const Bounds right = main.horizontalSplit(0.6f, 1.0f);

    std::stringstream rpm;
    rpm << (int)(std::round(m_displayRpm / 10.0) * 10.0);
    const bool nearLimit = m_displayRpm > limit - 300.0;
    const bool flash = nearLimit && std::fmod(m_time, 0.16f) < 0.08f;
    m_app->getTextRenderer()->SetColor(ysColor::linearToSrgb(
        flash ? m_app->getRed() : m_app->getForegroundColor()));
    drawAlignedText(rpm.str(), left.verticalSplit(0.45f, 1.0f), 64.0f, Bounds::rm, Bounds::rm);

    m_app->getTextRenderer()->SetColor(ysColor::linearToSrgb(
        mix(m_app->getBackgroundColor(), m_app->getForegroundColor(), 0.6f)));
    drawAlignedText("RPM", left.verticalSplit(0.3f, 0.48f), 16.0f, Bounds::rm, Bounds::rm);

    Vehicle *vehicle = m_simulator->getVehicle();
    if (vehicle != nullptr) {
        std::stringstream speed;
        speed << (int)std::round(units::convert(std::abs(vehicle->getSpeed()),
                                                units::km / units::hour))
              << " KM/H";
        m_app->getTextRenderer()->SetColor(ysColor::linearToSrgb(m_app->getForegroundColor()));
        drawAlignedText(speed.str(), left.verticalSplit(0.0f, 0.28f), 24.0f, Bounds::rm, Bounds::rm);
    }

    drawGear(right);
    drawChips(chips);
    drawTemps(temps);

    m_app->getTextRenderer()->SetColor(ysColor::linearToSrgb(m_app->getForegroundColor()));
    UiElement::render();
}

void HeroCluster::drawShiftLights(const Bounds &bounds, double rpm, double limit) {
    // 15 segments over the top 40% of the rev range: green, then amber, then
    // red; the whole strip flashes blue at the limiter -- the F1 convention.
    constexpr int Segments = 15;
    const double start = 0.6 * limit;
    const double lit = (rpm - start) / (limit - 200.0 - start) * Segments;
    const bool shiftNow = rpm > limit - 200.0;
    const bool blink = shiftNow && std::fmod(m_time, 0.12f) < 0.06f;

    const float w = bounds.width() / Segments;
    for (int i = 0; i < Segments; ++i) {
        const Bounds seg(
            bounds.left() + i * w + 2.0f, bounds.left() + (i + 1) * w - 2.0f,
            bounds.bottom(), bounds.top());

        ysVector on = (i < 5) ? m_app->getGreen()
            : (i < 10) ? m_app->getYellow()
            : m_app->getRed();
        if (shiftNow) on = blink ? m_app->getBlue() : m_app->getBackgroundColor();

        const bool isLit = shiftNow || i < lit;
        drawBox(seg, isLit
            ? on
            : mix(m_app->getBackgroundColor(), m_app->getForegroundColor(), 0.08f));
    }
}

void HeroCluster::drawGear(const Bounds &bounds) {
    const DriveController::Status &s = m_app->getDrive().getStatus();

    // Paddle feedback: a border flash that fades over 0.35 s -- green for an
    // accepted shift (or a queued one), red for a refused or locked-out one.
    ysVector frame = mix(m_app->getBackgroundColor(), m_app->getForegroundColor(), 0.3f);
    if (s.paddleAge < 0.35) {
        const float a = (float)(1.0 - s.paddleAge / 0.35);
        frame = mix(frame, s.lastPaddleAccepted ? m_app->getGreen() : m_app->getRed(), a);
    }
    drawFrame(bounds, 3.0f, frame, m_app->getBackgroundColor());

    const Bounds body = bounds.verticalSplit(0.28f, 1.0f);
    const Bounds advisor = bounds.verticalSplit(0.0f, 0.26f);

    std::stringstream g;
    if (s.gear == -1) g << "N";
    else g << (s.gear + 1);

    const bool shifting = s.phase != DriveController::ShiftPhase::Idle;
    m_app->getTextRenderer()->SetColor(ysColor::linearToSrgb(
        shifting ? m_app->getYellow() : m_app->getForegroundColor()));
    drawCenteredText(g.str(), body, 96.0f, Bounds::center);

    // Queued downshift: a pulsing ghost of where the queue will land.
    if (s.queuedSteps > 0) {
        const float pulse = 0.5f + 0.5f * std::sin(m_time * 10.0f);
        m_app->getTextRenderer()->SetColor(ysColor::linearToSrgb(
            mix(m_app->getBackgroundColor(), m_app->getOrange(), 0.4f + 0.6f * pulse)));
        std::stringstream q;
        q << "v" << (s.queuedGear + 1) << " QUEUED";
        drawCenteredText(q.str(), body.verticalSplit(0.0f, 0.25f), 16.0f, Bounds::center);
    }

    std::stringstream a;
    ysVector color = mix(m_app->getBackgroundColor(), m_app->getForegroundColor(), 0.55f);
    if (s.gear == -1) a << "NEUTRAL";
    else if (!s.advisorAvailable) a << "LOWEST";
    else {
        a << "v" << s.gear << " " << std::fixed << std::setprecision(1)
          << (s.advisorRpm / 1000.0) << "k ";
        if (s.advisorSafe) {
            a << "SAFE";
            color = m_app->getGreen();
        }
        else {
            a << (s.tcuEnabled ? "TCU" : "OVER");
            color = m_app->getRed();
        }
    }
    m_app->getTextRenderer()->SetColor(ysColor::linearToSrgb(color));
    drawCenteredText(a.str(), advisor, 16.0f, Bounds::center);
}

void HeroCluster::drawChip(
    const Bounds &bounds, const std::string &label, bool on, const ysVector &onColor)
{
    const ysVector off = mix(m_app->getBackgroundColor(), m_app->getForegroundColor(), 0.12f);
    drawFrame(bounds, 1.0f, on ? onColor : off,
              on ? mix(m_app->getBackgroundColor(), onColor, 0.25f) : m_app->getBackgroundColor());
    m_app->getTextRenderer()->SetColor(ysColor::linearToSrgb(
        on ? onColor : mix(m_app->getBackgroundColor(), m_app->getForegroundColor(), 0.4f)));
    drawCenteredText(label, bounds, 14.0f, Bounds::center);
}

void HeroCluster::drawChips(const Bounds &bounds) {
    const DriveController &drive = m_app->getDrive();
    const DriveController::Status &s = drive.getStatus();

    Grid grid;
    grid.h_cells = 5;
    grid.v_cells = 1;

    drawChip(grid.get(bounds, 0, 0).inset(2.0f), "TCU", s.tcuEnabled, m_app->getGreen());

    std::string assist = "ASSIST";
    if (drive.assist.master) {
        assist += " ";
        if (drive.assist.queueDownshifts) assist += "Q";
        if (drive.assist.autoUpshift) assist += "U";
        if (drive.assist.autoDownshift) assist += "D";
    }
    drawChip(grid.get(bounds, 1, 0).inset(2.0f), assist, drive.assist.master, m_app->getBlue());

    const char *launch = "LAUNCH";
    bool launchOn = false;
    ysVector launchColor = m_app->getGreen();
    if (!s.launchAvailable) launch = "NO LC";
    else if (!drive.launchEnabled) launch = "LC OFF";
    else if (s.launch == DriveController::LaunchState::Armed) {
        launch = "LC ARMED";
        launchOn = std::fmod(m_time, 0.3f) < 0.2f;
        launchColor = m_app->getOrange();
    }
    else if (s.launch == DriveController::LaunchState::Launching) {
        launch = "LAUNCH!";
        launchOn = true;
        launchColor = m_app->getRed();
    }
    else launchOn = s.launch == DriveController::LaunchState::Ready;
    drawChip(grid.get(bounds, 2, 0).inset(2.0f), launch, launchOn, launchColor);

    const bool isrOn = s.upshiftCutAvailable && drive.assist.master && drive.assist.upshiftCut;
    drawChip(grid.get(bounds, 3, 0).inset(2.0f), s.sparkCut ? "CUT" : "ISR",
             isrOn || s.sparkCut, s.sparkCut ? m_app->getRed() : m_app->getPink());

    drawChip(grid.get(bounds, 4, 0).inset(2.0f), "LIMP",
             s.limp && std::fmod(m_time, 0.5f) < 0.35f, m_app->getRed());
}

void HeroCluster::drawTemps(const Bounds &bounds) {
    const DriveController::Status &s = m_app->getDrive().getStatus();
    if (!s.coolingModelled) return;

    auto tempColor = [&](double c, double hot) {
        if (c > hot) return m_app->getRed();
        if (c > hot - 12.0) return m_app->getOrange();
        return m_app->getForegroundColor();
    };

    const Bounds l = bounds.horizontalSplit(0.0f, 0.5f);
    const Bounds r = bounds.horizontalSplit(0.5f, 1.0f);

    std::stringstream h2o, oil;
    h2o << "H2O " << (int)std::round(s.coolantC) << "C";
    oil << "OIL " << (int)std::round(s.oilC) << "C";

    m_app->getTextRenderer()->SetColor(ysColor::linearToSrgb(tempColor(s.coolantC, s.overheatC)));
    drawCenteredText(h2o.str(), l, 16.0f, Bounds::center);
    m_app->getTextRenderer()->SetColor(ysColor::linearToSrgb(tempColor(s.oilC, s.overheatC + 15.0)));
    drawCenteredText(oil.str(), r, 16.0f, Bounds::center);
}
