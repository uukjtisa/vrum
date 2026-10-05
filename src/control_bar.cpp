#include "../include/control_bar.h"

#include "../include/engine_sim_application.h"
#include "../include/ui_utilities.h"

ControlBar::ControlBar() {
    m_checkMouse = false;
}

ControlBar::~ControlBar() {
    /* void */
}

UiButton *ControlBar::addButton(
    Action action,
    const std::string &text,
    const std::string &shortcut,
    bool momentary,
    const ysVector &accent,
    int row)
{
    UiButton *button = addElement<UiButton>(this);
    button->m_text = text;
    button->m_shortcut = shortcut;
    button->m_momentary = momentary;
    button->m_fontSize = 15.0f;
    button->m_accent = accent;
    button->m_hasAccent = true;

    m_buttons.push_back(button);
    m_actions.push_back(action);
    m_buttonRows.push_back(row);

    return button;
}

UiSlider *ControlBar::addSlider(
    int row,
    const std::string &label,
    double *value,
    double min,
    double max,
    const std::string &unit,
    const std::string &minText,
    const ysVector &accent)
{
    UiSlider *slider = addElement<UiSlider>(this);
    slider->m_label = label;
    slider->m_value = value;
    slider->m_min = min;
    slider->m_max = max;
    slider->m_unit = unit;
    slider->m_minText = minText;
    slider->m_accent = accent;
    slider->m_hasAccent = true;

    m_sliders.push_back(slider);
    m_sliderRows.push_back(row);

    return slider;
}

void ControlBar::initialize(EngineSimApplication *app) {
    UiElement::initialize(app);

    // Grouped by what they do: engine state, then sound, then drivetrain, then
    // which engine is loaded. Colour follows the same grouping rather than
    // being decorative -- green for "running", red for the destructive-sounding
    // ones, blue for capture.
    addButton(Action::Ignition,    "IGNITION",  "A",     false, m_app->getGreen());
    addButton(Action::Starter,     "STARTER",   "HOLD S", true, m_app->getYellow());
    addButton(Action::Dyno,        "DYNO",      "D",     false, m_app->getBlue());
    addButton(Action::Backfire,    "BACKFIRE",  "O",     false, m_app->getOrange());
    addButton(Action::RevMatch,    "REV MATCH", "J",     false, m_app->getGreen());
    addButton(Action::Outdoor,     "ACOUSTICS", ". CLOSE", false, m_app->getYellow());
    addButton(Action::Brake,       "BRAKE",     "HOLD B", true, m_app->getRed());
    addButton(Action::RecordAudio, "REC WAV",   "P",     false, m_app->getRed());
    addButton(Action::GearDown,    "GEAR -",    "DOWN",   true, m_app->getPink());
    addButton(Action::GearUp,      "GEAR +",    "UP",     true, m_app->getPink());
    addButton(Action::PrevEngine,  "< ENGINE",  "PGUP",   true, m_app->getBlue());
    addButton(Action::NextEngine,  "ENGINE >",  "PGDN",   true, m_app->getBlue());

    // Second row -- the drive electronics. TCU and launch control are the
    // car's; SHIFT ASSIST is the driver aid, and its four options only act
    // while ASSIST itself is on.
    addButton(Action::Tcu,            "TCU",          "over-rev guard", false, m_app->getGreen(), 1);
    addButton(Action::Launch,         "LAUNCH",       "F5",             false, m_app->getOrange(), 1);
    addButton(Action::Assist,         "SHIFT ASSIST", "F6",             false, m_app->getBlue(), 1);
    addButton(Action::AssistQueue,    "QUEUE DOWN",   "assist",         false, m_app->getBlue(), 1);
    addButton(Action::AssistAutoUp,   "AUTO UP",      "assist",         false, m_app->getBlue(), 1);
    addButton(Action::AssistAutoDown, "AUTO DOWN",    "assist",         false, m_app->getBlue(), 1);
    addButton(Action::AssistIsr,      "ISR CUT",      "assist",         false, m_app->getPink(), 1);

    // Row 0 -- ACOUSTICS. The output-space mix: how much of what you hear is
    // reflected rather than direct, how long those reflections last, how much
    // top end they lose, how loud the near-wall slap-backs are, and how much
    // air absorption sits over the whole thing. The ACOUSTICS button loads
    // presets into exactly these; moving any of them reads back as CUSTOM.
    const ysVector space = m_app->getYellow();
    addSlider(0, "WET MIX", m_app->getAcousticsMix(),
              0.0, 1.0, "", "dry", space);
    addSlider(0, "ROOM SIZE", m_app->getAcousticsRoom(),
              0.0, 1.0, "", "", space);
    addSlider(0, "DAMPING", m_app->getAcousticsDamping(),
              0.0, 1.0, "", "", space);
    addSlider(0, "EARLY REFL", m_app->getAcousticsEarly(),
              0.0, 1.0, "", "off", space);
    addSlider(0, "AIR LP", m_app->getAcousticsAir(),
              0.0, 16000.0, "Hz", "off", space);

    // Row 1 -- pedal feel. Press and release are separate on purpose: a slow
    // press is what "rolling onto the pedal" means, while a slow RELEASE just
    // makes the car feel stuck to the throttle. Release defaults to instant.
    addSlider(1, "THROTTLE PRESS", m_app->getThrottleRampTime(),
              0.0, 1.5, "s", "instant", m_app->getGreen());
    addSlider(1, "BRAKE PRESS", m_app->getBrakeRampTime(),
              0.0, 1.5, "s", "instant", m_app->getRed());
    addSlider(1, "PEDAL RELEASE", m_app->getPedalReleaseTime(),
              0.0, 1.0, "s", "instant (default)", m_app->getYellow());
}

void ControlBar::update(float dt) {
    const int count = (int)m_buttons.size();
    if (count == 0) return;

    // Buttons on top, then one row per slider group. The button row gets more
    // height than a slider row because it carries two lines of text.
    const float gap = 6.0f;
    const float rowGap = 5.0f;
    const int sliderCount = (int)m_sliders.size();

    const float sliderRowHeight = 34.0f;
    const float sliderBlock = (sliderCount > 0)
        ? SliderRowCount * (sliderRowHeight + rowGap)
        : 0.0f;
    const float buttonHeight = m_bounds.height() - sliderBlock;

    const float buttonBottom = m_bounds.top() - buttonHeight;
    const float x0 = m_bounds.left();

    // Button rows split the button block evenly; each row lays out its own
    // count across the full width.
    const float buttonRowHeight =
        (buttonHeight - rowGap * (ButtonRowCount - 1)) / ButtonRowCount;
    for (int row = 0; row < ButtonRowCount; ++row) {
        int inRow = 0;
        for (int i = 0; i < count; ++i) {
            if (m_buttonRows[i] == row) ++inRow;
        }
        if (inRow == 0) continue;

        const float width = (m_bounds.width() - gap * (inRow - 1)) / inRow;
        const float top = m_bounds.top() - row * (buttonRowHeight + rowGap);

        int slot = 0;
        for (int i = 0; i < count; ++i) {
            if (m_buttonRows[i] != row) continue;
            Bounds b;
            b.m0 = { x0 + slot * (width + gap), top - buttonRowHeight };
            b.m1 = { x0 + slot * (width + gap) + width, top };
            m_buttons[i]->m_bounds = b;
            ++slot;
        }
    }

    // Each row is laid out independently, so rows may hold different counts.
    for (int row = 0; row < SliderRowCount; ++row) {
        int inRow = 0;
        for (int i = 0; i < sliderCount; ++i) {
            if (m_sliderRows[i] == row) ++inRow;
        }

        if (inRow == 0) continue;

        const float rowTop = buttonBottom - row * (sliderRowHeight + rowGap) - rowGap;
        const float sliderWidth =
            (m_bounds.width() - gap * (inRow - 1)) / inRow;

        int slot = 0;
        for (int i = 0; i < sliderCount; ++i) {
            if (m_sliderRows[i] != row) continue;

            Bounds b;
            b.m0 = { x0 + slot * (sliderWidth + gap), rowTop - sliderRowHeight };
            b.m1 = { x0 + slot * (sliderWidth + gap) + sliderWidth, rowTop };
            m_sliders[i]->m_bounds = b;
            ++slot;
        }
    }

    // Toggles mirror the simulator rather than remembering their own state, so
    // pressing the keyboard shortcut lights the button too.
    for (int i = 0; i < count; ++i) {
        switch (m_actions[i]) {
        case Action::Ignition:
            m_buttons[i]->m_active = m_app->isIgnitionEnabled();
            break;
        case Action::Starter:
            m_buttons[i]->m_active = m_app->isStarterEnabled();
            break;
        case Action::Dyno:
            m_buttons[i]->m_active = m_app->isDynoEnabled();
            break;
        case Action::Backfire:
            m_buttons[i]->m_active = m_app->isBackfireEnabled();
            break;
        case Action::RevMatch:
            m_buttons[i]->m_active = m_app->isRevMatchEnabled();
            break;
        case Action::Outdoor:
            // More than two states, so the caption carries the current preset
            // (or CUSTOM) rather than the button just being lit or not.
            m_buttons[i]->m_active = m_app->isOutdoorPerspective();
            m_buttons[i]->m_shortcut =
                std::string(". ") + m_app->getPerspectiveName();
            break;
        case Action::RecordAudio:
            m_buttons[i]->m_active = m_app->isCapturingAudio();
            break;
        case Action::Brake:
            m_buttons[i]->m_active = m_app->isBraking();
            break;
        case Action::Tcu:
            m_buttons[i]->m_active = m_app->getDrive().isTcuEnabled();
            break;
        case Action::Launch:
            m_buttons[i]->m_active = m_app->getDrive().launchEnabled
                && m_app->getDrive().getStatus().launchAvailable;
            break;
        case Action::Assist:
            m_buttons[i]->m_active = m_app->getDrive().assist.master;
            break;
        case Action::AssistQueue:
            m_buttons[i]->m_active = m_app->getDrive().assist.master
                && m_app->getDrive().assist.queueDownshifts;
            break;
        case Action::AssistAutoUp:
            m_buttons[i]->m_active = m_app->getDrive().assist.master
                && m_app->getDrive().assist.autoUpshift;
            break;
        case Action::AssistAutoDown:
            m_buttons[i]->m_active = m_app->getDrive().assist.master
                && m_app->getDrive().assist.autoDownshift;
            break;
        case Action::AssistIsr:
            m_buttons[i]->m_active = m_app->getDrive().assist.master
                && m_app->getDrive().assist.upshiftCut;
            break;
        default:
            break;
        }
    }

    // STARTER and BRAKE are pedals: they act while the mouse is held, the way
    // S and B do, rather than latching on click.
    for (int i = 0; i < count; ++i) {
        if (m_actions[i] == Action::Starter) {
            m_app->setStarterEnabled(m_buttons[i]->isMouseHeld());
        }
        else if (m_actions[i] == Action::Brake) {
            m_app->setBrakeInput(m_buttons[i]->isMouseHeld());
        }
    }

    UiElement::update(dt);
}

void ControlBar::render() {
    UiElement::render();
}

void ControlBar::signal(UiElement *element, Event event) {
    if (event != Event::Clicked) return;

    for (size_t i = 0; i < m_buttons.size(); ++i) {
        if (m_buttons[i] != element) continue;

        switch (m_actions[i]) {
        case Action::Ignition:    m_app->toggleIgnition(); break;
        case Action::Dyno:        m_app->toggleDyno(); break;
        case Action::Backfire:    m_app->toggleBackfire(); break;
        case Action::RevMatch:    m_app->toggleRevMatch(); break;
        case Action::Outdoor:     m_app->togglePerspective(); break;
        case Action::RecordAudio: m_app->toggleAudioCapture(); break;
        case Action::GearDown:    m_app->shiftGear(-1); break;
        case Action::GearUp:      m_app->shiftGear(1); break;
        // Switching engines tears down and rebuilds the whole UI tree --
        // including this ControlBar and the UiButton whose update() is on the
        // stack right now, because signal() is called from inside the button's
        // own click detection. Calling cycleEngine() here is a use-after-free;
        // it crashed every time, while PageUp/PageDown did not, because the key
        // handler runs before m_uiManager.update() rather than inside it.
        // Queue it and let the app run it between frames.
        case Action::PrevEngine:  m_app->requestEngineCycle(-1); break;
        case Action::NextEngine:  m_app->requestEngineCycle(1); break;
        case Action::Tcu:         m_app->toggleTcu(); break;
        case Action::Launch:      m_app->toggleLaunchControl(); break;
        case Action::Assist:      m_app->toggleAssist(); break;
        case Action::AssistQueue:
            m_app->toggleAssistOption(&DriveController::Assist::queueDownshifts, "QUEUE DOWNSHIFTS");
            break;
        case Action::AssistAutoUp:
            m_app->toggleAssistOption(&DriveController::Assist::autoUpshift, "AUTO UPSHIFT");
            break;
        case Action::AssistAutoDown:
            m_app->toggleAssistOption(&DriveController::Assist::autoDownshift, "AUTO DOWNSHIFT");
            break;
        case Action::AssistIsr:
            m_app->toggleAssistOption(&DriveController::Assist::upshiftCut, "ISR UPSHIFT CUT");
            break;
        // Starter and brake are driven by mouse-hold in update(), not clicks.
        case Action::Starter:     break;
        case Action::Brake:       break;
        default: break;
        }

        return;
    }
}
