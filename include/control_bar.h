#ifndef ATG_ENGINE_SIM_CONTROL_BAR_H
#define ATG_ENGINE_SIM_CONTROL_BAR_H

#include "ui_element.h"
#include "ui_button.h"
#include "ui_slider.h"

#include <vector>

// VRUM: a clickable control strip along the bottom of the engine panel.
//
// Everything here was previously keyboard-only, discoverable solely through a
// wall of text printed across the bottom of the screen. The buttons drive the
// exact same actions, stay in sync with the simulator's real state (so they
// also work as indicators when the keys are used), and keep the shortcut
// printed underneath so the keyboard stays learnable.
class ControlBar : public UiElement {
    public:
        enum class Action {
            Ignition,
            Starter,
            Dyno,
            Backfire,
            RevMatch,
            Outdoor,
            Brake,
            RecordAudio,
            GearDown,
            GearUp,
            PrevEngine,
            NextEngine,
            Tcu,
            Launch,
            Assist,
            AssistQueue,
            AssistAutoUp,
            AssistAutoDown,
            AssistIsr,
            Count
        };

    public:
        ControlBar();
        virtual ~ControlBar();

        virtual void initialize(EngineSimApplication *app);
        virtual void update(float dt);
        virtual void render();
        virtual void signal(UiElement *element, Event event);

    protected:
        UiButton *addButton(
            Action action,
            const std::string &text,
            const std::string &shortcut,
            bool momentary,
            const ysVector &accent,
            int row = 0);

        UiSlider *addSlider(
            int row,
            const std::string &label,
            double *value,
            double min,
            double max,
            const std::string &unit,
            const std::string &minText,
            const ysVector &accent);

        static constexpr int SliderRowCount = 2;
        static constexpr int ButtonRowCount = 2;

        std::vector<UiButton *> m_buttons;
        std::vector<Action> m_actions;
        std::vector<int> m_buttonRows;
        std::vector<UiSlider *> m_sliders;
        std::vector<int> m_sliderRows;
};

#endif /* ATG_ENGINE_SIM_CONTROL_BAR_H */
