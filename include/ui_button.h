#ifndef ATG_ENGINE_SIM_UI_BUTTON_H
#define ATG_ENGINE_SIM_UI_BUTTON_H

#include "ui_element.h"

class UiButton : public UiElement {
    public:
        UiButton();
        virtual ~UiButton();

        virtual void update(float dt);
        virtual void render();

        std::string m_text;
        float m_fontSize;

        // VRUM: state used by the control bar.
        //
        // m_active draws the button filled in its accent colour, so a toggle
        // reads as on or off at a glance instead of having to watch the log
        // line. m_momentary buttons (gear changes, engine cycling) never latch,
        // so they stay outlined.
        bool m_active = false;
        bool m_momentary = false;

        // Accent used when active; falls back to the theme highlight if unset.
        bool m_hasAccent = false;
        ysVector m_accent;

        // Dim caption under the label showing the keyboard equivalent, so the
        // buttons teach the shortcuts rather than replacing them.
        std::string m_shortcut;
};

#endif /* ATG_ENGINE_SIM_UI_BUTTON_H */
