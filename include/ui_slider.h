#ifndef ATG_ENGINE_SIM_UI_SLIDER_H
#define ATG_ENGINE_SIM_UI_SLIDER_H

#include "ui_element.h"

#include <string>

// VRUM: a horizontal drag slider.
//
// The upstream UI is read-only: every gauge displays, nothing accepts input,
// and every tunable is a hold-a-key-and-scroll binding. That is fine for the
// author of the code and useless to anyone else, so anything a user is expected
// to *set* rather than watch belongs here instead.
//
// Click anywhere on the track to jump, then keep dragging. The value is owned
// by the caller: the slider reads it every frame and writes back through
// m_value, so a keyboard binding and the slider can drive the same number
// without either going stale.
class UiSlider : public UiElement {
    public:
        UiSlider();
        virtual ~UiSlider();

        virtual void update(float dt);
        virtual void render();

        virtual void onMouseDown(const Point &mouseLocal);
        virtual void onDrag(const Point &p0, const Point &mouse0, const Point &mouse);

        // Pointer to the value being edited, in the caller's own units. Null
        // makes the slider inert rather than crashing, which matters because
        // the UI tree is rebuilt whenever the engine changes.
        double *m_value = nullptr;

        double m_min = 0.0;
        double m_max = 1.0;

        std::string m_label;
        // Appended to the formatted number, e.g. "s".
        std::string m_unit;
        // Shown instead of the number when the value is at m_min -- used to say
        // "instant" rather than "0.00s", which is the meaningful part.
        std::string m_minText;

        float m_fontSize = 13.0f;
        bool m_hasAccent = false;
        ysVector m_accent;

    protected:
        void setFromMouse(const Point &mouseWorld);

        double normalized() const;
};

#endif /* ATG_ENGINE_SIM_UI_SLIDER_H */
