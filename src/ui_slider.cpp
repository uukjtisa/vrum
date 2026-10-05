#include "../include/ui_slider.h"

#include "../include/engine_sim_application.h"
#include "../include/ui_utilities.h"

#include <sstream>
#include <iomanip>

UiSlider::UiSlider() {
    m_checkMouse = true;
    m_accent = ysVector();
}

UiSlider::~UiSlider() {
    /* void */
}

void UiSlider::update(float dt) {
    m_mouseBounds = m_bounds;
}

double UiSlider::normalized() const {
    if (m_value == nullptr || m_max <= m_min) return 0.0;

    const double t = (*m_value - m_min) / (m_max - m_min);
    return (t < 0.0) ? 0.0 : ((t > 1.0) ? 1.0 : t);
}

void UiSlider::setFromMouse(const Point &mouseWorld) {
    if (m_value == nullptr) return;

    const float width = m_bounds.width();
    if (width <= 0.0f) return;

    // mouseOver() already resolved this element, so the incoming point is in
    // the same space m_bounds is expressed in -- no local transform needed.
    double t = (mouseWorld.x - m_bounds.left()) / width;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    *m_value = m_min + t * (m_max - m_min);
}

void UiSlider::onMouseDown(const Point &mouseLocal) {
    UiElement::onMouseDown(mouseLocal);
    setFromMouse(localToWorld(mouseLocal));
}

void UiSlider::onDrag(const Point &p0, const Point &mouse0, const Point &mouse) {
    // Deliberately NOT UiElement::onDrag, which moves the element itself.
    // `mouse` is already in world space here; see UiManager::update().
    if (isMouseHeld()) setFromMouse(mouse);
}

void UiSlider::render() {
    const ysVector background = m_app->getBackgroundColor();
    const ysVector foreground = m_app->getForegroundColor();
    const ysVector accent = m_hasAccent ? m_accent : m_app->getHightlight1Color();

    ysVector edge = mix(background, foreground, 0.25f);
    if (isMouseHeld()) edge = accent;
    else if (isMouseOver()) edge = mix(edge, accent, 0.6f);

    drawFrame(m_bounds, 1.0f, edge, background);

    // Label and value share one line above the track, so the whole control
    // fits in the same height as one row of buttons.
    const float trackHeight = 6.0f;
    const float pad = 6.0f;

    const float textY = m_bounds.top() - m_fontSize - 2.0f;
    Bounds labelBounds(
        m_bounds.left() + pad, m_bounds.right() - pad, textY, textY);
    drawAlignedText(m_label, labelBounds, m_fontSize, Bounds::bl, Bounds::bl);

    std::stringstream ss;
    if (m_value == nullptr) {
        ss << "--";
    }
    else if (!m_minText.empty() && *m_value <= m_min) {
        ss << m_minText;
    }
    else {
        ss << std::fixed << std::setprecision(2) << *m_value << m_unit;
    }

    drawAlignedText(ss.str(), labelBounds, m_fontSize, Bounds::br, Bounds::br);

    const float trackBottom = m_bounds.bottom() + pad;
    Bounds track(
        m_bounds.left() + pad, m_bounds.right() - pad,
        trackBottom, trackBottom + trackHeight);

    drawBox(track, mix(background, foreground, 0.18f));

    const float filledWidth = track.width() * (float)normalized();
    if (filledWidth > 0.5f) {
        Bounds filled(
            track.left(), track.left() + filledWidth,
            track.bottom(), track.top());
        drawBox(filled, accent);
    }

    // Handle, so it reads as draggable rather than as a progress bar.
    const float handleX = track.left() + filledWidth;
    Bounds handle(
        handleX - 3.0f, handleX + 3.0f,
        track.bottom() - 3.0f, track.top() + 3.0f);
    drawBox(handle, isMouseOver() || isMouseHeld() ? foreground : accent);
}
