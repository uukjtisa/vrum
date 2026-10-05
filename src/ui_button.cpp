#include "../include/ui_button.h"

#include "../include/engine_sim_application.h"
#include "../include/ui_utilities.h"

UiButton::UiButton() {
    m_text = "";
    m_fontSize = 12;
    m_checkMouse = true;
    m_accent = ysVector();
}

UiButton::~UiButton() {
    /* void */
}

void UiButton::update(float dt) {
    m_mouseBounds = m_bounds;
}

void UiButton::render() {
    const ysVector background = m_app->getBackgroundColor();
    const ysVector foreground = m_app->getForegroundColor();
    const ysVector accent = m_hasAccent ? m_accent : m_app->getHightlight1Color();

    // An engaged toggle is filled with its accent and its label inverted to the
    // panel colour; everything else stays a quiet outline that only lifts off
    // the background on hover. That keeps a row of a dozen buttons from
    // shouting when none of them are on.
    ysVector fill = background;
    ysVector edge = mix(background, foreground, 0.25f);

    if (m_active && !m_momentary) {
        fill = accent;
        edge = accent;
    }

    if (isMouseHeld()) {
        fill = mix(fill, foreground, 0.18f);
        edge = accent;
    }
    else if (isMouseOver()) {
        fill = mix(fill, foreground, 0.08f);
        edge = mix(edge, accent, 0.6f);
    }

    drawFrame(m_bounds, 1.0f, edge, fill);

    if (m_shortcut.empty()) {
        drawCenteredText(m_text, m_bounds, m_fontSize);
        return;
    }

    // Label above centre, keyboard shortcut below it. Anchored on the box's
    // centre rather than the default top-middle, which places text outside the
    // frame -- UiButton was unused upstream, so that was never apparent.
    const float half = m_bounds.height() * 0.5f;
    const float labelY = m_bounds.bottom() + half + m_fontSize * 0.15f;
    const float shortcutY = m_bounds.bottom() + half - m_fontSize * 0.75f;

    Bounds labelBounds(m_bounds.left(), m_bounds.right(), labelY, labelY);
    drawCenteredText(m_text, labelBounds, m_fontSize, Bounds::center);

    Bounds shortcutBounds(
        m_bounds.left(), m_bounds.right(), shortcutY, shortcutY);
    drawCenteredText(
        m_shortcut, shortcutBounds, m_fontSize * 0.75f, Bounds::center);
}
