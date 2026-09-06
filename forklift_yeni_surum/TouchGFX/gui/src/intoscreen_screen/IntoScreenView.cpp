#include <gui/intoscreen_screen/IntoScreenView.hpp>

IntoScreenView::IntoScreenView() :
    touchCallback(this, &IntoScreenView::touchHandler)
{

}

void IntoScreenView::setupScreen()
{
    IntoScreenViewBase::setupScreen();

    // Ekrana dokunulduğunda ana sayfaya geçiş etkileşimi
    btnTouchAnywhere.setAction(touchCallback);
}

void IntoScreenView::tearDownScreen()
{
    IntoScreenViewBase::tearDownScreen();
}

void IntoScreenView::handleTickEvent()
{
    IntoScreenViewBase::handleTickEvent();
}

void IntoScreenView::touchHandler(const touchgfx::AbstractButtonContainer& src)
{
    application().gotoScreen1ScreenNoTransition();
}
