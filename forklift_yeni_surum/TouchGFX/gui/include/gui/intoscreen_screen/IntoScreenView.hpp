#ifndef INTOSCREENVIEW_HPP
#define INTOSCREENVIEW_HPP

#include <gui_generated/intoscreen_screen/IntoScreenViewBase.hpp>
#include <gui/intoscreen_screen/IntoScreenPresenter.hpp>

class IntoScreenView : public IntoScreenViewBase
{
public:
    IntoScreenView();
    virtual ~IntoScreenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();

protected:

private:
    int tickCounter;

    touchgfx::Callback<IntoScreenView, const touchgfx::AbstractButtonContainer&> touchCallback;
    void touchHandler(const touchgfx::AbstractButtonContainer& src);
};

#endif // INTOSCREENVIEW_HPP
