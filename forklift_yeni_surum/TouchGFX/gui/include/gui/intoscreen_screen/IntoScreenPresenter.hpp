#ifndef INTOSCREENPRESENTER_HPP
#define INTOSCREENPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class IntoScreenView;

class IntoScreenPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    IntoScreenPresenter(IntoScreenView& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~IntoScreenPresenter() {}

private:
    IntoScreenPresenter();

    IntoScreenView& view;
};

#endif // INTOSCREENPRESENTER_HPP
