#include "button_state.h"

#include <cstdio>
#include <cstdlib>

static void Require(bool value, const char* message) {
    if (!value) {
        std::fprintf(stderr, "%s\n", message);
        std::exit(1);
    }
}

int main() {
    using Event = ButtonState::Event;
    ButtonState button;
    button.Update(true, true, 1'000'000'000);
    Require(button.IsDown() && button.lastEvent == Event::Pressed, "Press starts a hold");
    button.Update(true, true, 1'000'000'000);
    Require(button.lastEvent == Event::None, "Repeated sample must not restart a grip gesture");
    button.Update(true, false, 1'100'000'000);
    Require(!button.IsDown() && button.lastEvent == Event::ShortPress, "Short release emits once");
    button.Update(true, false, 1'100'000'000);
    Require(button.lastEvent == Event::None, "Repeated sample must not repeat a short press");

    button.Update(true, true, 2'000'000'000);
    button.Update(true, true, 2'249'999'999);
    Require(!button.IsLongHeld(), "Long press must respect the threshold");
    button.Update(true, true, 2'250'000'000);
    Require(button.IsLongHeld() && button.lastEvent == Event::LongPress, "Threshold crossing emits once");
    button.Update(true, true, 2'250'000'000);
    Require(button.IsLongHeld() && button.lastEvent == Event::None, "Same sample retains hold without repeating the event");
    button.Update(true, true, 2'500'000'000);
    Require(button.IsLongHeld() && button.lastEvent == Event::None, "Fresh held sample must not repeat the event");
    button.Update(true, false, 2'600'000'000);
    Require(!button.IsDown() && button.lastEvent == Event::None, "Long release must not become a short press");

    button.Update(true, true, 3'000'000'000);
    button.Update(false, true, 3'100'000'000);
    button.Update(true, true, 3'500'000'000);
    Require(!button.IsDown() && button.lastEvent == Event::None, "Inactive action cancels and blocks a held button");
    button.Update(true, false, 3'600'000'000);
    Require(button.lastEvent == Event::None, "Recovery release must not trigger an action");
    button.Update(true, true, 3'700'000'000);
    button.Update(true, false, 3'800'000'000);
    Require(button.lastEvent == Event::ShortPress, "Fresh press after recovery works");

    button.Update(true, true, 4'000'000'000);
    button.Cancel(); // Opening an overlay or switching action sets.
    button.Update(true, true, 4'000'000'000);
    Require(!button.IsDown(), "Cancellation also applies to an already consumed sample");
    button.Update(true, false, 4'100'000'000);
    Require(button.lastEvent == Event::None, "Cancelled release stays suppressed");

    button.Update(true, true, 5'000'000'000);
    button.Update(true, true, 1'000'000'000); // New XR clock/session.
    button.Update(true, true, 1'500'000'000);
    Require(!button.IsDown() && button.lastEvent == Event::None, "Time reversal must cancel history");
    button.Update(true, false, 1'600'000'000);
    button.Update(true, true, 1'700'000'000);
    button.Update(true, false, 1'800'000'000);
    Require(button.lastEvent == Event::ShortPress, "A new clock can rearm after release");
    button.Update(true, true, 0);
    Require(!button.IsDown() && button.lastEvent == Event::None, "Missing sample time cannot generate input");
    std::puts("Button timing, repeat, cancellation and recovery checks passed");
}
