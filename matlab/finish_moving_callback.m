function finish_moving_callback(src,evt,f)
    global still_moving;
    still_moving = false;
    uiresume(f);
end