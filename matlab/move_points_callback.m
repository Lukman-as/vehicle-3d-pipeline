function move_points_callback(src,evt,j,k,f)
    evname = evt.EventName;
    switch(evname)
        case{'ROIMoved'}
            disp(['ROI moved previous position: ' mat2str(evt.PreviousPosition)]);
            disp(['ROI moved current position: ' mat2str(evt.CurrentPosition)]);
            global annotations;
            annotations(:,14*(k-1)+j) = evt.CurrentPosition';
            global still_moving;
            still_moving = true;
    end
end