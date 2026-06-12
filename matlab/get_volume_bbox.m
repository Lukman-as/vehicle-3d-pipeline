function volume = get_volume_bbox(bbox)
    height = abs(max(bbox(2,:))- min(bbox(2,:)));
    base = abs(max(bbox(1,:))- min(bbox(1,:)))*...
        abs(max(bbox(3,:))- min(bbox(3,:)));
    volume = base*height;
end