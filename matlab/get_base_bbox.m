function base = get_base_bbox(bbox)
    base = abs(max(bbox(1,:))- min(bbox(1,:)))*...
        abs(max(bbox(3,:))- min(bbox(3,:)));
end