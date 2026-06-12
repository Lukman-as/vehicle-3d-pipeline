function cMw = get_cMw_tire(sym_normal_unit,tire_extremal)
    origin_sym_coor = tire_extremal(1:3,3); %Near tireground first
    vec = tire_extremal(1:3,4)-tire_extremal(1:3,3);
    vec = vec/norm(vec);
    Rwc = [vec [0 1 0]' sym_normal_unit];
    Rcw = inv(Rwc);
    t_vector = -Rcw*origin_sym_coor;

    cMw = [Rcw t_vector; 0 0 0 1];
end