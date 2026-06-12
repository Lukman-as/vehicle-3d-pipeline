function a_point_corrected =  extend_a_point(all_annos,fieldname,a_point,dist_to_move,ds)
      dist = abs(dot(all_annos.(fieldname).sym_normal_unit,a_point)+ds);
      a_point_corrected_1 = a_point + dist_to_move*all_annos.(fieldname).sym_normal_unit;
      a_point_corrected_2 = a_point - dist_to_move*all_annos.(fieldname).sym_normal_unit;
      dist_1 = abs(dot(all_annos.(fieldname).sym_normal_unit,a_point_corrected_1)+ds);
      dist_2 = abs(dot(all_annos.(fieldname).sym_normal_unit,a_point_corrected_2)+ds);

      if dist_1 > abs(dist_2)
        a_point_corrected = a_point_corrected_1;
      else
        a_point_corrected = a_point_corrected_2;
      end
      dist_after = abs(dot(all_annos.(fieldname).sym_normal_unit,a_point_corrected)+ds);
      assert(dist_after > dist, 'Gotta move further away from the original plane');
end