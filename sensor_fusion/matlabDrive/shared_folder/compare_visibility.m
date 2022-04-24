function visibility_score = compare_visibility(polygon,sector_polygon)



visibility_score = 1 - area(sector_polygon)/area(polygon);


end

