photons = csvread("photons.csv", 0, 0);
[uni,id] = unique(photons, 'rows');
photons_new = photons;
photons_new(id, :) = 0;
scatter3(photons_new(:,1), photons_new(:,2), photons_new(:,3));