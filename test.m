coords = [0, 0; 0, 10; 10, 10; 10, 0];


% Ananymous functions
velCtrl = @(v) v;
thCtrl = @(th) th;

testRegion = Region(coords, velCtrl, thCtrl);
testRegion.plot(gca, 'r');